#include <fmt/core.h>
#include <fmt/ostream.h>

#include <algorithm>
#include <chrono>
#include <expected>
#include <functional>
#include <future>
#include <string>
#include <thread>
#include <vector>

template <>
struct fmt::formatter<std::thread::id> : fmt::ostream_formatter
{
};

struct Error
{
    std::string message;
};

template <typename T>
class Continuation
{
private:
    std::future<std::expected<T, Error>> m_future;

public:
    Continuation(std::future<std::expected<T, Error>> &&fut) : m_future(std::move(fut)) {}

    // Chain an operation on the SUCCESS path.
    template <typename Func>
    auto and_then(Func &&func)
    {
        using ResultType = std::invoke_result_t<Func, T>;
        return Continuation<typename ResultType::value_type>(
            std::async(std::launch::async,
                       [f = std::forward<Func>(func), fut = std::move(m_future)]() mutable -> ResultType
                       {
                           auto expected = fut.get();
                           return expected ? f(expected.value()) : std::unexpected(expected.error());
                       }));
    }

    // Attempt to RECOVER from an error on the FAILURE path.
    template <typename Func>
    auto or_else(Func &&func)
    {
        using ResultType = std::invoke_result_t<Func, Error>;
        return Continuation<T>(
            std::async(std::launch::async,
                       [f = std::forward<Func>(func), fut = std::move(m_future)]() mutable -> std::expected<T, Error>
                       {
                           auto expected = fut.get();
                           return expected ? expected : f(expected.error());
                       }));
    }

    // Recover from an error with a DEFAULT VALUE.
    auto or_value(T default_value)
    {
        return or_else([val = std::move(default_value)](const Error &) -> std::expected<T, Error>
                       {
            fmt::print("TID [{}]: Recovering from error with default value...\n", std::this_thread::get_id());
            return val; });
    }

    std::expected<T, Error> get() { return m_future.get(); }
};

// Helper to start a continuation chain
template <typename Func, typename... Args>
auto run_async(Func &&func, Args &&...args)
{
    using ResultType = std::invoke_result_t<Func, Args...>;
    return Continuation<typename ResultType::value_type>(
        std::async(std::launch::async, std::forward<Func>(func), std::forward<Args>(args)...));
}

// --- Example Functions ---
std::expected<std::string, Error> fetch_user(int id)
{
    fmt::print("TID [{}]: Fetching user {}...\n", std::this_thread::get_id(), id);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (id == 101)
    {
        return "Alex";
    }
    else
    {
        return std::unexpected(Error{fmt::format("User {} not found", id)});
    }
}

std::expected<std::string, Error> to_uppercase(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), ::toupper);
    return s;
}

int main()
{
    std::vector<Continuation<std::string>> pipelines;

    // Pipeline 1: Success path using and_then
    pipelines.emplace_back(
        run_async(fetch_user, 101)
            .and_then(to_uppercase));

    // Pipeline 2: Failure path short-circuits and_then
    pipelines.emplace_back(
        run_async(fetch_user, 999)
            .and_then(to_uppercase) // This will be skipped
    );

    // Pipeline 3: Failure recovered with or_value, then continues with and_then
    pipelines.emplace_back(
        run_async(fetch_user, 999)
            .or_value("GUEST")      // Recovers from the error
            .and_then(to_uppercase) // Now this will run
    );

    // Pipeline 4: Failure recovered with or_else attempting a second fetch
    pipelines.emplace_back(
        run_async(fetch_user, 999)
            .or_else([](const Error &e)
                     {
                         fmt::print("TID [{}]: In or_else, received error: '{}'. Retrying with user 101...\n", std::this_thread::get_id(), e.message);
                         return fetch_user(101); // Attempt recovery
                     })
            .and_then(to_uppercase));

    fmt::print("Main TID [{}]: All pipelines launched. Waiting for results...\n\n", std::this_thread::get_id());

    for (size_t i = 0; i < pipelines.size(); ++i)
    {
        auto result = pipelines[i].get();
        fmt::print("--- Pipeline {} Result ---\n", i + 1);
        if (result)
        {
            fmt::print("✅ Success: '{}'\n\n", *result);
        }
        else
        {
            fmt::print("❌ Failure: '{}'\n\n", result.error().message);
        }
    }

    return 0;
}