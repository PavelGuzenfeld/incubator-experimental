#include <iostream>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

// Helper to process each step in the chain at compile time using recursion
template <typename T, typename Func, typename... Rest>
constexpr auto process_chain(int n, const Func &first, const Rest &...rest) -> std::optional<T>
{
    if (auto result = first(n); result.has_value())
    {
        return result;
    }
    else if constexpr (sizeof...(rest) > 0)
    {
        return process_chain<T>(n, rest...);
    }
    else
    {
        return std::nullopt;
    }
}

// Helper to unpack a tuple and apply it to the process_chain function
template <typename T, typename... Funcs>
constexpr auto make_chain(std::tuple<Funcs...> steps)
{
    return [=](int n) constexpr -> std::optional<T>
    {
        return std::apply([n](const auto &...funcs) constexpr
                          { return process_chain<T>(n, funcs...); }, steps);
    };
}

int main()
{
    using namespace std::literals;

    // Create the fizz_buzz chain with constexpr lambdas
    constexpr auto fizz_buzz = make_chain<std::string>(std::make_tuple(
        [](int n) constexpr
        { return n % 15 == 0 ? std::optional{"FizzBuzz"s} : std::nullopt; },
        [](int n) constexpr
        { return n % 3 == 0 ? std::optional{"Fizz"s} : std::nullopt; },
        [](int n) constexpr
        { return n % 5 == 0 ? std::optional{"Buzz"s} : std::nullopt; },
        [](int n) constexpr
        { return std::optional{std::to_string(n)}; }));

    // Run the FizzBuzz chain for numbers 1 to 100
    for (int i = 1; i <= 100; ++i)
    {
        std::cout << i << ": " << *fizz_buzz(i) << "\n";
    }
}
