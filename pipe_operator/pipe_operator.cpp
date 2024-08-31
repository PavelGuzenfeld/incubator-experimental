#include <optional>
#include <type_traits>
#include <utility>

template <typename In, typename Func, typename = std::enable_if_t<std::is_invocable_v<Func, In>>>
constexpr auto operator|(std::optional<In> &&input, Func func)
{
    using ResultType = decltype(func(*input));

    if (input)
    {
        if constexpr (std::is_same_v<ResultType, void>)
        {
            func(*input);
            return std::move(input); // Return the original input without re-wrapping
        }
        else
        {
            return std::optional<ResultType>{func(*input)};
        }
    }
    return std::optional<ResultType>{std::nullopt}; // Ensure correct type deduction
}

template <typename In, typename Func, typename = std::enable_if_t<std::is_invocable_v<Func, In>>>
constexpr auto operator|(const std::optional<In> &input, Func func)
{
    using ResultType = decltype(func(*input));

    if (input)
    {
        if constexpr (std::is_same_v<ResultType, void>)
        {
            func(*input);
            return std::optional<ResultType>{*input};
        }
        else
        {
            return std::optional<ResultType>{func(*input)};
        }
    }
    return std::optional<ResultType>{std::nullopt};
}

int main()
{
    int one = 1;

    auto add_one = [one](int x) -> int
    {
        return x + one;
    };

    auto to_string = [](int x) -> char
    {
        return static_cast<char>(x);
    };

    std::optional<int> result = 5;

    auto final_result = result | add_one | to_string;

    return final_result.value_or(-1);
}
