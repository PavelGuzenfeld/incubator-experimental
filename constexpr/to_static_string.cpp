#include <array>
#include <fmt/format.h>
#include <string>

constexpr auto CONSTEXPR_BUFFER = 10 * 1024 * 1024;
using OVERSIZED_ARRAY = std::pair<std::array<char, CONSTEXPR_BUFFER>, std::size_t>;

constexpr auto to_oversized_array(std::string const &input)
{
    OVERSIZED_ARRAY result;
    std::copy(input.begin(), input.end(), result.first.begin());
    result.second = input.size();
    return result;
}

template <typename CALLABLE>
consteval auto right_size_array(CALLABLE cb)
{
    constexpr auto oversized = to_oversized_array(cb());
    std::array<char, oversized.second> result;
    std::copy(oversized.first.begin(),
              std::next(oversized.first.begin(), oversized.second), result.begin());
    return result;
}

template <auto DATA>
consteval auto const &make_static()
{
    return DATA;
}

consteval auto to_string_view(auto callable) -> std::string_view
{
    constexpr auto &static_data = make_static<right_size_array(callable)>();
    return {static_data.begin(), static_data.end()};
}

constexpr std::string make_string(std::string_view base, int const repeat)
{
    std::string retval;
    for (unsigned int count = 0; count < repeat; ++count)
    {
        retval += base;
    }

    return retval;
}

int main()
{
    constexpr auto make_data = []
    { return make_string("Hello Jason, ", 3); };
    constexpr auto sv = to_string_view(make_data);
    fmt::print("{}: {}", sv.size(), sv);
    return 0;
}