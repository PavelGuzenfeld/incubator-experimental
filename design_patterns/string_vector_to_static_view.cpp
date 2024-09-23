#include <algorithm>
#include <array>
#include <iostream>
#include <ranges>
#include <string>
#include <vector>

template <auto Builder, std::size_t MaxSize>
constexpr auto to_view()
{
    // constexpr tuple of oversized things
    // we don't want this static yet
    constexpr auto data = [&]
    {
        const auto input = Builder();

        // we have no idea how big this will ultimately need to be
        // so we're taking the passed-in MaxSize per unit, and squaring it
        std::array<char, MaxSize * MaxSize> allchars{};

        std::array<std::size_t, MaxSize> string_lengths{};

        // save all of the string lengths and the set of packed characters
        auto current = allchars.begin();
        for (std::size_t index = 0; const auto &str : input)
        {
            current = std::ranges::copy(str, current).out;
            string_lengths[index++] = str.size();
        }
        const auto total_chars = std::distance(allchars.begin(), current);

        return std::tuple{input.size(), total_chars, allchars, string_lengths};
    }();

    constexpr auto total_chars = std::get<1>(data);

    // this is the only static thing, as it is the packed
    // set of characters that make up all of the strings, truncated
    // to exactly the right size
    static constexpr auto right_sized_chars = [&]
    {
        // this is the right-sized array, based on the previously
        // computed total string length
        std::array<char, total_chars> result;
        auto &allchars = std::get<2>(data);
        std::ranges::copy(allchars | std::views::take(total_chars), result.begin());
        return result;
    }();

    // now build and return an array of string_views into
    // that array of chars, based on the string sizes
    // that were saved in the very first IILE
    constexpr auto num_strings = std::get<0>(data);
    std::array<std::string_view, num_strings> views;
    std::size_t start = 0;
    auto &string_lengths = std::get<3>(data);
    for (std::size_t index = 0; index < num_strings; ++index)
    {
        const auto size = string_lengths[index];
        views[index] = std::string_view(right_sized_chars.begin() + start,
                                        right_sized_chars.begin() + start + size);
        start += size;
    }

    // for mega bonus points, we could probably de-duplicate strings and build
    // a new string table that reuses existing strings for the
    // string_views...

    return views;
}

constexpr std::vector<std::string> get_strings()
{
    // just return anything here, but check the very last
    // line of the assembly output to see the packed string table

    // note: the resulting table does not have null terminated strings
    // because string_view isn't null terminated. We could
    // make our own null-terminated string-view like thing, but I didn't
    return {"Jason", "Was", "Here"};
}

int main()
{
    // a constexpr array of string_views into the statically
    // allocated packed set of characters that makes up the
    // strings
    constexpr auto strings = to_view<get_strings, 255>();

    for (const auto &string : strings)
    {
        std::cout << string << '\n';
    }
}