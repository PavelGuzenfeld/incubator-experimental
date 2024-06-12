// I would like to execrise all the capabilities of the ranges library
// I will use the ranges library to implement the following functions
// 1. filter
// 2. transform
// 3. action
// 4. join
// 5. view
// 6. take
// 7. drop
// 8. reverse
// 9. sort
// 10. unique
// 11. for_each
// 12. find
// 13. count
// 14. equal

// I will use catch2 to test the functions
// The code should be constexpr

#include <algorithm>
#include <catch2/catch.hpp>
#include <ranges>
#include <vector>

constexpr auto is_even = [](const auto &i)
{ return i % 2 == 0; };

constexpr auto square = [](const auto &i)
{ return i * i; };

constexpr auto is_equal = [](const auto &a, const auto &b)
{ return a.first == b.first && a.second == b.second; };

constexpr auto are_both_devidede_by_2 = [](const auto &a, const auto &b)
{
    return is_even(a) && is_even(b);
};

// 1. filter
TEST_CASE("filter")
{
    const auto arr = std::array<int, 10>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto evens = arr | std::views::filter(is_even);

    // Convert initializer list to a range
    std::array<int, 5> expected = {2, 4, 6, 8, 10};

    // Compare the filtered view with the range
    REQUIRE(std::ranges::equal(evens, expected));
}

// 2. transform
TEST_CASE("transform")
{
    std::array<int, 10> arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto square = [](int i)
    { return i * i; };
    auto squares = arr | std::views::transform(square);

    // Convert initializer list to a range
    std::array<int, 10> expected = {1, 4, 9, 16, 25, 36, 49, 64, 81, 100};

    // Compare the transformed view with the range
    REQUIRE(std::ranges::equal(squares, expected));
}

// 3. filter and transform

TEST_CASE("filter and transform")
{
    std::array<int, 10> arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto even_squares = arr | std::views::filter(is_even) | std::views::transform(square);

    // Convert initializer list to a range
    std::array<int, 5> expected = {4, 16, 36, 64, 100};

    // Compare the filtered and transformed view with the range
    REQUIRE(std::ranges::equal(even_squares, expected));
}

// Cartesian product
TEST_CASE("cartesian product")
{
    std::array<int, 3> arr1 = {1, 2, 3};
    std::array<int, 3> arr2 = {4, 5, 6};
    auto product = std::views::cartesian_product(arr1, arr2);

    // Convert initializer list to a range
    std::array<std::pair<int, int>, 9> expected = {
        std::pair{1, 4}, std::pair{1, 5}, std::pair{1, 6}, std::pair{2, 4}, std::pair{2, 5}, std::pair{2, 6}, std::pair{3, 4}, std::pair{3, 5}, std::pair{3, 6}};

    REQUIRE(product.size() == expected.size());
    REQUIRE(std::equal(product.begin(), product.end(), expected.begin(), expected.end(), is_equal));
}

TEST_CASE("chunk by")
{
    std::array<int, 10> arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto chunks = arr | std::views::chunk_by(are_both_devidede_by_2);

    REQUIRE(!chunks.empty());
    auto got_no_chunks = std::ranges::all_of(chunks, [](const auto &chunk)
                                             { return chunk.size() == 1; });
    REQUIRE(got_no_chunks);
}

TEST_CASE("zip")
{
    std::array<int, 3> arr1 = {1, 2, 3};
    std::array<int, 3> arr2 = {4, 5, 6};
    auto zipped = std::views::zip(arr1, arr2);

    // Convert initializer list to a range
    std::array<std::pair<int, int>, 3> expected = {
        std::pair{1, 4}, std::pair{2, 5}, std::pair{3, 6}};

    REQUIRE(zipped.size() == expected.size());
    REQUIRE(std::equal(zipped.begin(), zipped.end(), expected.begin(), expected.end(), is_equal));
}

TEST_CASE("zip_transform")
{
    std::array<int, 3> arr1 = {1, 2, 3};
    std::array<int, 3> arr2 = {4, 5, 6};
    auto zipped = std::views::zip(arr1, arr2) | std::views::transform([](const auto &a)
                                                                      { return a.first + a.second; });

    // Convert initializer list to a range
    std::array<int, 3> expected = {5, 7, 9};

    REQUIRE(zipped.size() == expected.size());
    REQUIRE(std::equal(zipped.begin(), zipped.end(), expected.begin(), expected.end()));
}

TEST_CASE("ranges::to")
{
    //Works in compiler explorer
    // std::array<int, 10> arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    // auto evens = arr | std::views::filter(is_even) | std::ranges::to<std::vector>();

    // // Convert initializer list to a range
    // std::vector<int> expected = {2, 4, 6, 8, 10};

    // // Compare the filtered view with the range
    // REQUIRE(std::ranges::equal(evens, expected));
}


TEST_CASE("join strings")
{
    std::array<std::string, 3> arr = {"Hello", "World", "!"};
    auto joined = arr | std::views::join;

    // Convert initializer list to a range
    std::string_view expected = "HelloWorld!";

    auto result = std::accumulate(joined.begin(), joined.end(), std::string{}); // missing operator == for joined
    REQUIRE(result == expected);
}

TEST_CASE("sum of squares")
{
   
    const int N = 10;
    const int EXPECTED = 385;
    // iota(1) generates an infinite range starting from 1
    // transform(square) squares each element
    // take(N) takes the first N elements
    // join flattens the nested ranges
    auto sum_of_squares = std::views::iota(1) | std::views::transform(square) | std::views::take(N);

    auto result = std::ranges::fold_left(sum_of_squares, 0, std::plus{});
    REQUIRE(result == EXPECTED);
}
