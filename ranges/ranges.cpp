//I would like to execrise all the capabilities of the ranges library
//I will use the ranges library to implement the following functions
//1. filter
//2. transform
//3. action
//4. join
//5. view
//6. take
//7. drop
//8. reverse
//9. sort
//10. unique
//11. for_each
//12. find
//13. count
//14. equal

//I will use catch2 to test the functions
//The code should be constexpr

#include <catch2/catch.hpp>
#include <algorithm>
#include <ranges>


//1. filter
TEST_CASE("filter")
{
 const auto arr =  std::array<int, 10>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto even = [](int i)
    { return i % 2 == 0; };
    auto evens = arr | std::views::filter(even);

    // Convert initializer list to a range
    std::array<int, 5> expected = {2, 4, 6, 8, 10};

    // Compare the filtered view with the range
    REQUIRE(std::ranges::equal(evens, expected));
}

//2. transform
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

//3. filter and transform

TEST_CASE("filter and transform")
{
    std::array<int, 10> arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    auto even = [](int i)
    { return i % 2 == 0; };
    auto square = [](int i)
    { return i * i; };
    auto even_squares = arr | std::views::filter(even) | std::views::transform(square);

    // Convert initializer list to a range
    std::array<int, 5> expected = {4, 16, 36, 64, 100};

    // Compare the filtered and transformed view with the range
    REQUIRE(std::ranges::equal(even_squares, expected));
}