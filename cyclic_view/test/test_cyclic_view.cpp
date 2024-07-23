#define CATCH_CONFIG_MAIN
#include "cyclic_view/cyclic_view.hpp"
#include <catch2/catch.hpp>

using namespace cyclic;

TEST_CASE("CyclicView with std::array")
{
    std::array<int, 5> arr = {1, 2, 3, 4, 5};
    CyclicView view(arr, arr.size(), 0);

    std::vector<int> result(view.begin(), view.end());
    REQUIRE(result == std::vector<int>{1, 2, 3, 4, 5});

    // Test indexing
    REQUIRE(view[0] == 1);
    REQUIRE(view[1] == 2);
    REQUIRE(view[2] == 3);
    REQUIRE(view[3] == 4);
    REQUIRE(view[4] == 5);
    REQUIRE(view[5] == 1);
    REQUIRE(view[6] == 2);
    REQUIRE(view[7] == 3);
    REQUIRE(view[8] == 4);

    // Test size
    REQUIRE(view.size() == 5);

    // Test push
    view.push(6);
    auto result_push_6 = {2, 3, 4, 5, 6};
    REQUIRE(view.size() == 5);
    REQUIRE(std::equal(result_push_6.begin(), result_push_6.end(), view.begin()));

    view.push(7);
    auto result_push_7 = {3, 4, 5, 6, 7};
    REQUIRE(view.size() == 5);
    REQUIRE(std::equal(result_push_7.begin(), result_push_7.end(), view.begin()));

    view.push(8);
    auto result_push_8 = {4, 5, 6, 7, 8};
    REQUIRE(view.size() == 5);
    REQUIRE(std::equal(result_push_8.begin(), result_push_8.end(), view.begin()));
}

TEST_CASE("CyclicView with C-style array") {
    int c_arr[] = {6, 7, 8, 9, 10};
    std::size_t size = sizeof(c_arr) / sizeof(c_arr[0]);
    CyclicView view(c_arr, size, size);

    std::vector<int> result(view.begin(), view.end());
    REQUIRE(result == std::vector<int>{6, 7, 8, 9, 10});
}

TEST_CASE("CyclicView with std::vector")
{
    std::vector<int> vec = {11, 12, 13, 14, 15};
    CyclicView view_vec(vec, vec.size());

    std::vector<int> result(view_vec.begin(), view_vec.end());
    REQUIRE(result == std::vector<int>{11, 12, 13, 14, 15});
}

TEST_CASE("Pushing data into CyclicView")
{
    std::array<int, 3> push_arr = {0, 0, 0};
    CyclicView push_view(push_arr, push_arr.size());

    // Pushing elements
    push_view.push(1);
    push_view.push(2);
    push_view.push(3);
    push_view.push(4); // This should overwrite the first element

    std::vector<int> result(push_view.begin(), push_view.end());
    REQUIRE(result == std::vector<int>{2, 3, 4});
}

TEST_CASE("CyclicView with growing container")
{
    std::vector<int> vec = {1, 2, 3, 4, 5};
    CyclicView view(vec, 6);

    // Initial view check
    std::vector<int> result(view.begin(), view.end());
    REQUIRE(result == std::vector<int>{1, 2, 3, 4, 5});

    // Grow the underlying container
    vec.push_back(6);
    REQUIRE(vec.size() == 6);
    REQUIRE(vec == std::vector<int>{1, 2, 3, 4, 5, 6});

    // View should now handle the larger container correctly
    view.push(7);
    auto result_push_7 = {2, 3, 4, 5, 6, 7};
    REQUIRE(view.size() == 6);
    REQUIRE(std::equal(result_push_7.begin(), result_push_7.end(), view.begin()));

    // Push more elements
    view.push(8);
    auto result_push_8 = {3, 4, 5, 6, 7, 8};
    REQUIRE(view.size() == 6);
    REQUIRE(std::equal(result_push_8.begin(), result_push_8.end(), view.begin()));
}
