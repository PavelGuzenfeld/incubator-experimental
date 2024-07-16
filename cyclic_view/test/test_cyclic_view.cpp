#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "cyclic_view.hpp"

using namespace cyclic;

TEST_CASE("CyclicView with std::array") {
    std::array<int, 5> arr = {1, 2, 3, 4, 5};
    CyclicView view(arr, 0, arr.size());

    std::vector<int> result;
    for (const auto& elem : view) {
        result.push_back(elem);
    }
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
    auto c_view = make_cyclic_view(c_arr);

    std::vector<int> result;
    for (const auto& elem : c_view) {
        result.push_back(elem);
    }
    REQUIRE(result == std::vector<int>{6, 7, 8, 9, 10});
}

TEST_CASE("CyclicView with std::vector") {
    std::vector<int> vec = {11, 12, 13, 14, 15};
    CyclicView view_vec(vec, 0, vec.size());

    std::vector<int> result;
    for (const auto& elem : view_vec) {
        result.push_back(elem);
    }
    REQUIRE(result == std::vector<int>{11, 12, 13, 14, 15});
}

TEST_CASE("Pushing data into CyclicView") {
    std::array<int, 3> push_arr = {0, 0, 0};
    CyclicView push_view(push_arr, 0, 0);

    // Pushing elements
    push_view.push(1);
    push_view.push(2);
    push_view.push(3);
    push_view.push(4); // This should overwrite the first element

    std::vector<int> result;
    for (const auto& elem : push_view) {
        result.push_back(elem);
    }
    REQUIRE(result == std::vector<int>{4, 2, 3});
}
