#pragma once
#include <type_traits> // std::is_arithmetic_v
#include <numeric> // std::transform_reduce
#include <concepts> // std::convertible_to

template<typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

template<typename T>
concept ArithmeticPair = requires(T a) {
    { std::get<0>(a) } -> std::convertible_to<Arithmetic>;
    { std::get<1>(a) } -> std::convertible_to<Arithmetic>;
};

// Concept to check if a type has begin and end methods, and the value type is an ArithmeticPair
template<typename T>
concept ArithmeticPairCollection = requires(T a) {
    { a.begin() } -> std::same_as<decltype(a.begin())>;
    { a.end() } -> std::same_as<decltype(a.end())>;
    { a.size() } -> std::convertible_to<std::size_t>;
    requires ArithmeticPair<typename std::iterator_traits<decltype(a.begin())>::value_type>;
};

constexpr auto calculate_average_difference(const ArithmeticPairCollection auto& collection) {
    double sum =  std::transform_reduce(collection.begin(), collection.end(), 0.0, std::plus<>(), [](const auto& pair) {
        auto [first, second] = pair;
        return first - second;
    });
    double size = static_cast<double>(collection.size());
    double avg = sum / size;
    return avg;
}