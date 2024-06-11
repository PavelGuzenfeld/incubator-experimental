#include <cmath>
#include <cstdint>
#include <fplus/fplus.hpp>
#include <iostream>

using Point = std::pair<double, double>;
using Polygon = std::vector<Point>;

double calculate_edge_length(const Point &p1, const Point &p2)
{
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

double find_longest_edge(const Polygon &polygon)
{
    const auto edges = fplus::overlapping_pairs_cyclic(polygon);
    const auto edge_lengths = fplus::transform([](const auto &edge)
                                               { return calculate_edge_length(edge.first, edge.second); },
                                               edges);
    return fplus::maximum_on(fplus::identity<double>, edge_lengths);
}

int main()
{
    const Polygon polygon = {{1, 2}, {7, 3}, {6, 5}, {4, 4}, {2, 9}};
    const auto longest_edge = find_longest_edge(polygon);
    std::cout << longest_edge << std::endl;
}