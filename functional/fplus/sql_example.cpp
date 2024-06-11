#include <cstdint>
#include <fplus/fplus.hpp>

struct user
{
    std::string name;
    std::string country;
    std::size_t visits;
};

std::string get_country(const user &u)
{
    return u.country;
}

std::size_t get_visits(const user &u)
{
    return u.visits;
}

int main()
{
    const std::vector<user> users = {
        {"Nicole", "GER", 2},
        {"Justin", "USA", 1},
        {"Rachel", "USA", 5},
        {"Robert", "USA", 6},
        {"Stefan", "GER", 4}};

    // SELECT country, SUM(visits)
    //     FROM users
    //     GROUP BY country;

    // Exercise:
    //     Implement the task solved by the SQL query above
    //     in C++ using the following fplus functions:
    //     - group_globally_on_labeled
    //     - transform_snd
    //     - sum
    //     Look them up in the API search,
    //     and understand the type signatures.

    // Bonus Exercise:
    //     The run-time complexity of your algorithm will be O(n^2),
    //     due to using group_globally_on_labeled.
    //     But Strings form a partially ordered set.
    //     See if you can get down to O(n*log(n))
    //     by using sort and group_on_labeled.

    // Solution:

    // const auto result = fplus::fwd::apply(users,
    //                                       // Group users by country
    //                                       fplus::fwd::group_globally_on_labeled(get_country),
    //                                       // For each group, sum the visits
    //                                       fplus::fwd::transform([](const auto &group)
    //                                                             { return std::make_pair(group.first, fplus::sum(fplus::transform(get_visits, group.second))); }));

    // std::cout << fplus::show(result) << std::endl;

    // const auto result = fplus::fwd::apply(
    //     users,
    //     // Sort users by country using a custom comparator
    //     fplus::fwd::sort_by([](const user &a, const user &b)
    //                         { return get_country(a) < get_country(b); }),
    //     // Group sorted users by country
    //     fplus::fwd::group_on_labeled(get_country),
    //     // For each group, sum the visits
    //     fplus::fwd::transform([](const auto &group)
    //                           { return std::make_pair(group.first, fplus::sum(fplus::transform(get_visits, group.second))); }));

    const auto conpose_sql_query = fplus::fwd::compose(
        fplus::fwd::sort_by([](const user &a, const user &b)
                            { return get_country(a) < get_country(b); }),
        fplus::fwd::group_on_labeled(get_country),
        fplus::fwd::transform([](const auto &group)
                              { return std::make_pair(group.first, fplus::sum(fplus::transform(get_visits, group.second))); }));

    const auto result = conpose_sql_query(users);

    std::cout << fplus::show(result) << std::endl;
}