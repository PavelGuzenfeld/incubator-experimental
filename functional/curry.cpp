#include <concepts> // std::invocable
#include <functional> // std::invoke
#include <cassert>
#include <string>
#include <iostream> // std::cout

template <typename F, typename... Args>
constexpr auto curry(F f, Args... args)
{
    if constexpr (std::invocable<F, Args...>)
    {
        // Call function if possible
        return std::invoke(f, args...);
    }
    else
    {
        // Capture args in recursive call
        return [f, ... args = std::move(args)](auto &&...as) -> decltype(auto)
        {
            return curry(f, args..., std::forward<decltype(as)>(as)...);
        };
    }
}

// Generic compose function
template <typename F, typename G>
constexpr auto compose(F f, G g)
{
    return [f, g](auto &&...args) -> decltype(auto)
    {
        return g(f(std::forward<decltype(args)>(args)...));
    };
}

// Recursive composition of multiple functions
template <typename F, typename... Fs>
constexpr auto compose(F f, Fs... fs)
{
    return [f, composedFs = compose(fs...)](auto &&...args) -> decltype(auto)
    {
        return composedFs(f(std::forward<decltype(args)>(args)...));
    };
}

constexpr auto multiply(int x, int y) -> int
{
    return x * y;
}

constexpr auto add(int a, int b) -> int
{
    return a + b;
}

std::vector<int> filter( bool (*cond)(int),std::vector<int> data)
{
    std::vector<int> result;
    for (auto &&x : data)
    {
        if (cond(x))
        {
            result.push_back(x);
        }
    }
    return result;
}

std::vector<int> map( int (*trans)(int), std::vector<int> data)
{
    std::vector<int> result;
    for (auto &&x : data)
    {
        result.push_back(trans(x));
    }
    return result;
}

constexpr auto conditionA(int x) -> bool
{ 
    return x > 10;
}

constexpr auto transformationB(int x) -> int
{ 
    return x * 2;
}

constexpr int calculate(int a, int b, int c, int d)
{
    return a + b + c + d;
}

constexpr void sendEmail(std::vector<std::string> recipients, std::string subject, std::string body, std::string sender)
{
    // Send email
    // For demonstration, just print out the parameters
    std::cout << "Sending email to: ";
    for (const auto &recipient : recipients)
        std::cout << recipient << " ";
    std::cout << "\nSubject: " << subject << "\nBody: " << body << "\nFrom: " << sender << std::endl;
}

constexpr bool checkWithRules( std::vector<std::string> ruleset,int value)
{ 
    // Check value with ruleset
    return true;
}

int main()
{
    // Partial Function Application:
    constexpr auto times_two = curry(multiply, 2);
    constexpr auto times_three = curry(multiply, 3);
    constexpr  auto times_four = curry(multiply, 4);

    // Using curry for partial application
    static_assert(times_two(2) == 4);
    static_assert(times_three(3) == 9);
    static_assert(times_four(4) == 16);

    // Creating specialized functions
    constexpr auto addOne = curry(add, 1);
    constexpr auto addToTen = curry(add, 10);

    static_assert(addOne(1) == 2);
    static_assert(addToTen(10) == 20);

    // Functional Programming Paradigms
    // Currying to create a functional pipeline

    // Create partially applied functions
    auto filterWithConditionA = curry(filter, conditionA);
    auto mapWithTransformationB = curry(map, transformationB);

    // Compose the two functions
    auto processData = compose(filterWithConditionA, mapWithTransformationB);

    // Data and expected result
    std::vector<int> data{1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    std::vector<int> expected_map{22, 24, 26, 28, 30, 32, 34, 36, 38};

    // Apply the composed function
    auto result = processData(data);

    // Check the result
    assert(result == expected_map);


    // Test recursive composition
    constexpr auto composed = compose(times_two, times_three, times_four);
    static_assert(composed(2) == 48);

    // Test recursive composition with curry and vectors
    auto composed2 = compose(filterWithConditionA, mapWithTransformationB, mapWithTransformationB);
    auto result2 = composed2(data);
    std::vector<int> expected_map2{44, 48, 52, 56, 60, 64, 68, 72, 76};
    assert(result2 == expected_map2);

    // Test lazy evaluation
    // Partially apply the function
    constexpr auto  partiallyCalculated = curry(calculate, 1, 2);

    // Call the function with the remaining arguments
    static_assert(partiallyCalculated(3, 4) == 10);

    // Simplifying Function Interfaces
    std::vector<std::string> memoList = {"recipient@example.com"};
    auto sendMemo = curry(sendEmail, memoList, "Memo", std::placeholders::_1, "sender@example.com");

    // Simplified function call
    sendMemo("This is a memo body");

    // Higher-Order Functions
    std::vector<std::string> rulesetA = {"rule1", "rule2"};
    auto createValidator = curry(checkWithRules, rulesetA);

    assert(createValidator(1) == true);

    return 0;
}
