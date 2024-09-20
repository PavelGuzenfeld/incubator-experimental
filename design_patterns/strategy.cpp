#include <iostream>
#include <string>
using namespace std;

// Strategy interface (generic template)
template <typename T>
struct Strategy
{
    // Pure virtual method to be implemented by specializations
    virtual void execute(T data) const = 0;
};

// Specialization of Strategy for integers
template <>
struct Strategy<int>
{
    void execute(int data) const
    {
        cout << "Executing integer strategy with value: " << data << endl;
    }
};

// Specialization of Strategy for strings
template <>
struct Strategy<string>
{
    void execute(string data) const
    {
        cout << "Executing string strategy with value: " << data << endl;
    }
};

// Context class that uses the strategy
template <typename T, template <typename> class StrategyType = Strategy>
class Context
{
    const StrategyType<T> &strategy;

public:
    // Constructor takes a strategy as input
    Context(const StrategyType<T> &strategy) : strategy(strategy) {}

    // Call the execute method of the strategy
    void executeStrategy(T data) const
    {
        strategy.execute(data);
    }
};

// Main function demonstrating compile-time strategy selection
int main()
{
    Strategy<int> intStrategy;       // Integer strategy
    Strategy<string> stringStrategy; // String strategy

    Context<int> intContext(intStrategy);
    intContext.executeStrategy(42); // Outputs: Executing integer strategy with value: 42

    Context<string> stringContext(stringStrategy);
    stringContext.executeStrategy("Hello"); // Outputs: Executing string strategy with value: Hello

    return 0;
}
