#include <iostream>
#include <string>
#include <tuple>
#include <utility> // for std::forward

using namespace std;

// Memento class: captures state (compile-time storage of state)
template <typename... States>
class Memento
{
private:
    std::tuple<States...> state; // Tuple to hold the state

public:
    // Constructor to initialize the memento state (taking lvalue references)
    constexpr Memento(const States &...args) : state(args...) {}

    // Method to access the state
    constexpr const std::tuple<States...> &getState() const { return state; }
};

// Originator class: The object whose state we want to save and restore
template <typename Derived, typename... States>
class Originator
{
private:
    std::tuple<States...> currentState; // Current state of the originator

protected:
    // Method to set the current state (using a memento)
    constexpr void restoreState(const Memento<States...> &memento)
    {
        currentState = memento.getState();
    }

    // Method to get the current state (create a memento)
    constexpr Memento<States...> saveState() const
    {
        return Memento<States...>(std::get<States>(currentState)...);
    }

public:
    // Allow Caretaker to access protected methods
    template <typename, typename...>
    friend class Caretaker;

    // Constructor to initialize the originator's state
    constexpr Originator(States &&...args) : currentState(std::forward<States>(args)...) {}

    // A method to manipulate the state
    template <std::size_t Index, typename ValueType>
    constexpr void setState(ValueType &&value)
    {
        std::get<Index>(currentState) = std::forward<ValueType>(value);
    }

    // A method to print the current state
    constexpr void printState() const
    {
        apply([](const auto &...args)
              { ((cout << args << " "), ...); }, currentState);
        cout << endl;
    }
};

// Caretaker class: Manages saving and restoring mementos
template <typename OriginatorType, typename... States>
class Caretaker
{
private:
    OriginatorType &originator; // The originator object
    Memento<States...> memento; // Stored memento

public:
    // Constructor to initialize the caretaker with an originator
    constexpr Caretaker(OriginatorType &originator)
        : originator(originator), memento(originator.saveState()) {}

    // Save the current state of the originator
    constexpr void save()
    {
        memento = originator.saveState();
    }

    // Restore the originator to the previously saved state
    constexpr void restore()
    {
        originator.restoreState(memento);
    }
};

// Example usage with compile-time Memento pattern
class MyOriginator : public Originator<MyOriginator, int, double, string>
{
public:
    using Originator<MyOriginator, int, double, string>::Originator; // Inherit constructor
};

int main()
{
    // Initialize the originator with state (int, double, string)
    MyOriginator originator(10, 3.14, "Hello");

    // Create a caretaker to manage the originator's state
    Caretaker<MyOriginator, int, double, string> caretaker(originator);

    // Print the initial state
    cout << "Initial State: ";
    originator.printState(); // Output: 10 3.14 Hello

    // Modify the state of the originator
    originator.setState<0>(42);      // Set int state to 42
    originator.setState<1>(2.718);   // Set double state to 2.718
    originator.setState<2>("World"); // Set string state to "World"

    // Print the modified state
    cout << "Modified State: ";
    originator.printState(); // Output: 42 2.718 World

    // Restore the originator's state to the saved state
    caretaker.restore();

    // Print the restored state
    cout << "Restored State: ";
    originator.printState(); // Output: 10 3.14 Hello

    return 0;
}
