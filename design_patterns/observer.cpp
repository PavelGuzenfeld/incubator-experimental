#include <array>
#include <functional> // for std::reference_wrapper
#include <iostream>
#include <utility> // for std::forward

using namespace std;

// CRTP (Curiously Recurring Template Pattern) for compile-time polymorphism.
template <typename T, typename Derived>
class Observer
{
public:
    // Perfect forwarding for notify arguments
    template <typename... Args>
    constexpr void onNotify(Args &&...args) const
    {
        // Correctly cast `this` to the derived class and call the derived implementation of `onNotify`
        static_cast<const Derived *>(this)->onNotify(std::forward<Args>(args)...);
    }
};

// Observable class with a static list of observers using references.
template <typename T, typename Derived, size_t N>
class Observable
{
private:
    const array<reference_wrapper<Observer<T, Derived>>, N> observers; // Static list of observer references

public:
    // Constructor accepts a list of observers
    constexpr Observable(array<reference_wrapper<Observer<T, Derived>>, N> &&observersList)
        : observers(std::move(observersList)) {}

    // Notify all observers with perfect forwarding
    template <typename... Args>
    constexpr void notify(Args &&...args) const
    {
        for (const auto &observer : observers)
        {
            observer.get().onNotify(std::forward<Args>(args)...); // Use `get()` to access the underlying observer
        }
    }
};

// Helper function to create Observable using variadic templates
template <typename T, typename Derived, typename... Observers>
constexpr auto make_observable(Observers &...observers)
{
    return Observable<T, Derived, sizeof...(Observers)>{
        array<reference_wrapper<Observer<T, Derived>>, sizeof...(Observers)>{observers...}};
}

// Concrete observer for integer events (using CRTP)
class IntegerObserver : public Observer<int, IntegerObserver>
{
public:
    void onNotify(int event) const
    {
        cout << "IntegerObserver received event: " << event << endl;
    }
};

// Concrete observer for string events (using CRTP)
class StringObserver : public Observer<string, StringObserver>
{
public:
    void onNotify(const string &event) const
    {
        cout << "StringObserver received event: " << event << endl;
    }
};

// Example usage
int main()
{
    // Create integer observers
    IntegerObserver intObserver1;
    IntegerObserver intObserver2;

    // Use variadic templates to create Observable without initializer_list
    auto intObservable = make_observable<int, IntegerObserver>(intObserver1, intObserver2);

    // Notify the integer observers with perfect forwarding (rvalue)
    intObservable.notify(42); // Output: IntegerObserver received event: 42 (twice)

    // Create string observers
    StringObserver stringObserver1;
    StringObserver stringObserver2;

    // Use variadic templates to create Observable without initializer_list
    auto stringObservable = make_observable<string, StringObserver>(stringObserver1, stringObserver2);

    // Notify the string observers with perfect forwarding (lvalue)
    string myMessage = "Hello Observer Pattern!";
    stringObservable.notify(myMessage); // Output: StringObserver received event: Hello Observer Pattern! (twice)

    // Notify the string observers with perfect forwarding (rvalue)
    stringObservable.notify("Goodbye!"); // Output: StringObserver received event: Goodbye! (twice)

    return 0;
}
