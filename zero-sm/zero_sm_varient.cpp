#include <functional>
#include <iostream>
#include <variant>

// Define actions
constexpr auto do_nothing = []()
{ std::cout << "Doing nothing...\n"; };
constexpr auto print_something = []()
{ std::cout << "Printing something...\n"; };

// Define states and events
struct State1
{
};
struct State2
{
};
struct Event1
{
};
struct Event2
{
};

// Transition template
template <typename From, typename Event, auto Guard, auto Action, typename To>
struct Transition
{
    using from = From;
    using event = Event;
    using to = To;
    static constexpr auto guard = Guard;
    static constexpr auto action = Action;
};

// Define a variant to hold any of the states
using StateVariant = std::variant<State1, State2>;

// StateMachine definition with transition pack
template <typename... Transitions>
class StateMachine
{
    StateVariant currentState;

public:
    StateMachine() : currentState(State1{}) {} // Initial state

    template <typename Event>
    constexpr void handleEvent(const Event &event)
    {
        currentState = std::visit([&event](auto &state) -> StateVariant
                                  { return handleEventImpl(state, event, Transitions{}...); }, currentState);
    }

private:
    // Helper to statically find and apply the correct transition
    template <typename State, typename Event, typename FirstTransition, typename... RestTransitions>
    static constexpr StateVariant handleEventImpl(const State &, const Event &event, FirstTransition, RestTransitions... rest)
    {
        if constexpr (std::is_same_v<typename FirstTransition::from, State> &&
                      std::is_same_v<typename FirstTransition::event, Event> &&
                      FirstTransition::guard())
        {
            FirstTransition::action();
            return typename FirstTransition::to{};
        }
        else if constexpr (sizeof...(RestTransitions) > 0)
        {
            return handleEventImpl(State{}, event, rest...);
        }
        else
        {
            // std::cout << "No valid transition found for this event.\n";
            return State{};
        }
    }
};

using MyStateMachine = StateMachine<
    Transition<State1, Event1, []() constexpr
               { return true; }, print_something, State2>,
    Transition<State2, Event2, []() constexpr
               { return true; }, do_nothing, State1>>;

int main()
{
    MyStateMachine sm;
    sm.handleEvent(Event1()); // Should transition to State2
    sm.handleEvent(Event2()); // Should transition back to State1
    sm.handleEvent(Event1()); // Should attempt to transition to State2 again
}
