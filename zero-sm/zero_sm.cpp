#include <type_traits>

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

// Dummy transition to indicate the end of recursion
struct DummyTransition
{
};

template <typename CurrentState, typename... Transitions>
class StateMachine
{
public:
    template <typename Event>
    auto handleEvent(const Event &event)
    {
        using Transition = typename findTransition<CurrentState, Event, Transitions..., DummyTransition>::type;
        if constexpr (!std::is_same_v<Transition, DummyTransition>)
        {
            Transition::action();
            return StateMachine<typename Transition::to, Transitions...>();
        }
        else
        {
            return *this;
        }
    }

private:
    template <typename State, typename Event, typename First, typename... Rest>
    struct findTransition
    {
        using type = typename std::conditional_t<
            std::is_same_v<typename First::from, State> &&
                std::is_same_v<typename First::event, Event> &&
                First::guard(),
            First,
            typename findTransition<State, Event, Rest...>::type>;
    };

    template <typename State, typename Event>
    struct findTransition<State, Event, DummyTransition>
    {
        using type = DummyTransition;
    };
};

// Actions
constexpr auto do_nothing = []()
{ };
constexpr auto print_something = []()
{ };

// States and Events
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

// StateMachine with transitions
using MyStateMachine = StateMachine<
    State1,
    Transition<State1, Event1, []() constexpr
               { return true; }, print_something, State2>,
    Transition<State2, Event2, []() constexpr
               { return true; }, do_nothing, State1>>;

int main()
{
    MyStateMachine sm;
    auto sm2 = sm.handleEvent(Event1());  // Should transition to State2
    auto sm3 = sm2.handleEvent(Event2()); // Should transition back to State1
    auto sm4 = sm3.handleEvent(Event1()); // No valid transition
}
