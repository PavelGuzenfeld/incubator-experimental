#include <type_traits>

// Transition template
template <typename From, typename Event, typename Guard, typename Action, typename To>
struct Transition
{
    using from = From;
    using event = Event;
    using to = To;
    using guard = Guard;
    using action = Action;
};

// Dummy transition to indicate the end of recursion
struct DummyTransition
{
};

// Find transaction helpers
template <typename State, typename Event, typename First, typename... Rest>
// Recursion loop
struct findTransition
{
    using type = typename std::conditional_t<
        std::is_same_v<typename First::from, State> &&
            std::is_same_v<typename First::event, Event> && First::guard(),
        First, typename findTransition<State, Event, Rest...>::type>;
};
// Stop condition
template <typename State, typename Event>
struct findTransition<State, Event, DummyTransition>
{
    using type = DummyTransition;
};

template <typename CurrentState, typename... Transitions>
class StateMachine
{
public:
    template <typename Event>
    constexpr auto handleEvent(const Event &event)
    {
        using Transition =
            typename findTransition<CurrentState, Event, Transitions...,
                                    DummyTransition>::type;
        if constexpr (!std::is_same_v<Transition, DummyTransition>)
        {
            Transition::action(event);
            return StateMachine<typename Transition::to, Transitions...>();
        }
        else
        {
            return *this;
        }
    }
};

int main()
{

    int state = 0;

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

    // Guards
    constexpr bool g = true;
    constexpr auto guard_st1_st2 = [&]()
    { return g; };
    constexpr auto guard_st2_st1 = [&]()
    { return g; };

    // Actions
    auto update_state_1 = [&](auto)
    { state = 1; };
    auto print_something = [](auto) {};

    // StateMachine with transitions
    using MyStateMachine = StateMachine<
        State1,
        Transition<State1, Event1, decltype(guard_st1_st2), decltype(print_something), State2>, Transition<State2, Event2, decltype(guard_st2_st1), decltype(update_state_1), State1>>;

    MyStateMachine sm;
    auto sm2 = sm.handleEvent(Event1());  // Should transition to State2
    auto sm3 = sm2.handleEvent(Event2()); // Should transition back to State1
    auto sm4 = sm3.handleEvent(Event1()); // No valid transition
    return state;
}
