#include <algorithm>
#include <array>
#include <optional>
#include <string>
#include <tuple>

// Checklist class that can handle any event types
template <typename... Events>
class Checklist
{
public:
    // Construct from variadic predicate functions (deduce types automatically)
    constexpr Checklist(bool (*...predicates)(const Events &))
        : events_{EventData<Events>{predicates}...}, states_{}
    {
        initialize_states();
    }

    // Method to handle a specific event type (now mutating the internal state)
    template <typename Event>
    bool handle_event(const Event &event)
    {
        constexpr std::size_t index = find_event_index<Event>();
        static_assert(index < sizeof...(Events), "Attempting to handle an unregistered event type.");

        if (!states_[index].has_value())
        {
            states_[index] = std::get<index>(events_).predicate(event);
        }

        // Return true only if all events have been checked and hold true
        return std::all_of(states_.begin(), states_.end(), [](const std::optional<bool> &state)
                           { return state.has_value() && state.value(); });
    }

private:
    // Helper to store the predicate function for each event
    template <typename Event>
    struct EventData
    {
        bool (*predicate)(const Event &);
    };

    std::tuple<EventData<Events>...> events_;
    std::array<std::optional<bool>, sizeof...(Events)> states_;

    constexpr void initialize_states()
    {
        states_.fill(std::nullopt);
    }

    // Compile-time search for the index of a particular event type
    template <typename Event, std::size_t... Is>
    constexpr std::size_t find_event_index_impl(std::index_sequence<Is...>) const
    {
        return ((std::is_same_v<Event, Events> ? Is : 0) + ...);
    }

    template <typename Event>
    constexpr std::size_t find_event_index() const
    {
        return find_event_index_impl<Event>(std::index_sequence_for<Events...>{});
    }
};

// Example events
struct Event1
{
    int value;
};

struct Event2
{
    std::string name;
};

// Example predicates
constexpr bool is_event1_valid(const Event1 &e)
{
    return e.value > 10; // Arbitrary condition
}

constexpr bool is_event2_valid(const Event2 &e)
{
    return e.name == "Hello"; // Another arbitrary condition
}

int main()
{
    // Instantiate the checklist with predicates, now no wrapping required
    Checklist checklist{
        is_event1_valid,
        is_event2_valid};

    // Handle the events with their correct types
    Event1 e1{12};      // Should not pass
    Event2 e2{"Hello"}; // Should pass

    bool all_events_valid = !checklist.handle_event(e1) && checklist.handle_event(e2);

    return all_events_valid ? 0 : 1; // Return 0 if all events are valid, 1 otherwise
}
