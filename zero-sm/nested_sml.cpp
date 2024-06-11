#include <boost/sml.hpp>
#include <iostream>

namespace sml = boost::sml;

// Define chess states and events
struct StartChess
{
};
struct Win
{
};
struct Lose
{
};

// Chess state machine
struct Chess
{
    auto operator()() const noexcept
    {
        using namespace sml;
        // States
        return make_transition_table(
            *"idle"_s + event<StartChess> = "playing"_s,
            "playing"_s + event<Win> = "winning"_s,
            "playing"_s + event<Lose> = "losing"_s,

            "winning"_s + on_entry<_> / []
                              { std::cout << "Won the chess game!\n"; },
            "losing"_s + on_entry<_> / []
                             { std::cout << "Lost the chess game...\n"; });
    }
};

// Define coffee machine states and events
struct Start
{
};
struct DoneMakingCoffee
{
};
struct GetBored
{
};

// Coffee machine state machine
struct CoffeeMachine
{
    auto operator()() const noexcept
    {
        using namespace sml;
        // States
        return make_transition_table(
            *"idle"_s + event<Start> = "making_coffee"_s,
            "making_coffee"_s + event<DoneMakingCoffee> = "coffee_ready"_s,
            "coffee_ready"_s + event<GetBored> / [] // Transition to nested state machine
                                   { std::cout << "Getting bored, let's play chess!\n"; } = state<Chess> // Nested state machine
        );
    }
};

int main()
{
    sml::sm<CoffeeMachine> sm;
    sm.process_event(Start{});            // Start making coffee
    sm.process_event(DoneMakingCoffee{}); // Coffee is ready
    sm.process_event(GetBored{});         // Gets bored, starts playing chess
    sm.process_event(StartChess{});       // Playing chess now
    sm.process_event(Win{});              // Wins the game

    return 0;
}
