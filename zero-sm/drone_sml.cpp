#include <boost/sml.hpp>
#include <iostream>
#include <variant>
#include <functional>

namespace sml = boost::sml;

// Define events
struct EventGPSFail {};
struct EventGPSRecover {};

struct CommunicationFail {};
struct CommunicationRecover {};

struct EventTakeoff { float altitudeTarget; };
struct EventMissionStart {};
struct EventMissionEnd {};
struct EventLand {};

// Define states
struct GPSHealthy {};
struct GPSUnhealthy {};

struct CommunicationHealthy {};
struct CommunicationUnhealthy {};

struct OnGround {};
struct TakingOff {};
struct MissionInProgress {};
struct ManualControl {};
struct Landing {};

// Define a variant to hold all possible event types
using EventVariant = std::variant<EventGPSFail, EventGPSRecover, CommunicationFail, CommunicationRecover, EventTakeoff, EventMissionStart, EventMissionEnd, EventLand>;


struct gpsHealthStateMachine_t {
    auto constexpr operator()() const {
        using namespace sml;
        return make_transition_table(
            *state<GPSHealthy> + event<EventGPSFail> = state<GPSUnhealthy>,
            state<GPSUnhealthy> + event<EventGPSRecover> = state<GPSHealthy>
            );
        }
    };
struct TakeoffStateMachine_t
    {
        // bool isGPSHealthy(sml::sm<gpsHealthStateMachine_t>& gpsSM) const noexcept
        // {
        //     return gpsSM.is(sml::state<GPSHealthy>);
        // }

        
        auto constexpr operator()() const noexcept
        {
            using namespace sml;
            return make_transition_table(
                *state<OnGround> + event<EventTakeoff> [!![](const auto& , sml::sm<gpsHealthStateMachine_t>& gpsSM) { return gpsSM.is(sml::state<GPSHealthy>); }] = state<TakingOff>,
                state<OnGround> + event<EventTakeoff> [![](const auto& , sml::sm<gpsHealthStateMachine_t>& gpsSM) { return gpsSM.is(sml::state<GPSHealthy>); }] = X);
        }
    };

int main() {

    sml::sm<gpsHealthStateMachine_t> gpsSM{gpsHealthStateMachine_t{}};
    sml::sm<TakeoffStateMachine_t> takeoffSM{TakeoffStateMachine_t{gpsSM}};


    gpsSM.process_event(EventGPSFail{});
    takeoffSM.process_event(EventTakeoff{1000}, gpsSM);
    gpsSM.process_event(EventGPSRecover{});


    // auto gpsSM = GPSStateMachine();
    // auto takeoffSM = TakeoffStateMachine(gpsSM);

    // // Process events through the unified entry point
    // gpsSM.processEvent(EventGPSFail{});
    // takeoffSM.processEvent(EventTakeoff{1000});
    // gpsSM.processEvent(EventGPSRecover{});

    // auto gpsHealthStateMachine_t = [] {
    //     using namespace sml;
    //     return make_transition_table(
    //         *state<GPSHealthy> + event<EventGPSFail> = state<GPSUnhealthy>,
    //         state<GPSUnhealthy> + event<EventGPSRecover> = state<GPSHealthy>
    //     );
    // };
    // sml::sm<decltype(gpsHealthStateMachine_t)> gpsSM{gpsHealthStateMachine_t};
    
    // auto isGPSHealthy = [&] { return gpsSM.is(sml::state<GPSHealthy>); };
    // auto takeoffAction = [](const auto& event) {std::cout << "Taking off to altitude: " << event.altitudeTarget << "\n";};
    // auto cannotTakeoffAction = [] { std::cout << "Cannot take off: GPS unhealthy.\n"; };

    // auto takeoffStateMachine = [&] {
    //     using namespace sml;
    //     return make_transition_table(
    //         * "idle"_s + event<EventTakeoff>[isGPSHealthy] / takeoffAction = "taking_off"_s,
    //         "idle"_s + event<EventTakeoff>[!isGPSHealthy] / cannotTakeoffAction = X,
    //     );
    // };

    // // Instantiate the state machines

    // sml::sm<decltype(takeoffStateMachine)> takeoffSM{takeoffStateMachine};

    // // Unified event processor
    // auto processEvent = [&](const EventVariant& event) {
    //     std::visit([&](const auto& evt) {
    //         // Forward the event to both state machines
    //         gpsSM.process_event(evt);
    //         takeoffSM.process_event(evt);
    //     }, event);
    // };

    // // Process events through the unified entry point
    // processEvent(EventGPSFail{});
    // processEvent(EventTakeoff{1000});
    // processEvent(EventGPSRecover{});

    return 0;
}
