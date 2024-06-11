#pragma once

/*
1. A compile time generic behavior tree library
2. A header only library
3. Based on SML style syntax
    - State is a type that can hold a sub state machine, a set of transitions, and a set of actions.
    - Event is a type that can be processed.
    - Action is a type that can be executed on an event. Default is no-op.
    - Guard is a type that returns a boolean value to determine if a transition should be taken. Default is true.
    - Transition is a type that holds a source state, a target state, a guard, and a set of actions. When an event is processed, the transition is taken if the guard returns true.
    - Transition table is a type that holds a set of transitions and can process events resulting in a transition to a target state.
    - State machine is a type that holds has a state and can process events via transitions table.
5. Event driven.
4. C++ 23 latest features
*/

namespace sml_compile_time_state_machine
{

    




}