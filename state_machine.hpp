#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <optional>

/**
 * @brief A deterministic finite state machine (FSM) with strict transition validation.
 *
 * Each state can have multiple outgoing transitions, each with its own condition and optional callback.
 * On update(), exactly one condition must evaluate to true; otherwise, an exception is thrown.
 * It is the user's responsibility to provide mutually exclusive conditions.
 *
 * @tparam StateType Enum-like type representing states.
 */
template <typename StateType> class StateMachine {
  public:
    using Condition = std::function<bool()>;
    using Callback = std::function<void()>;
    using StateToString = std::function<std::string(StateType)>;

    /**
     * @brief Represents a transition from one state to another.
     */
    struct Transition {
        StateType to;                     ///< The target state.
        Condition condition;              ///< Condition under which the transition occurs.
        std::optional<Callback> callback; ///< Optional callback invoked when the transition occurs.
    };

    /**
     * @brief Construct a FSM with an initial state and optional previous state.
     *
     * @param initial_state The starting state of the FSM.
     * @param previous_state Optional previous state. If not provided, defaults to initial_state. The purpose of this
     * parameter to allow you to run an initial transition to set things up if that's needed in your system.
     * @param state_to_string_fn Optional function to convert a StateType value to a human-readable string.
     *        If provided, exceptions thrown by the FSM (e.g., in update(), set_condition(), or set_callback())
     *        will include readable state names, making error messages much clearer.
     */
    explicit StateMachine(StateType initial_state, std::optional<StateType> previous_state = std::nullopt,
                          std::optional<StateToString> state_to_string_fn = std::nullopt)
        : current_state(initial_state), previous_state(previous_state.value_or(initial_state)),
          state_to_string(std::move(state_to_string_fn)) {}

    /**
     * @brief Add a transition between two states with an optional callback.
     *
     * @param from Source state.
     * @param to Target state.
     * @param condition Condition function for the transition.
     * @param callback Optional callback executed when the transition occurs.
     */
    void add_transition(StateType from, StateType to, Condition condition,
                        std::optional<Callback> callback = std::nullopt) {
        transitions[from].push_back(Transition{to, std::move(condition), std::move(callback)});
    }

    /**
     * @brief Update the condition for a specific transition.
     *
     * @param from Source state.
     * @param to Target state.
     * @param new_condition New condition function.
     *
     * @throws std::runtime_error if the transition does not exist.
     */
    void set_condition(StateType from, StateType to, Condition new_condition) {
        auto &vec = get_transition_vector(from);
        for (auto &transition : vec) {
            if (transition.to == to) {
                transition.condition = std::move(new_condition);
                return;
            }
        }

        if (state_to_string) {
            throw std::runtime_error("No transition from state " + state_to_string.value()(from) + " to target state " +
                                     state_to_string.value()(to) + " found.");
        } else {
            throw std::runtime_error("No transition from this state to target state found.");
        }
    }

    /**
     * @brief Update the callback for a specific transition.
     *
     * @param from Source state.
     * @param to Target state.
     * @param new_callback New callback to replace the existing one (or std::nullopt to remove it).
     *
     * @throws std::runtime_error if the transition does not exist.
     */
    void set_callback(StateType from, StateType to, std::optional<Callback> new_callback) {
        auto &vec = get_transition_vector(from);
        for (auto &transition : vec) {
            if (transition.to == to) {
                transition.callback = std::move(new_callback);
                return;
            }
        }

        if (state_to_string) {
            throw std::runtime_error("No transition from state " + state_to_string.value()(from) + " to target state " +
                                     state_to_string.value()(to) + " found.");
        } else {
            throw std::runtime_error("No transition from this state to target state found.");
        }
    }

    /**
     * @brief Extend an existing callback for a transition.
     *
     * This will preserve the existing callback (if any) and call the new one after it.
     *
     * @param from Source state.
     * @param to Target state.
     * @param additional_callback The callback to append.
     *
     * @throws std::runtime_error if the transition does not exist.
     */
    void extend_callback(StateType from, StateType to, Callback additional_callback) {
        auto &vec = get_transition_vector(from);
        for (auto &transition : vec) {
            if (transition.to == to) {
                if (transition.callback) {
                    Callback original = transition.callback.value();
                    transition.callback = [original, additional_callback]() {
                        original();
                        additional_callback();
                    };
                } else {
                    transition.callback = std::move(additional_callback);
                }
                return;
            }
        }

        if (state_to_string) {
            throw std::runtime_error("No transition from state " + state_to_string.value()(from) + " to target state " +
                                     state_to_string.value()(to) + " found.");
        } else {
            throw std::runtime_error("No transition from this state to target state found.");
        }
    }

    /**
     * @brief Update the FSM — transitions to exactly one next state.
     *
     * Evaluates all conditions for the current state's outgoing transitions.
     * Exactly one must be true; otherwise, an exception is thrown.
     * If no conditions are true and there is no explicit self-transition,
     * the FSM will remain in the current state.
     */
    void update() {
        auto transitions_from_current = transitions.find(current_state);

        // If there are no outgoing transitions at all, just stay in the current state
        if (transitions_from_current == transitions.end()) {
            previous_state = current_state;
            return;
        }

        auto &outgoing_transitions = transitions_from_current->second;
        std::vector<Transition *> transitions_whose_conditions_were_met;
        transitions_whose_conditions_were_met.reserve(outgoing_transitions.size());

        for (auto &transition : outgoing_transitions) {
            if (transition.condition && transition.condition()) {
                transitions_whose_conditions_were_met.push_back(&transition);
            }
        }

        if (transitions_whose_conditions_were_met.empty()) {
            // Check if there is a self-transition explicitly
            bool has_self_transition = false;
            for (auto &t : outgoing_transitions) {
                if (t.to == current_state) {
                    has_self_transition = true;
                    break;
                }
            }

            if (has_self_transition) { // even the self transition was untrue, this is bad
                // There is a self-transition, but its condition is false treat it as no valid transition
                if (state_to_string) {
                    throw std::runtime_error("No transition conditions are true for the current state: " +
                                             state_to_string.value()(current_state));
                } else {
                    throw std::runtime_error("No transition conditions are true for the current state.");
                }
            }

            // there were no valid outgoing transtitions, but there was no self transition, so we just stay in the
            // current state

            previous_state = current_state;
            // Comment: "No valid transitions and no self-transition; FSM remains in current state."
            return;
        }

        if (transitions_whose_conditions_were_met.size() > 1) {
            if (state_to_string) {
                std::string states_str;
                for (auto *t : transitions_whose_conditions_were_met) {
                    states_str += state_to_string.value()(t->to) + " ";
                }
                throw std::runtime_error("Multiple transition conditions are true for current state " +
                                         state_to_string.value()(current_state) + ". Valid targets: " + states_str);
            } else {
                throw std::runtime_error("Multiple transition conditions are true for the current state.");
            }
        }

        // Exactly one valid transition
        Transition &transition_to_execute = *transitions_whose_conditions_were_met.front();
        previous_state = current_state;
        current_state = transition_to_execute.to;

        if (transition_to_execute.callback) {
            transition_to_execute.callback.value()();
        }
    }

    /**
     * @brief Get the previous and current states of the FSM.
     *
     * @return Tuple of (previous_state, current_state)
     */
    [[nodiscard]] std::tuple<StateType, StateType> get_state() const { return {previous_state, current_state}; }

  private:
    /**
     * @brief Helper to get the vector of transitions from a given state.
     *
     * @param from The source state.
     * @return Reference to the vector of transitions.
     * @throws std::runtime_error if no transitions exist from the given state.
     */
    std::vector<Transition> &get_transition_vector(StateType from) {
        auto it = transitions.find(from);
        if (it == transitions.end()) {
            if (state_to_string) {
                throw std::runtime_error("No transitions from state: " + state_to_string.value()(from));
            } else {
                throw std::runtime_error("No transitions from this state.");
            }
        }
        return it->second;
    }

    std::optional<StateToString> state_to_string;

    StateType current_state;  ///< The currently active state.
    StateType previous_state; ///< The previous state before the last transition.

    std::unordered_map<StateType, std::vector<Transition>> transitions; ///< All transitions keyed by source state.
};

/**
 * @note Usage:
 *
 * 1. **Polling style:**
 *    Call `update()` regularly (e.g., in a loop) and then check `get_state()`.
 *    ```cpp
 *    fsm.update();
 *    auto [prev, curr] = fsm.get_state();
 *    if (curr == MyState::Ready) { ... }
 *    ```
 *
 * 2. **Callback style:**
 *    Provide a callback when adding a transition. It will be invoked immediately
 *    when the transition condition evaluates to true.
 *    ```cpp
 *    fsm.add_transition(MyState::Idle, MyState::Active,
 *                       [](){ return should_activate(); },
 *                       [](auto prev, auto curr){ std::cout << "Transitioned!\n"; });
 *    ```
 */

#endif // STATE_MACHINE_HPP
