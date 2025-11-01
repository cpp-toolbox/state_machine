#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <functional>
#include <unordered_map>
#include <vector>
#include <string>
#include <optional>

/**
 * @brief A minimal and flexible finite state machine (FSM) implementation.
 *
 * This class provides a generic state machine framework that can operate in two modes:
 * - **MULTI_EVENT:** Emits multiple events per update (enter, update, exit, and transition).
 * - **SINGLE_EVENT:** Emits only one event per update, suitable for systems that process one event at a time.
 *
 * It does not perform side effects directly—only emits `Event` objects describing what happened.
 * Users can then handle these events externally in their own systems.
 *
 * @tparam StateType The enum or integral type representing all possible states.
 */
template <typename StateType> class PureStateMachine {
  public:
    using Condition = std::function<bool()>;

    /**
     * @brief Represents a possible transition from one state to another.
     */
    struct Transition {
        StateType to;        ///< The target state to transition to.
        Condition condition; ///< A callable that returns true when the transition should occur.
    };

    /**
     * @brief The type of event emitted by the state machine.
     */
    enum class EventType {
        ENTER_STATE,  ///< Indicates entry into a new state.
        UPDATE_STATE, ///< Indicates the current state is being updated.
        EXIT_STATE,   ///< Indicates exit from a previous state.
        TRANSITION    ///< Indicates a transition between two states.
    };

    /**
     * @brief Represents a state-related event emitted during an update.
     */
    struct Event {
        EventType type;       ///< The type of event.
        StateType state;      ///< The current state.
        StateType next_state; ///< The next state (may be the same as the current for non-transition events).
    };

    /**
     * @brief Determines how many events are emitted per update tick.
     */
    enum class Mode {
        MULTI_EVENT, ///< Default mode: may emit multiple events per tick.
        SINGLE_EVENT ///< Alternate mode: only one event per tick.
    };

    /**
     * @brief Constructs a new PureStateMachine with an initial state and mode.
     *
     * @param initial_state The starting state of the machine.
     * @param mode The event emission mode (default: MULTI_EVENT).
     */
    explicit PureStateMachine(StateType initial_state, Mode mode = Mode::MULTI_EVENT)
        : current_state(initial_state), mode(mode) {}

    /**
     * @brief Adds a new state to the machine.
     *
     * @param state The state to add.
     */
    void add_state(StateType state) { states[state] = {}; }

    /**
     * @brief Adds a conditional transition between two states.
     *
     * @param from The state from which the transition originates.
     * @param to The state to transition to.
     * @param condition A callable returning true if the transition should occur.
     */
    void add_transition(StateType from, StateType to, Condition condition) { states[from].push_back({to, condition}); }

    /**
     * @brief Updates the state machine and emits events describing what happened.
     *
     * The emitted events depend on the current mode:
     * - In `MULTI_EVENT` mode, multiple events may be emitted per tick (enter, update, transition, etc.).
     * - In `SINGLE_EVENT` mode, only one event is emitted per tick.
     *
     * @return A vector of events representing what occurred during the update.
     */
    [[nodiscard]] std::vector<Event> update_pure() {
        switch (mode) {
        case Mode::MULTI_EVENT:
            return update_multi_event();
        case Mode::SINGLE_EVENT:
            return update_single_event();
        default:
            return {};
        }
    }

    /**
     * @brief Gets the current state.
     * @return The current state.
     */
    [[nodiscard]] StateType get_state() const { return current_state; }

    /**
     * @brief Sets the current operational mode of the FSM.
     * @param new_mode The new mode to use.
     */
    void set_mode(Mode new_mode) { mode = new_mode; }

    /**
     * @brief Gets the current operational mode.
     * @return The current mode.
     */
    [[nodiscard]] Mode get_mode() const { return mode; }

  private:
    /**
     * @brief Handles multi-event update mode.
     *
     * This mode can emit multiple events per update, such as:
     * - `EXIT_STATE` and `ENTER_STATE` when the state changes.
     * - `UPDATE_STATE` for ongoing updates.
     * - `TRANSITION` if a condition triggers a state change.
     *
     * @return A vector of events emitted during this update.
     */
    [[nodiscard]] std::vector<Event> update_multi_event() {
        std::vector<Event> events;

        bool state_changed = current_state != previous_state;
        if (state_changed) {
            if (there_is_a_previous_state)
                events.push_back({EventType::EXIT_STATE, previous_state, current_state});

            events.push_back({EventType::ENTER_STATE, current_state, current_state});
            previous_state = current_state;
            there_is_a_previous_state = true;
        }

        events.push_back({EventType::UPDATE_STATE, current_state, current_state});

        for (auto &t : states[current_state]) {
            if (t.condition && t.condition()) {
                events.push_back({EventType::TRANSITION, current_state, t.to});
                current_state = t.to;
                break;
            }
        }

        return events;
    }

    /**
     * @brief Handles single-event update mode.
     *
     * This mode emits only one event per tick, prioritizing transitions.
     * The order of evaluation is:
     * 1. State change detection (enter/exit)
     * 2. Conditional transitions
     * 3. Update event (default)
     *
     * @return A single-element vector containing the emitted event.
     */
    [[nodiscard]] std::vector<Event> update_single_event() {
        bool state_changed = current_state != previous_state;

        if (state_changed) {
            if (there_is_a_previous_state) {
                previous_state = current_state;
                there_is_a_previous_state = true;
                return {{EventType::EXIT_STATE, previous_state, current_state}};
            } else {
                previous_state = current_state;
                there_is_a_previous_state = true;
                return {{EventType::ENTER_STATE, current_state, current_state}};
            }
        }

        // Evaluate transitions first
        for (auto &t : states[current_state]) {
            if (t.condition && t.condition()) {
                current_state = t.to;
                return {{EventType::TRANSITION, previous_state, current_state}};
            }
        }

        // Default: update event
        return {{EventType::UPDATE_STATE, current_state, current_state}};
    }

  private:
    StateType current_state;                ///< The current active state.
    StateType previous_state;               ///< The previous state (if any).
    bool there_is_a_previous_state = false; ///< Whether a previous state has been recorded.
    Mode mode;                              ///< The current operational mode (multi-event or single-event).

    std::unordered_map<StateType, std::vector<Transition>> states; ///< All defined states and their transitions.
};

/**
 * @brief A state machine wrapper that binds actions (callbacks/lambdas) to state lifecycle events.
 *
 * This class wraps a PureStateMachine and executes side effects (callbacks) corresponding
 * to the emitted events from the pure FSM. It’s suitable for real-time or game logic,
 * where on_enter, on_update, and on_exit actions should trigger immediate effects.
 *
 * @tparam StateType Enum or integral type representing states.
 */
template <typename StateType> class StateMachine {
  public:
    using Action = std::function<void()>;
    using Condition = std::function<bool()>;
    using Pure = PureStateMachine<StateType>;

    struct State {
        std::optional<Action> on_enter;
        std::optional<Action> on_update;
        std::optional<Action> on_exit;
    };

    explicit StateMachine(StateType initial_state) : pure_fsm(initial_state) {}

    void add_state(StateType state, std::optional<Action> on_enter = std::nullopt,
                   std::optional<Action> on_update = std::nullopt, std::optional<Action> on_exit = std::nullopt) {
        pure_fsm.add_state(state);
        states[state] = {on_enter, on_update, on_exit};
    }

    void add_transition(StateType from, StateType to, Condition condition) {
        pure_fsm.add_transition(from, to, condition);
    }

    /**
     * @brief Performs one update tick:
     *  - Runs pure logic (no side effects)
     *  - Executes callbacks corresponding to emitted events
     */
    void update() {
        auto events = pure_fsm.update_pure();

        for (auto &event : events) {

            // NOTE: the order in this switch defines that exit is run first, then enter, and then update
            // which is how we would want it to be in terms of linear time
            switch (event.type) {

            case Pure::EventType::EXIT_STATE:
                if (states[event.state].on_exit)
                    (*states[event.state].on_exit)();
                break;

            case Pure::EventType::ENTER_STATE:
                if (states[event.state].on_enter)
                    (*states[event.state].on_enter)();
                break;

            case Pure::EventType::UPDATE_STATE:
                if (states[event.state].on_update)
                    (*states[event.state].on_update)();
                break;

            default:
                break;
            }
        }
    }

    [[nodiscard]] StateType get_state() const { return pure_fsm.get_state(); }

    [[nodiscard]] const Pure &get_pure_fsm() const { return pure_fsm; }

  private:
    Pure pure_fsm;
    std::unordered_map<StateType, State> states;
};

#endif // STATE_MACHINE_HPP
