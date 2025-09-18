#pragma once
#include <functional>
#include <vector>
#include <map>
#include "fsm_debug.h"

// Alias for any callback with no args
using ActionFn = std::function<void()>;

// Transition descriptor
template<typename StateT, typename EventT>
struct Transition {
    StateT    from;
    EventT    event;
    StateT    to;
    ActionFn  action;
};

// Core state-machine template
template<typename StateT, typename EventT>
class StateMachine {
public:
    StateMachine(StateT init) 
      : _current(init) {}

    // Register a transition
    void addTransition(const Transition<StateT,EventT>& t) {
        _transitions.push_back(t);
    }

    // Register onEntry/onExit for a state
    void setEntry(StateT s, ActionFn fn) { _entryMap[s] = fn; }
    void setExit (StateT s, ActionFn fn) { _exitMap[s]  = fn; }

    // Handle an incoming event. Returns true if a transition fired.
    bool handleEvent(EventT ev) {
        for (auto& t : _transitions) {
            if (t.from == _current && t.event == ev) {
                auto exitFn  = _exitMap[_current];
                auto entryFn = _entryMap[t.to];

                if (exitFn)  exitFn();
                if (t.action) t.action();

                _current = t.to;
                if (entryFn) entryFn();
                                FSM_DBG_PRINTLN("StateMachine: event consumed");
                return true;
            }
        }
        return false;
    }

    // Run the “active” logic for the current state
    void run() {
        if (_runMap.count(_current))  
            _runMap[_current]();
    }

    StateT getState() const { return _current; }

    void setRun(StateT s, ActionFn fn) { _runMap[s] = fn; }

private:
    StateT _current;
    std::vector<Transition<StateT,EventT>>         _transitions;
    std::map<StateT,ActionFn>                      _entryMap;
    std::map<StateT,ActionFn>                      _exitMap;
    std::map<StateT,ActionFn>                      _runMap;
};