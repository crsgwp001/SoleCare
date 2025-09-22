#pragma once

// system includes
#include <functional>
#include <map>
#include <vector>

// project includes
#include "fsm_debug.h"

// Alias for any callback with no args
using ActionFn = std::function<void()>;

// Transition descriptor
template <typename StateT, typename EventT> struct Transition {
  StateT from;
  EventT event;
  StateT to;
  ActionFn action;
};

// Core state-machine template
template <typename StateT, typename EventT> class StateMachine {
public:
  StateMachine(StateT init) : _current(init) {}

  // Register a transition
  void addTransition(const Transition<StateT, EventT> &t) {
    _transitions.push_back(t);
  }

  // Register onEntry/onExit for a state
  void setEntry(StateT s, ActionFn fn) {
    _entryMap[s] = fn;
  }
  void setExit(StateT s, ActionFn fn) {
    _exitMap[s] = fn;
  }

  // Handle an incoming event. Returns true if a transition fired.
  // Uses map::find() to avoid mutating the maps during dispatch and
  // a tiny last-match cache to speed up repeated identical dispatches.
  bool handleEvent(EventT ev) {
    // Fast-path: if the last dispatched (state,event) is likely to
    // occur again, use the cached transition index to avoid scanning.
    if (_hasCache) {
      if (_cacheState == _current && _cacheEvent == ev) {
        if (_cacheIndex < _transitions.size()) {
          const auto &t = _transitions[_cacheIndex];
          if (t.from == _current && t.event == ev) {
            auto exitIt = _exitMap.find(_current);
            if (exitIt != _exitMap.end() && exitIt->second)
              exitIt->second();
            if (t.action)
              t.action();
            _current = t.to;
            auto entryIt = _entryMap.find(_current);
            if (entryIt != _entryMap.end() && entryIt->second)
              entryIt->second();
            FSM_DBG_PRINTLN("StateMachine: event consumed (cache)");
            return true;
          }
        }
        // fall through to full scan on cache miss/inconsistency
      }
    }

    for (size_t i = 0; i < _transitions.size(); ++i) {
      const auto &t = _transitions[i];
      if (t.from != _current || t.event != ev)
        continue;

      // call exit if present (use find() to avoid mutating the map)
      auto exitIt = _exitMap.find(_current);
      if (exitIt != _exitMap.end() && exitIt->second)
        exitIt->second();

      if (t.action)
        t.action();

      _current = t.to;

      auto entryIt = _entryMap.find(_current);
      if (entryIt != _entryMap.end() && entryIt->second)
        entryIt->second();

      FSM_DBG_PRINTLN("StateMachine: event consumed");

      // update small cache for likely repeated transitions
      _hasCache = true;
      _cacheState = t.from;
      _cacheEvent = t.event;
      _cacheIndex = i;

      return true;
    }
    return false;
  }

  // Run the active logic for the current state (use find() to avoid accidental map insertion)
  void run() {
    auto it = _runMap.find(_current);
    if (it != _runMap.end() && it->second)
      it->second();
  }

  StateT getState() const {
    return _current;
  }

  void setRun(StateT s, ActionFn fn) {
    _runMap[s] = fn;
  }

private:
  StateT _current;
  std::vector<Transition<StateT, EventT>> _transitions;
  std::map<StateT, ActionFn> _entryMap;
  std::map<StateT, ActionFn> _exitMap;
  std::map<StateT, ActionFn> _runMap;
  // Small cache to speed up repeated identical (state,event) dispatches
  bool _hasCache = false;
  StateT _cacheState{};
  EventT _cacheEvent{};
  size_t _cacheIndex = 0;
};