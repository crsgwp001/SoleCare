#pragma once

#include <Events.h>

void createStateMachineTask(void);
// Post an Event into the FSM queue from other tasks
bool fsmExternalPost(Event ev);