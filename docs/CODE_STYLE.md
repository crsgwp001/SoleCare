Coding style and file layout

This project uses a simple, consistent layout to improve readability across C/C++ files.

Canonical file block order (headers, .h):

1. Header guard / #pragma once
2. Includes (standard/system first, then external libs, then project-local includes)
3. Public macros / #defines
4. Public constants (constexpr)
5. Typedefs / enums / structs (public API)
6. Forward declarations
7. Public function prototypes / class declarations

Canonical file block order (implementation, .cpp):

1. Includes (same ordering rules as headers)
2. File-local macros / static consts
3. Static forward declarations (helper functions)
4. Global/static variable definitions (minimize globals)
5. Public API function implementations
6. Static helper implementations
7. Task entry points / setup / main

Include rules:
- Use angled brackets for system and external libs: #include <vector>
- Use double quotes for project headers: #include "ui.h"
- Keep related includes grouped and leave a blank line between groups.

Formatting and style:
- Use the repository's `.clang-format` (root) for automated formatting.
- Prefer 2-space indentation for embedded readability (configured in .clang-format).
- Keep functions short and focused; extract helpers when code is repeated or blocks grow too large.
- Minimize cross-file globals. Use accessors where possible.

Commit and PR guidance:
- Tidy changes should be applied on a branch (e.g. `chore/tidy-code`) and have a single commit describing the formatting changes.
- Do not introduce functional changes in the tidy commit. If a fix is required, separate it into its own PR.

Examples

Header (my_sensor.h):

```c
#pragma once

#include <Arduino.h>
#include "Sensor.h"

#define SENSOR_TIMEOUT_MS 5000

extern void sensorInit();
extern bool sensorRead(float *out);
```

Implementation (my_sensor.cpp):

```c
#include "my_sensor.h"

static void helper_delay_ms(unsigned long ms);

static int s_state = 0;

void sensorInit() {
  // ...
}

bool sensorRead(float *out) {
  // ...
}

static void helper_delay_ms(unsigned long ms) {
  delay(ms);
}
```
