#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Visual course-hold (forward optical flow). The library is only linked into
// ArduPlane (see ArduPlane/wscript); it compiles unconditionally but is inert
// unless the vehicle creates an AP_VisualCourse instance.
#ifndef AP_VISUALCOURSE_ENABLED
#define AP_VISUALCOURSE_ENABLED 1
#endif
