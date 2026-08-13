#pragma once
#include "BoardProfile.h"

// Board selection. Each environment in platformio.ini sets -D FOCEARS_BOARD=<n>;
// the matching file below defines a single `static constexpr BoardProfile ACTIVE_BOARD`.
// To add a unit: create boards/boardN_<label>.h and add a case here.

#ifndef FOCEARS_BOARD
#error "FOCEARS_BOARD is not defined. Set -D FOCEARS_BOARD=<n> in the build environment."
#endif

#if FOCEARS_BOARD == 1
#include "boards/board1.h"
#elif FOCEARS_BOARD == 2
#include "boards/board2.h"
#else
#error "Unknown FOCEARS_BOARD value — add a case in board_config.h and a matching boards/ file."
#endif
