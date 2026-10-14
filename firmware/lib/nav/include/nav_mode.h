#pragma once

// Select one firmware navigation implementation before including this header.
#define NAV_MICROMOUSE 1
#define NAV_OBSTACLE_AVOIDANCE 2

#ifndef NAV_MODE
#define NAV_MODE NAV_OBSTACLE_AVOIDANCE
#endif

#if NAV_MODE != NAV_MICROMOUSE && NAV_MODE != NAV_OBSTACLE_AVOIDANCE
#error "NAV_MODE must be NAV_MICROMOUSE or NAV_OBSTACLE_AVOIDANCE"
#endif
