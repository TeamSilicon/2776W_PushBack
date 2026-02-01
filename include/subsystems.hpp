#pragma once
#include "const.hpp"
#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor outtake(-10);
inline pros::Motor intake(8);
inline ez::Piston matchloader(MATCHLOADER_PISTON);
inline ez::Piston middlegoal(MIDDLEGOAL_PISTON);
inline ez::Piston wing(WING_PISTON);