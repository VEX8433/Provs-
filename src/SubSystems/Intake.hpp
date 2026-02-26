#pragma once

#include "api.h"
#include "pros/optical.hpp"

class Intake{
  public:
    Intake(pros::Motor TOP, pros::Motor BOT, pros::adi::DigitalOut HOOD, pros::adi::DigitalOut MIDMECH, pros::adi::DigitalOut INTAKERAISE, pros::Distance DISTANCE, pros::Distance DISTANCE2);

    void telOP(bool intake, bool scoreTop, bool scoreMid, bool scoreBot, bool outtake);

  private:
    pros::Motor top;
    pros::Motor bot;
    pros::Distance distance;
    pros::Distance distance2;
    pros::adi::DigitalOut hood;
    pros::adi::DigitalOut midmech;
    pros::adi::DigitalOut intakeraise;
};