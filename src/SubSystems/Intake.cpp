#include "Intake.hpp"

Intake::Intake(pros::Motor TOP, pros::Motor BOT, pros::adi::DigitalOut HOOD, pros::adi::DigitalOut MIDMECH, pros::adi::DigitalOut INTAKERAISE, pros::Distance DISTANCE, pros::Distance DISTANCE2) : top(TOP), bot(BOT), hood(HOOD), midmech(MIDMECH), intakeraise(INTAKERAISE), distance(DISTANCE), distance2(DISTANCE2){}

void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool scoreBot, bool outtake){
    if(outtake){
        intakeraise.set_value(false);
        top.move_velocity(-600);
        bot.move_velocity(-600);
    }
    else if(scoreBot){
        intakeraise.set_value(true);
        top.move_velocity(-600);
        bot.move_velocity(-600);
    }
    else if(intake){
        intakeraise.set_value(false);
        bot.move_velocity(600);
        hood.set_value(false);
        midmech.set_value(false);

        if(distance2.get_distance() <= 30 && distance.get_distance() <= 30){
            top.move_velocity(0);
        }
        else{
            top.move_velocity(600);
        }
    }
    else if(scoreTop){
        intakeraise.set_value(false);
        hood.set_value(true);
        midmech.set_value(false);
        top.move_velocity(600);
        bot.move_velocity(600);
    }
    else if(scoreMid){
        intakeraise.set_value(false);
        top.move_velocity(600);
        bot.move_velocity(600);
        hood.set_value(false);
        midmech.set_value(true);
    }
    else{
        intakeraise.set_value(false);
        top.move_velocity(0);
        bot.move_velocity(0);
    }
}