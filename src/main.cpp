#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstddef>


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// left motors: (reversed) back - 02, middle - 04, (reversed) front - 06
pros::MotorGroup leftMotors({-2, 4, -6}, pros::MotorGearset::blue);
// right motors: right motors: back - 05, (reversed) middle - 07, front - 03
pros::MotorGroup rightMotors({3, 5, -7}, pros::MotorGearset::blue);

// motor
pros::Motor FirstCollector(16); // front bottom collector (2 drums)
pros::Motor SecondCollector(12); // back collector (2 drums)
pros::Motor ThirdCollector(11); // front top collector

//pros::MotorGroup intake({11, 10}, pros::MotorGearset::blue);

// optical sensor
pros::Optical optical(19);

// Inertial Sensor on port 10
pros::Imu imu(10);

pros::adi::Pneumatics hood('A', false);

pros::adi::Pneumatics feeder('H', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);



// lateral motion controller
lemlib::ControllerSettings linearController(13, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            51, // derivative gain (kD) **NEW VALUE, UNTESTED
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             50, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20); // left-right odom wheel
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-9); // front-back odom wheel
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    optical.disable_gesture(); // disable gesture

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            lemlib::Pose chass = chassis.getPose();
            pros::lcd::print(0, "X: %f", chass.x);
            pros::lcd::print(1, "Y: %f", chass.y);
            pros::lcd::print(2, "Theta: %f", chass.theta);

            // Display HSV
            pros::lcd::print(3, "Hue:  %f", optical.get_hue());
            pros::lcd::print(4, "Saturation:  %f", optical.get_saturation());
            pros::lcd::print(5, "Brightness:  %f", optical.get_brightness());
            pros::lcd::print(6, "Proximity: %d", optical.get_proximity());

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chass);

            pros::delay(50);
        }
    });
}

int colourDet() {
    double hue = optical.get_hue();
    int prox = optical.get_proximity();
    if ( hue > 315 || hue < 30 ) {
        return 1; // 1 = red
    } else if ( hue > 80 && hue < 270 ) {
        return 2; // 2 = blue
    } else {
        return 0; // 0 = none
    };
}

void intake(){
    FirstCollector.move(127);
    SecondCollector.move(-127);
}

void topOuttake(){
    SecondCollector.move(127);
    FirstCollector.move(127);
    ThirdCollector.move(127);
}

void midOuttake(){
    SecondCollector.move(127);
    FirstCollector.move(127);
    ThirdCollector.move(-127);
}

void bottomOuttake(){
    SecondCollector.move(127);
    FirstCollector.move(-127);
}

void stopAllCollectors() {
    FirstCollector.move(0);
    SecondCollector.move(0);
    ThirdCollector.move(0);
}


/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

// true = RED
// false = BLUE
int alliance = true;
void intakeWithDet() { // DO NOT TOUCH THIS FUNCTION. IF IT WORKS, DON'T TOUCH IT
    FirstCollector.move(127);

    while (colourDet() == 0) {} // hangs the damn clanker until it detects a colour

    // if detect colour matching alliance, intake
    if ( colourDet() == 2 - alliance ){
        bottomOuttake();
    } else if ( colourDet() == 1 + alliance ) {
        intake();
    }
}

// TODO: make it possible to terminate this loop with an and in the for statement
// TODO: make a version of this in a sepeate task for async (if possible) in a seperate function for the opcontrol loop
void intakeWithDetMultiple(int blocks) { // blocks is how many blocks to intake, 0 makes it infinite
    intake();

    while (colourDet() == 0) {}
    
    for (int i = 1; (i <= blocks or i > 1);) { // 
        // check colour (and choose to spit out or intake, if intake is chosen, then increment i)
        if ( colourDet() == 2 - alliance ){
            FirstCollector.move(-127);
            while (colourDet() == 2) {}
        } else if ( colourDet() == 1 + alliance ) {
            i++;
        }
        intake();
        while (colourDet() == 0) {}
    }
}

/**
 * Runs during auto
 */
void autonomous() {
    // note: offset is ~8 in, robot length (including feeder down) is 20 in
    // robot width is 15 in

    // auton strat
    // if the starting side is left, simply reverse the + or - sign of the X values
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24.25, 1);
    chassis.moveToPoint(19.25, 24.25, 1);
    intakeWithDetMultiple(3);
    chassis.moveToPoint(19.25, 48.5, 1);
    chassis.moveToPoint(43.5, 48.5, 1);
    intakeWithDetMultiple(2);
    chassis.moveToPoint(19.25, 48.5, 1);
    // knocks over the thing
    chassis.moveToPoint(61.6875, 48.5, 1); // origin was (67.75, 48.5), this is to make sure it doesn't ram the wall
    chassis.moveToPoint(43.5, -12.125, 1);
    intakeWithDetMultiple(2);
    chassis.moveToPoint(43.5, 12.125, 1);
    topOuttake();
    pros::delay(4269);
    stopAllCollectors();
    



    // // set position to x:0, y:0, heading:0
    // clamp.retract();
    // chassis.setPose(12, 46.54, 90);
    // // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    // chassis.moveToPoint(46.64, 46.54, 1000,  {.forwards=false, .maxSpeed = 127,});
    // chassis.waitUntilDone();
    // clamp.extend();
    // chassis.moveToPose(46.64, 23.08, 180, 1000);
    // chassis.waitUntil(21);
    // intake.move(127);
    // chassis.waitUntil(2.36);
    // intake.move(0);
    // chassis.moveToPose(12, 12, 225, 1000);
    // chassis.waitUntilDone();
    // intake.move(127);

    // chassis.moveToPoint(10, 0, 1000);

    // // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // // cancel the movement after it has traveled 10 inches
    // chassis.waitUntil(10);
    // chassis.cancelMotion();
    // // Turn to face the point x:45, y:-45. Timeout set to 1000
    // // dont turn faster than 60 (out of a maximum of 127)
    // chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // // Turn to face a d              irection of 90º. Timeout set to 1000
    // // will always be faster than 100 (out of a maximum of 127)
    // // also force it to turn clockwise, the long way around
    // chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // // following the path with the back of the robot (forwards = false)
    // // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // // the movement will run immediately
    // // Unless its another movement, in which case it will wait
    // chassis.waitUntil(10);
    // pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    
    // pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */

bool intakeDirection = true; 

bool climbExtended = false;

bool clampActivated = false;

bool pressed = false;

bool manual = false;

void opcontrol() {
    pros::Task autoIntakeTask([&]() {intakeWithDetMultiple();});
    // controller
    // loop to continuously update motors
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            manual = !manual;
            if (manual) {
                autoIntakeTask.remove();
            } else {
              pros::Task autoIntakeTask([&]() {intakeWithDetMultiple();});
            }
        }

        if (manual) {
            /////////////////////// intake ///////////////////////
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                // intakeDirection = true;
                intake();

            /////////////////////// outtake ///////////////////////
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                midOuttake();
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                bottomOuttake();
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                // intakeDirection = false;
                topOuttake();
            } else {
                stopAllCollectors();
            }
        }


        // TODO:
        // implement feeder and hood controls

        pros::delay(25);
    }
        

        // // if (intakeDirection){
        // //     intake.move(127);  // Move forward
        // // }

        // // else if (!intakeDirection){
        // //     intake.move(-127);
        // // }
        // ///////////////////////////////////////////
        
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        //     clampActivated = !clampActivated;  // Toggle state
        
        //     if (clampActivated) {
        //         clamp.extend();
        //     } else {
        //         clamp.retract();
        //     }
        // }




        
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        //     climbExtended = !climbExtended;  // Toggle state,mst
        
        //     if (climbExtended) {
        //         climb.extend();
        //     } else {
        //         climb.retract();
        //     }
        // }


        pros::delay(10);
    
}
