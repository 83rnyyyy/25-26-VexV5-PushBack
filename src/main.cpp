#include "main.h" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// ----------- INIT PORTS FOR ROBOT MODULES ----------- //
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, 4, -6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 5, -7}, pros::MotorGearset::blue);

// collectors
pros::Motor FirstCollector(16, pros::MotorGearset::blue);   // front bottom collector
pros::Motor SecondCollector(12, pros::MotorGearset::blue);  // back collector
pros::Motor ThirdCollector(-11, pros::MotorGearset::blue);   // front top collector

// sensors
pros::Imu imu(10);

// pneumatics
pros::adi::Pneumatics wing('E', false);
pros::adi::Pneumatics feeder('G', false);
pros::adi::Pneumatics stopper('H', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, lemlib::Omniwheel::NEW_275, 450, 2);
// controllers
lemlib::ControllerSettings linearController(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              44, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(3, // proportional gain (kP) 3
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD) 10
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// tracking wheels
pros::Rotation horizontalEnc(20);
pros::Rotation verticalEnc(-9);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.75);

// odom sensors
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

// drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


// -------------------- QUICK CONFIGURATION -------------------- //
constexpr int side = 1; // 1 = right, -1 = left


// -------------------- Intake & Outake functions -------------------- //
volatile bool autoIntakeEnabled = false;



void intake() {
    FirstCollector.move(-127);
    SecondCollector.move(-127);
    if (!stopper.is_extended()) stopper.extend();
}

void topOuttake() {
    if (stopper.is_extended()) stopper.retract();
    FirstCollector.move(-127);
    SecondCollector.move(-127);
    ThirdCollector.move(127);
    
}

void midOuttake() {
    stopper.retract();
    FirstCollector.move(-127);
    SecondCollector.move(-127);
    ThirdCollector.move(-127);
}

void bottomOuttake() {
    FirstCollector.move(127);
    SecondCollector.move(127);
    ThirdCollector.move(-127);
    // if (stopper.is_extended()) stopper.retract()
}

void bottomOuttake2(float spd) {
    FirstCollector.move(127);
    SecondCollector.move(spd);
    ThirdCollector.move(-127);
    // if (stopper.is_extended()) stopper.retract()
}

void stopAllCollectors() {
    FirstCollector.move(0);
    SecondCollector.move(0);
    ThirdCollector.move(0);
    // if (!stopper.is_extended()) {
    //     stopper.extend();
    // }
}

/* blocks=0 --> infinite */
void intakeMultiple(int blocks) {

    int accepted = 0;

    while (blocks == 0 || accepted < blocks) {
        if (!autoIntakeEnabled) {
            // hard gate: if disabled, motors must be stopped and we wait here
            stopAllCollectors();
            while (!autoIntakeEnabled) {
                pros::delay(20);
            }
        }

        // run intake while enabled
        intake();

        // wait for a block to show up (or auto to get disabled)
        while (autoIntakeEnabled /*&& colourDet() == 0*/) pros::delay(20);
        if (!autoIntakeEnabled) continue;

        // count each block intaked
        while (autoIntakeEnabled) pros::delay(20);
        accepted++;
    }

    stopAllCollectors();
}

void intakeTask(void*) {
    intakeMultiple(0);
}


// -------------------- PROS Callbacks -------------------- //
/* Init code upon program being run */
bool logDebug = true;
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    // optical.disable_gesture();

    // IMPORTANT: make tasks static so they don't get destroyed when initialize() returns
    static pros::Task screenTask([] {
        while (true) {
            lemlib::Pose chass = chassis.getPose();
            pros::lcd::print(0, "X: %f", chass.x);
            pros::lcd::print(1, "Y: %f", chass.y); 
            pros::lcd::print(2, "Theta: %f", chass.theta); 

            pros::lcd::print(5, "Auto intake %s", (autoIntakeEnabled ? "enabled" : "disabled"));

            // if (logDebug) {
            //     std::printf(
            //         "Chassis:\nX: %f\nY: %f\nTheta: %f\n\nAuto Intake: %s\n\nCollectors:\nFirst: %d\nSecond: %d\nThird: %d\n",
            //         chass.x,
            //         chass.y,
            //         chass.theta,
            //         (autoIntakeEnabled ? "enabled" : "disabled"),
            //         FirstCollector.get_faults(),
            //         SecondCollector.get_faults(),
            //         ThirdCollector.get_faults()
            //     );
            // }

            lemlib::telemetrySink()->info("Chassis pose: {}", chass);

            pros::delay(50);
        }
    });

    // Initialize asynchronous intake task
    static pros::Task autoIntakeTask(intakeTask);
}

/* Match end robot state */
void disabled() {
    feeder.retract();
    wing.retract();
    wing.set_value(false);
}

/* Init code that only runs in competition mode */
void competition_initialize() {}

ASSET(example_txt);

void auto_tune_pid(lemlib::ControllerSettings movementController, bool linear, int margin, int OSCMargin) {
    logDebug = false;
    // bool osc = false;
    while (true) {
        std::printf("Testing (%f, %f)\n", angularController.kP, angularController.kD);
        chassis.setPose(0, 0, 0);
        // osc = false;
        if (linear) {
            chassis.moveToPoint(0, 24, 4999);
            // while (std::abs(chassis.getPose().y-24) > OSCMargin) {
            //     pros::delay(20);
            // }
            // pros::delay(100);
            // while (chassis.isInMotion()) {
            //     if (std::abs(chassis.getPose().y - 24) > OSCMargin) {
            //         osc = true;
            //     }
            //     pros::delay(20);
            // }
        } else {
            chassis.turnToHeading(180, 4999);
            // while (std::abs(chassis.getPose().theta-180) > OSCMargin) {
            //     pros::delay(20);
            // }
            // pros::delay(100);
            // while (chassis.isInMotion()) {
            //     if (std::abs(chassis.getPose().theta - 180) > OSCMargin) {
            //         osc = true;
            //     }
            //     pros::delay(20);
            // }
        }
        // if (osc) {
        //     std::printf("Oscillation on values (%f, %f)\n", movementController.kP, movementController.kD);
        //     movementController.kD += 3;
        // } else {
        //     std::printf("No oscilation on values (%f, %f)\n", movementController.kP, movementController.kD);
        //     movementController.kP += 1;
        // }
        chassis.waitUntilDone();
        std::printf("Change kP and kD accordingly. Left up and down for kP, X and B for kD, and A to finish.\nCurrent values: (%f, %f)\n",movementController.kP, movementController.kD);
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                angularController.kP += 1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                angularController.kP -= 1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                angularController.kD += 1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                angularController.kD -= 1;
            }
            pros::delay(20);
        }

        lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
        
        std::printf("Press A on the controller to continue...\n");
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::delay(20);
        }
        std::printf("----------------------------------\n");
    }
}

/* Code that runs during autonomous period */
int tubeY = 34;
void autonomous() {
    autoIntakeEnabled = false;

    // begin
    chassis.setPose(0, 0, 0);
    feeder.extend(); // extending is slow, therefore call before moving

    // dispenser
    chassis.moveToPoint(0, tubeY, 1400);
    chassis.waitUntilDone();
    chassis.turnToHeading(side*90, 1000); // turn to tube
    chassis.waitUntilDone();

    // move into tube and intake
    chassis.moveToPoint(side*15, tubeY+3, 1500, {.maxSpeed = 80}); // t1000 -> t2500; y+2, -> y+1.5
    chassis.waitUntil(1);
    autoIntakeEnabled = true;
    chassis.waitUntilDone();
    
    // back into long goal and outtake
    chassis.moveToPoint(side*-26, tubeY+3, 1000, {.forwards = false}); // -26 -> -28
    chassis.waitUntil(5); // new addition as a JIC measure; remove if not working
    feeder.retract(); // -37 -> -36, 7 -> 6
    autoIntakeEnabled = false;
    chassis.waitUntilDone();
    topOuttake();
    pros::delay(3000); // 2800

    // back up and turn towards middle goal
    stopAllCollectors();
    chassis.moveToPoint(side*-8, tubeY, 1000);
    chassis.waitUntilDone();
    chassis.turnToPoint(side*-35, 4, 1000); // side*-26, 14
    chassis.waitUntilDone();
    
    // move to middle goal and outtake
    chassis.moveToPoint(side*-33.5, 5.5, 3000, {.maxSpeed = 55}); // -33 -> -34 -> 33.5, 6 -> 5 -> 5.5
    autoIntakeEnabled = true;
    // ******
    chassis.waitUntil(15);
    feeder.extend();
    chassis.waitUntil(10);
    feeder.retract();
    // ******
    chassis.waitUntilDone();
    autoIntakeEnabled = false;
    pros::delay(200);
    bottomOuttake();
    chassis.waitUntilDone();

    // chassis.moveToPoint(side*-20, tubeY-7, 1000, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 1000);
    // chassis.waitUntilDone();
    // wing.retract();
    // chassis.moveToPoint(side*-40, tubeY-7, 1000, {.forwards = false});
    // chassis.waitUntilDone();

    // *******************

    // chassis.moveToPoint(0, 6.674, 500); // 6.674
    // chassis.waitUntilDone();
    // chassis.turnToHeading(side*45, 500); // 90
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*8.933, 15.607, side*45, 1000, {.horizontalDrift = 8, .lead = 0});
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*17.73, 27.53, side*45, 1000, {.horizontalDrift = 8, .lead = 0, .maxSpeed = 48}); // 29.25, 29.25
    // chassis.turnToHeading(side*135, 800);
    // chassis.waitUntilDone();

    // Score the bottom tube of the middle goal
    // chassis.turnToPoint(side*6, 48.5, 1000)
    // chassis.moveToPose(side*6, 48.5, side*-45, 1000, {.horizontalDrift = 8, .minSpeed = 127});
    // pros::delay(500);
    // autoIntakeEnabled = false;
    // chassis.waitUntilDone();
    // if (side) bottomOuttake();
    // else midOuttake();
    // pros::delay(1000);

    // // Move to between long goal and dispenser
    // chassis.moveToPose(side*46.77, -16.5, 180, 800, {.horizontalDrift = 8, .minSpeed = 127}); // REMINDER: if this doesnt work, increase timeout
    // pros::delay(1000);

    // // Collect balls from dispenser
    // autoIntakeEnabled = true;
    // feeder.extend();
    // chassis.waitUntilDone();
    // pros::delay(1000);
    // chassis.moveToPose(side*46.77, 18.73, 180, 1000, {.forwards = false, .horizontalDrift = 8, .lead = 0});
    // autoIntakeEnabled = false;
    // chassis.waitUntilDone();
    // intake();
    // pros::delay(1000);

    // // 🧐
    // chassis.moveToPose(side*37, 10, 180, 1000);
    // wing.extend();
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*37, 48.5, 180, 1000, {.forwards = false, .horizontalDrift = 8, .lead = 0, .minSpeed = 127});

    // *** OLD CODE ****************

    // chassis.moveToPoint(0, 6.674, 500); // 6.674
    // chassis.waitUntilDone();
    // chassis.turnToHeading(side*45, 500); // 90
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*8.933, 15.607, side*45, 1000, {.lead = 0});
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*17.73, 27.53, side*45, 2500, {.lead = 0, .maxSpeed = 48}); // 29.25, 29.25
    // chassis.turnToHeading(side*135, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*46.22, -10, side*180, 3000, {.minSpeed = 127}); // 45.72 -> 46.22
    
    // feeder.extend();
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*46.77, -16.5, side*180, 800, {.lead = 0, .minSpeed = 127}); // 45.72 -> 45.22 -> 46.22 -> 46.72
    // chassis.waitUntilDone();
    // pros::delay(1000);
    // feeder.retract();
    // autoIntakeEnabled = false;
    // chassis.moveToPose(side*46.77, 0, 0, 800, {.forwards=false});
    // chassis.turnToHeading(0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*46.77, 18.73, side*13, 1000, {.lead = 0}); // working: 41, 14.73
    // chassis.waitUntilDone();
    // topOuttake();
}

// -------------------- Driver Control -------------------- //
bool manual = true;
bool feederExtended = false;
bool stopperExtended = true;
bool driveDirection = true; // default direction, brain side
int modifier = driveDirection ? 1 : -1;

/* Code that runs during driver control; The manual controls for the robot */
void opcontrol() {
    autoIntakeEnabled = false;

    while (true) {
        int leftY = modifier*controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // toggle manual/auto (debounced)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            manual = !manual;
            autoIntakeEnabled = !manual;
            if (manual) stopAllCollectors(); // manual takes control immediately
        }

        // Manual intake and outtake controls
        if (manual) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                bottomOuttake();
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake();
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                midOuttake();
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                topOuttake();
            } else {
                stopAllCollectors();
            }
        }

        // Wing control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            wing.extend(); 
        } else {
            wing.retract();
        }

        // Feeder control
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            feederExtended = !feederExtended;
            if (feederExtended) {
                feeder.extend();
            } else {
                feeder.retract();
            }
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            driveDirection = !driveDirection;
            modifier = driveDirection ? 1 : -1;
        }

        // AUTO PID TUNING ********************************************************

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            logDebug = false;
            auto_tune_pid(angularController, false, 2, 5);
        }

        // ************************************************************************

        pros::delay(25);
    }
}
