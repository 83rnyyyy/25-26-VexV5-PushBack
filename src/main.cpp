#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({2, -4, 6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-3, -5, 7}, pros::MotorGearset::blue);

// collectors
pros::Motor FirstCollector(16);   // front bottom collector
pros::Motor SecondCollector(12);  // back collector
pros::Motor ThirdCollector(11);   // front top collector

// sensors
pros::Optical optical(19);
pros::Imu imu(10);

// pneumatics
pros::adi::Pneumatics wing('A', false);
pros::adi::Pneumatics feeder('H', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, lemlib::Omniwheel::NEW_275, 450, 2);

// controllers
lemlib::ControllerSettings linearController(13, 0, 51, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angularController(5, 0, 50, 0, 0, 0, 0, 0, 0);

// tracking wheels
pros::Rotation horizontalEnc(20);
pros::Rotation verticalEnc(-9);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);

// odom sensors
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

// drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// -------------------- QUICK CONFIGURATION --------------------
constexpr bool allianceIsBlue = true; // true = keep BLUE, false = keep RED
constexpr int side = -1; // 1 = right, -1 = left

// -------------------- Intake / Color Sort --------------------

constexpr int PROX_THRESH = 120;       // tune this
volatile bool autoIntakeEnabled = false;
volatile bool rejectMid = true;

int colourDet() {
    int prox = optical.get_proximity();
    if (prox < PROX_THRESH) return 0;

    double hue = optical.get_hue();
    if (hue > 340 || hue < 25) return 1;        // red
    if (hue > 90 && hue < 260) return 2;        // blue
    return 0;
}

void intake() {
    FirstCollector.move(-127);
    SecondCollector.move(-127);
    ThirdCollector.move(127);
}

// void midOuttake() {
//     SecondCollector.move(127);
//     FirstCollector.move(127);
//     ThirdCollector.move(127);
// }

void bottomOuttake() {
    FirstCollector.move(127);
    SecondCollector.move(127);
    ThirdCollector.move(-127);
}

void stopAllCollectors() {
    FirstCollector.move(0);
    SecondCollector.move(0);
    ThirdCollector.move(0);
}

void intakeWithDet() {
    const int keep = allianceIsBlue ? 2 : 1;
    const int reject = allianceIsBlue ? 1 : 2;

    // do nothing unless enabled
    while (!autoIntakeEnabled) {
        stopAllCollectors();
        pros::delay(20);
    }

    FirstCollector.move(127);

    // wait until we actually see something (but allow disable to break out)
    while (autoIntakeEnabled && colourDet() == 0) pros::delay(20);
    if (!autoIntakeEnabled) { stopAllCollectors(); return; }

    int c = colourDet();
    if (c == reject) {
        if (rejectMid) midOuttake();
        else bottomOuttake();
    } else if (c == keep) intake();
}

void intakeWithDetMultiple(int blocks) { // blocks=0 => infinite
    const int keep = allianceIsBlue ? 2 : 1;
    const int reject = allianceIsBlue ? 1 : 2;

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
        while (autoIntakeEnabled && colourDet() == 0) pros::delay(20);
        if (!autoIntakeEnabled) continue;

        int c = colourDet();

        if (c == reject) {
            // eject until the rejected color is gone (or auto disabled)
            if (rejectMid) midOuttake();
            else bottomOuttake();
            while (autoIntakeEnabled && colourDet() == reject) pros::delay(20);
            pros::delay(50);
            if (rejectMid) pros::delay(500);
        } else if (c == keep) {
            // count once per block: wait until it leaves the sensor
            while (autoIntakeEnabled && colourDet() == keep) pros::delay(20);
            accepted++;
        } else {
            // unknown hue while prox high; just let it move through a bit
            pros::delay(30);
        }
    }

    stopAllCollectors();
}

void intakeTask(void*) {
    intakeWithDetMultiple(0);
}

// -------------------- PROS Callbacks --------------------

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    optical.disable_gesture();

    // IMPORTANT: make tasks static so they don't get destroyed when initialize() returns
    static pros::Task screenTask([] {
        while (true) {
            lemlib::Pose chass = chassis.getPose();
            pros::lcd::print(0, "X: %f", chass.x);
            pros::lcd::print(1, "Y: %f", chass.y);
            pros::lcd::print(2, "Theta: %f", chass.theta);

            pros::lcd::print(3, "Hue:  %f", optical.get_hue());
            int color = colourDet();
            pros::lcd::print(4, "Color detected: %s", (color == 0 ? "none" : (color == 1 ? "red" : "blue")));
            pros::lcd::print(5, "Auto intake %s", (autoIntakeEnabled ? "enabled" : "disabled"));

            lemlib::telemetrySink()->info("Chassis pose: {}", chass);
            pros::delay(50);
        }
    });

    static pros::Task autoIntakeTask(intakeTask);
}

void disabled() {}
void competition_initialize() {}

ASSET(example_txt);

void autonomous() {
    autoIntakeEnabled = false;
    rejectMid = true;

    chassis.setPose(side*7.826, 13.319, side*90); // TODO: get coordinates relative of origin pos

    chassis.moveToPose(side*46.77, -16.5, 180, 800, {.minSpeed = 127}); // feeder tube
    pros::delay(1000);
    autoIntakeEnabled = true;
    feeder.extend();
    chassis.waitUntilDone();
    pros::delay(1000);

    chassis.moveToPose(side*46.77, 18.73, 180, 1000, {.forwards = false, .lead = 0}); // long goal
    autoIntakeEnabled = false;
    chassis.waitUntilDone();
    intake();
    pros::delay(1000);

    chassis.moveToPose(side*23.7, 24.4, side*-45, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(17.7, 31.4, side*-45, 1000); // 3 balls
    autoIntakeEnabled = true;
    feeder.extend();
    chassis.waitUntilDone();
    autoIntakeEnabled = false;

    chassis.moveToPose(side*6, 48.5, side*-45, 1000, {.forwards = true}); // middle goal
    pros::delay(500);
    autoIntakeEnabled = false;
    chassis.waitUntilDone();

    // if (side == -1) bottomOuttake();
    // else midOuttake();
    bottomOuttake();
    pros::delay(1000);

    // move to start of pink (get 3 balls)
    chassis.moveToPose(side*-49.23, 18.73, 180, 1000, {.forwards = false, .lead = 0}); // long goal
    autoIntakeEnabled = false;
    chassis.waitUntilDone();
    intake();
    pros::delay(1000);
    
    // chassis.moveToPose(side*9.69, 15, side*45, 1000);
    // chassis.waitUntil(12);
    // autoIntakeEnabled = true;
    // chassis.waitUntilDone();

    // chassis.moveToPose(side*6, 48.5, side*-45, 1000, {.forwards = true});
    // pros::delay(500);
    // autoIntakeEnabled = false;
    // chassis.waitUntilDone();

    // if (side == -1) bottomOuttake();
    // else midOuttake();
    // pros::delay(1000);

    // chassis.moveToPose(side*46.77, -16.5, 180, 800, {.minSpeed = 127}); // REMINDER: if this doesnt work, increase timeout
    // pros::delay(1000);

    // autoIntakeEnabled = true;
    // feeder.extend();
    // chassis.waitUntilDone();
    // pros::delay(1000);
    // chassis.moveToPose(side*46.77, 18.73, 180, 1000, {.forwards = false, .lead = 0});
    // autoIntakeEnabled = false;
    // chassis.waitUntilDone();
    // topOuttake();
    // pros::delay(1000);

    // chassis.moveToPose(side*55, 10, 180, 1000);
    // wing.extend();
    // chassis.waitUntilDone();
    // chassis.moveToPose(side*55, 48.5, 180, 1000, {.forwards = false, .lead = 0, .minSpeed = 127});
    

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
    
    // rejectMid = false;
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

// -------------------- Driver Control --------------------

bool manual = true;
bool feederExtended = false;
bool wingExtended = false;
void opcontrol() {
    autoIntakeEnabled = false;
    rejectMid = true;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // toggle manual/auto (debounced)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            manual = !manual;
            autoIntakeEnabled = !manual;
            if (manual) stopAllCollectors(); // manual takes control immediately
        }

        if (manual) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake();
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                midOuttake();
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                bottomOuttake();
            } else {
                stopAllCollectors();
            }
        }

        // PNEUMATICS
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            wingExtended = !wingExtended;
            if (wingExtended) {
                wing.extend();
            } else {
                wing.retract();
            }
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            feederExtended = !feederExtended;
            if (feederExtended) {
                feeder.extend();
            } else {
                feeder.retract();
            }
        }

        pros::delay(25);
    }
}
