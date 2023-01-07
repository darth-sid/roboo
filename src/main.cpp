#include "main.h"

Motor DiskIntake(1, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor Flywheel(15, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_COUNTS);
ADIDigitalOut Expansion(4);
ADIDigitalOut Hood(8);
Optical optical(3);

Controller controller(E_CONTROLLER_MASTER);

// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-2, -13, -12}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{10, 16, 20}

  // IMU Port
  ,17

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.667

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);

#define RED optical.get_hue() < 100.0
#define BLUE optical.get_hue() > 150.0

bool isFrontClamp = true;
bool isOpen = true;
bool isConveyerUp = true;
bool isConveyerDown = true;
bool isRoller = true;
bool isFlywheel = true;

bool isHood = true;
bool isExpand = true;

void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();

  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.
  auto1R();
  //ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  while (true) {
    bool roller = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L2);
    bool expand = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT);
    bool flywheel = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_R1);
    bool flywheelFast = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X);
    bool hoodctl = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B);
    bool intakeUp = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1); // || controller2.get_digital_new_press(E_CONTROLLER_DIGITAL_R1);;
    bool index = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_R2); // || controller2.get_digital_new_press(E_CONTROLLER_DIGITAL_L1);

    //roller
    if (roller) {
        optical.set_led_pwm(100);
        while (RED) {
          if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) break;
          DiskIntake.move(110);

          delay(20);
          chassis.set_angle(0);
          chassis.set_drive_pid(1000, 40, true);
        }
        DiskIntake.move(0);
        optical.set_led_pwm(0);
    }
    //expand
    if (expand) {
      Expansion.set_value(true);
    }
    //flywheel
    if (flywheel) {
      isFlywheel ? (Flywheel.move(-100)) : Flywheel.move(0);
      isFlywheel = !isFlywheel;

    }
    //flywheel nyoom
    if (flywheelFast) {
      isFlywheel ? (Flywheel.move(-127)) : Flywheel.move(0);
      isFlywheel = !isFlywheel;

    }
    //hood(?)
    if (hoodctl) {
      Hood.set_value(isHood);
      isHood = !isHood;
    }
    //intake
    if (intakeUp) {
        if (isConveyerUp) {
            DiskIntake.move(-127);
            Hood.set_value(true);
            isHood = true;
        }
        else {
              DiskIntake.move(0);
          }
        isConveyerUp = !isConveyerUp;

    }
    //index
    if (index) {
        if (isConveyerDown && !isFlywheel) {
          DiskIntake.move(127);
          Hood.set_value(false);
          isHood = false;
        } else {
          DiskIntake.move(0);
        }
        isConveyerDown = !isConveyerDown;
    }
    //chassis.tank(); // Tank control
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
