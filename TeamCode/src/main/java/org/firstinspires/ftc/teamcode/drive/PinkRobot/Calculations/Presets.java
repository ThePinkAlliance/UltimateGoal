package org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations;

// Abstract Class to Define Preset Values Used Throughout Subsystems
public class Presets {
    //Collector Presets
    public static final double COLLECTOR_COLLECT_POWER = -1; // Power Sent to Motor While Collecting
    public static final double COLLECTOR_EJECT_POWER = 1; // Power Sent to Motor While Ejecting
    public static final double COLLECTOR_STOP_POWER = 0;

    public static final double COLLECTOR_HOLDER_RELEASE = 0.45;
    public static final double COLLECTOR_HOLDER_HOLD = 0;

    //Conveyor Commands
    public static final double CONVEYOR_COLLECT_POWER = 1;
    public static final double CONVEYOR_EJECT_POWER = -1;
    public static final double CONVEYOR_STOP_POWER = 0;


    public static final double CONVEYOR_FLAP_OPEN = .22;
    public static final double CONVEYOR_FLAP_CLOSE = 0;

    //Shooter Commands
    public static final double SHOOTER_SHOOT_POWER = 1;
    public static final double SHOOTER_STOP_POWER = 0;
    public static final double SHOOTER_FLAP_CLOSE = 0.22;

    // Under 16ft shooting values
    // Flap 0.392 -
    // RPM 1450

    public static final double SHOOTER_FLAP_OPEN = 0.880; // Short Flap .395; //increase in numbers is down; decrease in numbers is up.
    public static final double SHOOTER_FLAP_POWER_SHOT = SHOOTER_FLAP_OPEN - 0.025;// Short Flap .415; // .41 is good ideal value

    public static final double SHOOTER_FLAP_OPEN_AUTO = SHOOTER_FLAP_OPEN - 0.003; // Short .395 //increase in numbers is down; decrease in numbers is up.
  // public static final double SHOOTER_FLAP_OPEN_AUTO_FAR = .410; // Short .405; //increase in numbers is down; decrease in numbers is up.

    //Wobble Presets
    public static final double WOBBLE_UP = 0.30; //.32;
    public static final double WOBBLE_DOWN = 0.70; //.92;
    public static final double WOBBLE_GRIP = 0;
    public static final double WOBBLE_UNGRIP = 0.65; //.6;

    // Top Gate
    public static final double TOP_GATE_UP = 0.3;
    public static final double TOP_GATE_DOWN = 0.05;

    // Sweeper Servos
    public static final double SWEEPER_LEFT_STOP = 0.5;
    public static final double SWEEPER_LEFT_COLLECT = 0.0;

    public static final double SWEEPER_RIGHT_STOP = 0.5;
    public static final double SWEEPER_RIGHT_COLLECT = 1.0;

    // Ring Blocker
    public static final double RING_BROCKER_FOLDED = 0.0;
    public static final double RING_BROCKER_UP = 0.4;
    public static final double RING_BROCKER_DOWN = 0.80;

    // TeleOp
    public static final double TELEOP_HIGH_PID_RPM_TARGET = 2200; // Good Value 2100;
    public static final double TELEOP_HIGH_PID_RPM_TARGET_LOW = TELEOP_HIGH_PID_RPM_TARGET - 100;
    public static final double TELEOP_HIGH_PID_RPM_TARGET_HIGH = TELEOP_HIGH_PID_RPM_TARGET + 100;

    public static final double TELEOP_POWERSHOT_PID_RPM_TARGET = 1625;
    public static final double TELEOP_POWERSHOT_PID_RPM_TARGET_LOW = TELEOP_POWERSHOT_PID_RPM_TARGET - 500;
    public static final double TELEOP_POWERSHOT_PID_RPM_TARGET_HIGH = TELEOP_POWERSHOT_PID_RPM_TARGET + 500;

    public static final double TELEOP_AUTOAIM_POS = -12.0;

    public static final double TELEOP_AUTOAIM_POWERSHOT_RED_LEFT = 24;
    public static final double TELEOP_AUTOAIM_POWERSHOT_RED_MIDDLE = TELEOP_AUTOAIM_POWERSHOT_RED_LEFT - 7;
    public static final double TELEOP_AUTOAIM_POWERSHOT_RED_RIGHT = TELEOP_AUTOAIM_POWERSHOT_RED_MIDDLE - 7;

    //Hook Presets
    public static final double HOOK_DOWN = .5;
    public static final double HOOK_UP = 1;

    public static int COLLECTOR_HOLDER_INIT_POSITION = 80;
    public static float CAMERA_TO_INTAKE = 6.4f;
}