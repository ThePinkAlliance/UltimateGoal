package org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations;

// Abstract Class to Define Preset Values Used Throughout Subsystems
public class Presets {
    //Collector Presets
    public static final double COLLECTOR_COLLECT_POWER = -1; // Power Sent to Motor While Collecting
    public static final double COLLECTOR_EJECT_POWER = 1; // Power Sent to Motor While Ejecting
    public static final double COLLECTOR_STOP_POWER = 0;
    public static final double COLLECTOR_HOLDER_RELEASE = 1;
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
    public static final double SHOOTER_FLAP_POWER_SHOT = .415; // .41 is good ideal value
    public static final double SHOOTER_FLAP_OPEN = .395; //increase in numbers is down; decrease in numbers is up.

    //Wobble Presets
    public static final double WOBBLE_UP = .32;
    public static final double WOBBLE_DOWN = .92;
    public static final double WOBBLE_GRIP = 0;
    public static final double WOBBLE_UNGRIP = .6;

    //Hook Presets
    public static final double HOOK_DOWN = .5;
    public static final double HOOK_UP = 1;

    public static int COLLECTOR_HOLDER_INIT_POSITION = 80;
    public static float CAMERA_TO_INTAKE = 6.4f;
}