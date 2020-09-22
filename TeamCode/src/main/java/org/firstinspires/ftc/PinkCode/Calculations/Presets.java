package org.firstinspires.ftc.PinkCode.Calculations;

// Abstract Class to Define Preset Values Used Throughout Subsystems
public abstract class Presets {
    //Collector Presets
    public static final double COLLECTOR_COLLECT_POWER = .8; // Power Sent to Motor While Collecting
    public static final double COLLECTOR_EJECT_POWER = -.2; // Power Sent to Motor While Ejecting

    //Conveyor Commands
    public static final double CONVEYOR_COLLECT_POWER = 1;
    public static final double CONVEYOR_EJECT_POWER = -1;

    //Scorer Presets
    public static final double SCORER_STOW = .05;
    public static final double SCORER_HIGH = .65;
    public static final double SCORER_SCORE_POSITION = 1;
    public static final double SCORER_COLLECT = .85;
    public static final double SCORER_EJECT = .3;
    public static final double CAP_EJECT = 1;
    public static final double CAP_STOW = 0;

    //Hook Presets
    public static final double HOOK_DOWN = .5;
    public static final double HOOK_UP = 1;

    // Odometry
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TICKS_PER_REV = 1; // in
    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    // Convert Encoder Values to Inch
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}