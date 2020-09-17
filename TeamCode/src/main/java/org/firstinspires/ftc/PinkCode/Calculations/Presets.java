package org.firstinspires.ftc.PinkCode.Calculations;

// Abstract Class to Define Preset Values Used Throughout Subsystems
public abstract class Presets {
    //Collector Presets
    public static final double COLLECTOR_COLLECT_POWER = .8; // Power Sent to Motor While Collecting
    public static final double COLLECTOR_EJECT_POWER = -.2; // Power Sent to Motor While Ejecting

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
}