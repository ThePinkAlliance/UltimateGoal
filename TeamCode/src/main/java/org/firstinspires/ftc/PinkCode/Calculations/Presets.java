package org.firstinspires.ftc.PinkCode.Calculations;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    public static double TICKS_PER_REV = 1000; // in
    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel
    public static final double MAX_RPM = 1;
    public static final boolean RUN_USING_ENCODER = true;
    public static final PIDCoefficients MOTOR_VELO_PID = null;
    public static double TRACK_WIDTH = 1; // in

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    // Convert Encoder Values to Inch
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MAX_RPM * TICKS_PER_REV);
    }
}