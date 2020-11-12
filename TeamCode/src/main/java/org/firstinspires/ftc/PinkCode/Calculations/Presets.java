package org.firstinspires.ftc.PinkCode.Calculations;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;

// Abstract Class to Define Preset Values Used Throughout Subsystems
public abstract class Presets {
    //Collector Presets
    public static final double COLLECTOR_COLLECT_POWER = -1; // Power Sent to Motor While Collecting
    public static final double COLLECTOR_EJECT_POWER = 1; // Power Sent to Motor While Ejecting
    public static final double COLLECTOR_STOP_POWER = 0;
    public static final double COLLECTOR_DROP = 1;

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
    public static final double SHOOTER_FLAP_POWER_SHOT = .41;
    public static final double SHOOTER_FLAP_OPEN = .4; //increase in numbers is down; decrease in numbers is up.

    //Wobble Presets
    public static final double WOBBLE_UP = .3;
    public static final double WOBBLE_DOWN = .7;
    public static final double WOBBLE_GRIP = .7;
    public static final double WOBBLE_UNGRIP = 0;


    //Hook Presets
    public static final double HOOK_DOWN = .5;
    public static final double HOOK_UP = 1;

    // Odometry
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TICKS_PER_REV = 1; // in
    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel
    public static final double MAX_RPM = 0.1;
    public static final boolean RUN_USING_ENCODER = true;
    public static final PIDCoefficients MOTOR_VELO_PID = null;
    public static double TRACK_WIDTH = 1; // in
    public static double SCORER_MAX_Z = 80;

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    public static int COLLECTOR_HOLDER_INIT_POSITION = 80;
    public static float CAMERA_TO_INTAKE = 6.4f;

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