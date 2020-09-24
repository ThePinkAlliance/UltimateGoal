package org.firstinspires.ftc.PinkCode.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.odometry.WheelTracker;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public abstract class PinkNavigate {

    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TICKS_PER_REV = 1; // in
    private static Pose2d currentPos = new Pose2d(0, 0);
    private static Timer timer;
    private static WheelTracker tracker = new WheelTracker();

    private static DriveConstraints driveConstraints = new DriveConstraints(1, 1, 1, 1, 1, 1);
//    private static PIDCoefficients pidCoefficients = new PIDCoefficients(1, 2, 1);
//    private static PIDFController controller = new PIDFController(pidCoefficients);

    // Doc https://acme-robotics.gitbook.io/road-runner/tour/parametric-paths
    // Could use Line or quintic spline for nav across the field

    public PinkNavigate() {

    }

    public static void Init(boolean blueAlliance) {
        if (blueAlliance) {
            // Heading and pos is Subject to change
            currentPos = new Pose2d(0, 0, 0);
        } else {
            // Heading and pos is Subject to change
            currentPos = new Pose2d(0, 0, 90);

            timer = new Timer(false);

            Base.drive_by_command(false, 1, 1, 1, 1);

            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    Base.drive_stop();
                }
            }, 500);
        }
    }

    public static void DriveToPos(Pose2d drivePos) {
        // X and Y in inch format
        double x = drivePos.getX();
        double y = drivePos.getY();
        double heading = drivePos.getHeading();

        // I think encoder measures in mm so convert mm to inch
        double leftToPos = Presets.encoderTicksToInches(Subsystem.robot.encoder_left.getCurrentPosition());
        double rightToPos = Presets.encoderTicksToInches(Subsystem.robot.encoder_left.getCurrentPosition());
        double center = Presets.encoderTicksToInches(Subsystem.robot.encoder_center.getCurrentPosition());

        Vector2d currentPosVec = new Vector2d(x, y);

        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(currentPos, driveConstraints);

        trajectoryBuilder.splineTo(currentPosVec, 0).splineTo(new Pose2d(0, 0, 0).vec(), 0);

        // MecanumPIDVAFollower follower 
    }

    public static List<Double> getWheelPositions() {
        return tracker.getWheelPositions();
    }
}

