package org.firstinspires.ftc.PinkCode.Subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;
import org.firstinspires.ftc.PinkCode.odometry.WheelTracker;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.VuforiaWebcam;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public abstract class PinkNavigate {

//    public static double WHEEL_RADIUS = 2; // in
//    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
//    public static double TICKS_PER_REV = 1; // in
    private static Pose2d currentPos = new Pose2d(0, 0);
    private static Timer timer;
    private static WheelTracker tracker = new WheelTracker();

    private static DriveConstraints driveConstraints = new DriveConstraints(1, 1, 1, 1, 1, 1);
//    private static PIDCoefficients pidCoefficients = new PIDCoefficients(1, 2, 1);
//    private static PIDFController controller = new PIDFController(pidCoefficients);

    // Doc https://acme-robotics.gitbook.io/road-runner/tour/parametric-paths
    // Could use Line or quintic spline for nav across the field

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
        //double heading = drivePos.getHeading();

        if (Drive(new Pose2d(x, y))) {
            Base.drive_stop();
        }
    }

    private static boolean Drive(Pose2d driveTo) {


        return (false);
    }

    public static List<Double> getWheelPositions() {
        return tracker.getWheelPositions();
    }
}

