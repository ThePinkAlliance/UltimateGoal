package org.firstinspires.ftc.PinkCode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;

import org.jetbrains.annotations.NotNull;

import java.util.List;

public abstract class PinkNavigate extends ThreeTrackingWheelLocalizer {

    private static final double COUNTS_PER_INCH = 24.9; // Counts Previous 49.8
    private static final double STRAFE_COUNTS_PER_INCH = 31.1;
    private static Pose2d currentPos = new Pose2d(0, 0);

    // Doc https://acme-robotics.gitbook.io/road-runner/tour/parametric-paths
    // Could use Line or quintic spline for nav across the field

    public PinkNavigate(@NotNull List<Pose2d> wheelPoses) {
        super(wheelPoses);
    }

    public static void DriveToPos(Vector2d drivePos) {
        PathBuilder builder = new PathBuilder(currentPos);

        Path navToPath = builder.lineTo(drivePos).build();

        Drive(navToPath.start());
    }

    private static void Drive(Pose2d navPos) {
        double x = navPos.getX();
        double y = navPos.getY();
        double heading = navPos.getHeading();

        // I think encoder measures in mm so convert mm to inch
        double leftToPos = Subsystem.robot.encoder_left.getCurrentPosition() * COUNTS_PER_INCH;
        double rightToPos = Subsystem.robot.encoder_right.getCurrentPosition() * COUNTS_PER_INCH;
        double center = Subsystem.robot.encoder_center.getCurrentPosition() * COUNTS_PER_INCH;

        Base.drive_by_command(false,1 ,1,1,1);
    }

    public static double encoder_left_to_count() {
        return Subsystem.robot.encoder_left.getCurrentPosition() * COUNTS_PER_INCH;
    }

    public static double encoder_left_pos() {
        return Subsystem.robot.encoder_left.getCurrentPosition();
    }

    public static double encoder_right_pos() {
        return Subsystem.robot.encoder_right.getCurrentPosition();
    }

    public static double encoder_right_count() {
        return Subsystem.robot.encoder_right.getCurrentPosition() * COUNTS_PER_INCH;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }
}

