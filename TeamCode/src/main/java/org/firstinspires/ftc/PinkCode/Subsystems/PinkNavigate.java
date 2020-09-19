package org.firstinspires.ftc.PinkCode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public abstract class PinkNavigate extends ThreeTrackingWheelLocalizer {

    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TICKS_PER_REV = 1; // in
    private static Pose2d currentPos = new Pose2d(0, 0);

    // Doc https://acme-robotics.gitbook.io/road-runner/tour/parametric-paths
    // Could use Line or quintic spline for nav across the field

    public PinkNavigate(@NotNull List<Pose2d> wheelPoses) {
        super(wheelPoses);
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



        PathBuilder builder = new PathBuilder(currentPos);

        Path navToPath = builder.lineTo(drivePos.vec()).build();
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }
}

