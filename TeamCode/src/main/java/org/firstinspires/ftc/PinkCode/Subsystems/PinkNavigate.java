package org.firstinspires.ftc.PinkCode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;

import org.jetbrains.annotations.NotNull;

import java.util.List;

public class PinkNavigate extends ThreeTrackingWheelLocalizer {

    private final double COUNTS_PER_INCH = 24.9; // Counts Previous 49.8
    private final double STRAFE_COUNTS_PER_INCH = 31.1;
    private Pose2d currentPos = new Pose2d(0, 0);

    // Doc https://acme-robotics.gitbook.io/road-runner/tour/parametric-paths
    // Could use Line or quintic spline for nav across the field

    public PinkNavigate(@NotNull List<Pose2d> wheelPoses) {
        super(wheelPoses);
    }

    public void DriveToPos(Vector2d drivePos) {
        PathBuilder builder = new PathBuilder(currentPos);

        Path navToPath = builder.lineTo(drivePos).build();

        Drive(navToPath.start());
    }

    private void Drive(Pose2d navPos) {
        // add encoders to hardware map

        double left = Subsystem.robot.encoder_left.getCurrentPosition();
        double right = Subsystem.robot.encoder_right.getCurrentPosition();
        double center = Subsystem.robot.encoder_center.getCurrentPosition();

        Subsystem.robot.rightF_drive.setPower(1);
        Subsystem.robot.leftF_drive.setPower(1);
        Subsystem.robot.rightB_drive.setPower(1);
        Subsystem.robot.leftB_drive.setPower(1);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }
}

