package org.firstinspires.ftc.PinkCode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class PinkNavigate extends ThreeTrackingWheelLocalizer {
    public PinkNavigate() {
        super(Arrays.asList(
                new Pose2d(0.0,0.0),
                new Pose2d(0.0,0.0),
                // Math.toDegrees(90) for perpendicular
                new Pose2d(0.0, 0.0, Math.toDegrees(90))
        ));
    }

    public void DriveToPos(double leftF, double leftB, double rightF, double rightB) {

    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }
}

