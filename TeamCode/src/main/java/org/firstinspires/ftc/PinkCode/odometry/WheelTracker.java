package org.firstinspires.ftc.PinkCode.odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class WheelTracker extends ThreeTrackingWheelLocalizer {
    public WheelTracker() {
        super(Arrays.asList(
                new Pose2d(0, Presets.LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -Presets.LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(0, Presets.FORWARD_OFFSET, Math.toRadians(90)) // front
        ));
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                // Left Encoder
                Presets.encoderTicksToInches(Subsystem.robot.encoder_left.getCurrentPosition()),
                // Right Encoder
                Presets.encoderTicksToInches(Subsystem.robot.encoder_right.getCurrentPosition()),
                // Center Encoder
                Presets.encoderTicksToInches(Subsystem.robot.encoder_center.getCurrentPosition())
        );
    }

    @NotNull
    public List<Double> getWheelVelocity() {
        return Arrays.asList(
//                Presets.encoderTicksToInches(Subsystem.robot.encoder_left.getRawVelocity()),
//                Presets.encoderTicksToInches(Subsystem.robot.encoder_right.getRawVelocity()),
//                Presets.encoderTicksToInches(Subsystem.robot.encoder_center.getRawVelocity())
        );
    }
}
