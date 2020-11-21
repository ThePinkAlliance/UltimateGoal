package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.PinkCode.odometry.SampleMecanumDrive;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Conveyor;
import org.firstinspires.ftc.PinkCode.Subsystems.Shooter;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Base;
import org.firstinspires.ftc.PinkCode.Robot.Controls;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RRAuto", group = "Auto")
public class RRAuto extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public int numRings = 0; //0 rings Front (A), 1 ring is mid (B), 4 rings is back (C)

    private enum stages {
        INIT,
        DRIVE_BACK,
        Linear,
        STOP,
    }

    stages stage = stages.DRIVE_BACK;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if(numRings == 4)
            DISTANCE = 105;
        else if (numRings == 1)
            DISTANCE = 80;
        else
            DISTANCE = 67;

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-DISTANCE,0, Math.toRadians(-30)))
                .addTemporalMarker(3.5, () -> {
                    drive.wobble_arm.setPosition(Presets.WOBBLE_DOWN);
                })
                .addTemporalMarker(3.7, () -> {
                    drive.wobble_grip.setPosition(Presets.WOBBLE_UNGRIP);
                })
                .addTemporalMarker(3.9, () -> {
                    drive.wobble_grip.setPosition(Presets.WOBBLE_GRIP);
                })
                .addTemporalMarker(4.0, () -> {
                    drive.wobble_arm.setPosition(Presets.WOBBLE_UP);
                })
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(17,-20, Math.toRadians(210)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

            drive.followTrajectory(trajectory);
            drive.followTrajectory(trajectory1);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}