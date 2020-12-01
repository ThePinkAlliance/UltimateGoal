package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.PinkCode.odometry.SampleMecanumDrive;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Conveyor;
import org.firstinspires.ftc.PinkCode.Subsystems.Shooter;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Base;
import org.firstinspires.ftc.PinkCode.Robot.Controls;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;

import java.net.ProxySelector;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RRAuto", group = "Auto")
public class RRAuto extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public int numRings = 0; //0 rings Front (A), 1 ring is mid (B), 4 rings is back (C)

    @Override
    public void runOpMode() throws InterruptedException {
        Subsystem.robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if(numRings == 4)
            DISTANCE = 105;
        else if (numRings == 1)
            DISTANCE = 80;
        else
            DISTANCE = 56;

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-DISTANCE,0, Math.toRadians(-20)))
                .addTemporalMarker(3.0, () -> {
                    Subsystem.robot.wobble_arm.setPosition(Presets.WOBBLE_DOWN);
                })
                .addTemporalMarker(3.1, () -> {
                    Subsystem.robot.conveyor_flap.setPosition(Presets.CONVEYOR_FLAP_OPEN);
                })
                .addTemporalMarker(3.5, () -> {
                    Subsystem.robot.wobble_grip.setPosition(Presets.WOBBLE_UNGRIP);
                })
                .addTemporalMarker(4.3, () -> {
                    Subsystem.robot.wobble_arm.setPosition(Presets.WOBBLE_UP);
                })
                .addTemporalMarker(4.5, () -> {
                    Subsystem.robot.shoot2.setPower(.8);
                })
                .addTemporalMarker(4.6, () -> {
                    Subsystem.robot.shoot1.setPower(.8);
                })
                .addTemporalMarker(4.6, () -> {
                    Subsystem.robot.shoot_flap.setPosition(Presets.SHOOTER_FLAP_POWER_SHOT - .01);
                })
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(3,-20, Math.toRadians(190)))
                .addTemporalMarker(8, () -> {
                    Subsystem.robot.conveyor.setPower(1);
                })
                .build();

//        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
//                .lineToLinearHeading(new Pose2d(1,1,Math.toRadians())).
//                .build();

        waitForStart();

        if (isStopRequested()) return;

            drive.followTrajectory(trajectory);
            drive.followTrajectory(trajectory1);
            drive.turn(Math.toRadians(20));
            drive.turn(Math.toRadians(20));

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}