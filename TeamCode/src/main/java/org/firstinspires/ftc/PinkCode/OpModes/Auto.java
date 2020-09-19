package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;

import java.util.List;

public class Auto extends OpMode {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    private PIDFController controller;
    private DriveConstraints constraints;
    private TrajectoryFollower trajectory_follower;
    private MotionProfile profile;
    private List<Pose2d> position_history;

    @Override
    public void init() {
    }

    @Override
    public void loop() {

    }

//    public void turnAsync(double angle) {
//        double heading = getPoseEstimate().getHeading();
//
//        lastPoseOnTurn = getPoseEstimate();
//
//        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(heading, 0, 0, 0),
//                new MotionState(heading + angle, 0, 0, 0),
//                constraints.maxAngVel,
//                constraints.maxAngAccel,
//                constraints.maxAngJerk
//        );
//
//        turnStart = clock.seconds();
//        mode = Mode.TURN;
//    }
}
