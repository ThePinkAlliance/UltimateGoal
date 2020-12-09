package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.PinkCode.odometry.SampleMecanumDrive;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Config
@Autonomous(name = "RRAuto", group = "Auto")
public class RRAuto extends LinearOpMode {
    public static double markedTime = 0;
    private ElapsedTime runtime = new ElapsedTime();
    public static double DISTANCE = 60; // in
    public static double ANGLE = -20;
    public static double CenteringAngle = 0;
    public static double CenteringDistance = 0;
    public int numRings = 0; //0 rings Front (A), 1 ring is mid (B), 4 rings is back (C)
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private Vector2d pos_from_home = new Vector2d(0, 0);

    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private enum Stage {
        INITIALIZE,
        RUN,
        STOP
    }

    private enum States {
        INIT,
        DRIVE_TO_TARGETS,
        GO_TO_HOME_POSITION,
        FIND_TARGETS,
        SHOOT,
        STOP
    }

    private Stage stage = Stage.INITIALIZE;
    private States states = States.INIT;

    @Override
    public void runOpMode() throws InterruptedException {

        Subsystem.robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (numRings == 4) {
            ANGLE = -20;
            DISTANCE = -102;
            CenteringAngle = 198;
            CenteringDistance = 49;
        } else if (numRings == 1) {
            ANGLE = 20;
            DISTANCE = -79;
            CenteringAngle = 158;
            CenteringDistance = 26;
        } else {
            ANGLE = -20;
            DISTANCE = -56;
            CenteringDistance = 3;
            CenteringAngle = 198;
        }

        telemetry.addData("DISTANCE:", DISTANCE);
        telemetry.addData("Time:", runtime.milliseconds() - markedTime);
        telemetry.update();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(DISTANCE, 0, Math.toRadians(ANGLE)))
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
                    Subsystem.robot.shoot2.setPower(.7);
                })
                .addTemporalMarker(4.6, () -> {
                    Subsystem.robot.shoot1.setPower(.7);
                })
                .addTemporalMarker(4.6, () -> {
                    Subsystem.robot.shoot_flap.setPosition(Presets.SHOOTER_FLAP_OPEN);
                })
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(CenteringDistance, -20, Math.toRadians(CenteringAngle)))
                .addTemporalMarker(7.5, () -> {
                    Subsystem.robot.conveyor.setPower(1);
                })
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