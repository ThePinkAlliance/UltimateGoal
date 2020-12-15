package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;


import org.firstinspires.ftc.PinkCode.odometry.SampleMecanumDrive;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;


import java.net.ProxySelector;
import java.sql.Time;
import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RRAuto", group = "Auto")
public class RRAuto extends LinearOpMode {

    public static double markedTime = 0;
    private ElapsedTime runtime = new ElapsedTime();
    public static double DISTANCE = 60; // in
    public static double ANGLE = -20;
    public static double TimeChecker = 12000;
    public static double CenteringAngle = 0;
    public static double CenteringDistanceX = 0;
    public static double CenteringDistanceY = 0;
    public String numRings = ""; //0 rings Front (A), 1 ring is mid (B), 4 rings is back (C)
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private Vector2d pos_from_home = new Vector2d(0, 0);

    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {

        Subsystem.robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("TFOD ON:", 1);
        telemetry.update();
        markedTime = runtime.milliseconds();


        while(!isStarted()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (Recognition r: updatedRecognitions) {
                        numRings = r.getLabel().toLowerCase();
                        telemetry.addData("obj", r.getLabel());
                    }
                    telemetry.addData("updatedRecognitions", updatedRecognitions.toArray());
                    telemetry.update();
                }
            }
        }

        telemetry.update();

        if (numRings.equals("quad")) {
            ANGLE = -25;
            DISTANCE = -103;
            CenteringAngle = 210;
            markedTime = 4.0;
            CenteringDistanceX = 77;
            CenteringDistanceY = -3;
        } else if (numRings.equals("single")) {
            ANGLE = 10;
            DISTANCE = -47;
            markedTime = 2.3;
            CenteringAngle = 177;
            CenteringDistanceX = 37;
            CenteringDistanceY = -12;
        } else {
            ANGLE = -22;
            markedTime = 3.0;
            DISTANCE = -56;
            CenteringDistanceX = 1;
            CenteringDistanceY = -20;
            CenteringAngle = 181;
        }

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .back(35)
                .addTemporalMarker(.2, () -> {
                    Subsystem.robot.wobble_arm.setPosition(Presets.WOBBLE_UP);
                })
                .addTemporalMarker(.2, () -> {
                    Subsystem.robot.wobble_grip.setPosition(Presets.WOBBLE_GRIP);
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(DISTANCE, 0, Math.toRadians(ANGLE)))
                .addTemporalMarker(.2, () -> {
                    Subsystem.robot.wobble_arm.setPosition(Presets.WOBBLE_UP);
                })
                .addTemporalMarker(.2, () -> {
                    Subsystem.robot.wobble_grip.setPosition(Presets.WOBBLE_GRIP);
                })
                .addTemporalMarker(markedTime + .0, () -> {
                    Subsystem.robot.wobble_arm.setPosition(Presets.WOBBLE_DOWN);
                })
                .addTemporalMarker(markedTime + .1, () -> {
                    Subsystem.robot.conveyor_flap.setPosition(Presets.CONVEYOR_FLAP_OPEN);
                })
                .addTemporalMarker(markedTime + .5, () -> {
                    Subsystem.robot.wobble_grip.setPosition(Presets.WOBBLE_UNGRIP);
                })
                .addTemporalMarker( markedTime + 1.3, () -> {
                    Subsystem.robot.wobble_arm.setPosition(Presets.WOBBLE_UP);
                })
                .addTemporalMarker(markedTime + 1.5, () -> {
                    Subsystem.robot.shoot2.setPower(.7);
                })
                .addTemporalMarker(markedTime + 1.6, () -> {
                    Subsystem.robot.shoot1.setPower(.7);
                })
                .addTemporalMarker(markedTime + 1.6, () -> {
                    Subsystem.robot.shoot_flap.setPosition(Presets.SHOOTER_FLAP_OPEN - .005);
                })
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(CenteringDistanceX, CenteringDistanceY, Math.toRadians(CenteringAngle)))
                .addTemporalMarker(7.5, () -> {
                    Subsystem.robot.conveyor.setPower(.5);
                })
                .build();

        Trajectory tracjectory4 = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .addTemporalMarker(18, () -> {
                    Subsystem.robot.shoot2.setPower(0);
                })
                .addTemporalMarker(18.1, () -> {
                    Subsystem.robot.shoot1.setPower(0);
                })
                .addTemporalMarker(18.3, () -> {
                    Subsystem.robot.conveyor.setPower(0);
                })
                .build();

        waitForStart();

        markedTime = runtime.milliseconds();
        if(numRings.equals("quad")) {
            TimeChecker = 16000;
        } else if(numRings.equals("single")) {
            TimeChecker = 15000;
        } else {
            TimeChecker = 12000;
        }
        if (isStopRequested()) return;

        if(numRings.equals("single")) {
            drive.followTrajectory(trajectory1);
        }
        drive.followTrajectory(trajectory2);
        drive.followTrajectory(trajectory3);
        while(runtime.milliseconds() - markedTime < TimeChecker) {
            telemetry.addData("waiting","waitihgm");
            telemetry.update();
        }
        drive.followTrajectory(tracjectory4);

        if(!(Subsystem.robot.shoot2.getVelocity() > 1450)) {
            Subsystem.robot.conveyor.setPower(0);
        } else {
            Subsystem.robot.conveyor.setPower(.5);
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

            if (tfod != null) {
                tfod.deactivate();
            }
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = Subsystem.robot.webcam;


        ClassFactory.getInstance().getCameraManager().nameForUnknownCamera();

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}