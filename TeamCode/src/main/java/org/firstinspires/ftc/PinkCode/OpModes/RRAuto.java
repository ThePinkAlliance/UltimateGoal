package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.PinkCode.odometry.SampleMecanumDrive;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RRAuto", group = "Auto")
public class RRAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public static double DISTANCE = 60; // in
    private BNO055IMU imu;
    private ElapsedTime runtime;
    private SampleMecanumDrive navigate;

    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public int numRings = 0; //0 rings Front (A), 1 ring is mid (B), 4 rings is back (C)

    @Override
    public void runOpMode() throws InterruptedException {
        Subsystem.robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //imu initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        runtime = new ElapsedTime();
        runtime.startTime();

//            telemetry.addData("Timer", runtime.time());
//            Subsystem.robot.leftB_drive.setPower(0);


        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        // add a delay
        sleep(1000);

        int amount = updatedRecognitions.size();

        // sometimes tensorflow wont find all 3 instead of 4



        /** Wait for the game to begin */
        telemetry.addData("amount", amount);
        telemetry.addData("distance", DISTANCE);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

//        if(numRings == 4)
//            DISTANCE = 105;
//        else if (numRings == 1)
//            DISTANCE = 80;
//        else
//            DISTANCE = 56;

//        Trajectory trajectory5 = drive.trajectoryBuilder(new Pose2d())
//                .forward(1)
//                .addTemporalMarker(3.0, () -> {
//                    amount.set(updatedRecognitions.size());
//
//                    telemetry.addData("amount", amount.toString());
//                    telemetry.update();
//                })
//                .build();

        if(amount == 4)
            DISTANCE = 105;
        else if (amount == 3)
            DISTANCE = 105;
        else if (amount == 1)
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


//            drive.followTrajectory(trajectory5);
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

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = Subsystem.robot.webcam;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}