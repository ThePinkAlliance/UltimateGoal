package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Conveyor;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigate;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigateTest;
import org.firstinspires.ftc.PinkCode.odometry.SampleMecanumDrive;
import org.firstinspires.ftc.PinkCode.odometry.WheelTracker;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "Auto", group = "Auto")
//@Disabled
public class Auto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private States state = States.INIT;
    private BNO055IMU imu;
    private SampleMecanumDrive navigate;
    private ElapsedTime runtime;
    private WheelTracker tracker;
    private enum States {
        INIT,
        ONE_STACK,
        NO_STACK,
        THREE_STACK,
        STOP,
        PARK
    }

    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        navigate = new SampleMecanumDrive(hardwareMap);
        Subsystem.set_motor_powers();
        Subsystem.set_servo_positions();

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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("IMU", imu.getAngularOrientation());
                telemetry.addData("Encoder Positions", tracker.getWheelPositions());
                telemetry.addData("Encoder Speeds", tracker.getWheelVelocity());

                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        switch (state) {
                            /*
                             * C : Three Stack
                             * B : One Stack
                             * A : None
                             */
                            case INIT:
                                navigate.setPoseEstimate(new Pose2d(0,0, Math.toRadians(0)));
                                telemetry.addData("state", "init");
                                telemetry.update();
                                final int amount = updatedRecognitions.size();

                                if (amount == 0) {
                                    state = States.NO_STACK;
                                } else if (amount == 1) {
                                    state = States.ONE_STACK;
                                } else if (amount == 3) {
                                    state = States.THREE_STACK;
                                } else if (amount == 4) {
                                    state = States.THREE_STACK;
                                } else {
                                    state = States.NO_STACK;
                                }
                                break;

                            case NO_STACK:
                                telemetry.addData("state", "No Stack");
                                telemetry.addData("pos", navigate.getWheelPositions());
                                telemetry.update();
                                Trajectory none = navigate.trajectoryBuilder(new Pose2d(0,0))
                                        .splineToLinearHeading(new Pose2d(-60, -10), Math.toRadians(-90))
                                        .addDisplacementMarker(() -> {
                                            Wobble.wobble_arm_down();
                                            Subsystem.set_servo_positions();
                                            Wobble.wobble_ungrip();
                                            Subsystem.set_servo_positions();

                                            Wobble.wobble_arm_up();
                                            Subsystem.set_servo_positions();
                                            Wobble.wobble_grip();
                                            Subsystem.set_servo_positions();
                                        })
                                        .splineToLinearHeading(new Pose2d(0, -15), Math.toRadians(-90))
                                        .build();

                                navigate.followTrajectory(none);
                                state = States.STOP;
                                break;

                            case ONE_STACK:
                                telemetry.addData("state", "One Stack");
                                telemetry.addData("pos", navigate.getWheelPositions());
                                telemetry.update();
                                Trajectory oneStack = navigate.trajectoryBuilder(new Pose2d())
                                        .splineToLinearHeading(new Pose2d(-64, -10), Math.toRadians(-90))
                                        .addDisplacementMarker(() -> {
                                            Wobble.wobble_arm_down();
                                            Subsystem.set_servo_positions();
                                            Wobble.wobble_ungrip();
                                            Subsystem.set_servo_positions();
                                            Wobble.wobble_arm_up();
                                            Subsystem.set_servo_positions();
                                        })
                                        .build();

                                navigate.followTrajectory(oneStack);
                                state = States.STOP;
                                break;

                            case THREE_STACK:
                                telemetry.addData("state", "One Stack");
                                telemetry.addData("pos", navigate.getWheelPositions());
                                telemetry.update();
                                Trajectory threeStack = navigate.trajectoryBuilder(new Pose2d(0,0))
                                        .splineToLinearHeading(new Pose2d(-69, -10), Math.toRadians(-90))
                                        .addDisplacementMarker(() -> {
                                            Wobble.wobble_arm_down();
                                            Subsystem.set_servo_positions();
                                            Wobble.wobble_ungrip();
                                            Subsystem.set_servo_positions();
                                            Wobble.wobble_arm_up();
                                            Subsystem.set_servo_positions();
                                        })
                                        .build();

                                navigate.followTrajectory(threeStack);
                                state = States.STOP;
                                break;

                            case STOP:
                                break;
                        }
                    }
                }

                telemetry.update();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = Subsystem.robot.webcam;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}