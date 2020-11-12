package org.firstinspires.ftc.PinkCode.OpModes;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.pedro.rtplibrary.rtmp.RtmpCamera1;
import com.pedro.rtplibrary.rtsp.RtspCamera1;
import com.pedro.rtsp.rtsp.RtspClient;
import com.pedro.rtsp.utils.ConnectCheckerRtsp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import net.ossrs.rtmp.ConnectCheckerRtmp;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Conveyor;
import org.firstinspires.ftc.PinkCode.Subsystems.Shooter;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigate;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.lang.reflect.Array;
import java.util.List;

@TeleOp(name = "TensorFlow Auto Webcam", group = "Auto")
public class TensorFlow extends OpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private PinkNavigate navigate;

    private enum States {
        INIT,
        DRIVE_TO_TARGETS,
        GO_TO_HOME_POSITION,
        FIND_TARGETS,
        SHOOT,
        STOP
    };

    public abstract static class Config {
        /*
        *    On Auto INIT best to grab Wobble Goal and place it across the wall
        *    then grab the start stack then shoot at the line then park at the line
        */

        static boolean SHOOT = false;
        static boolean DRIVE_TO_DONUTS = false;
        static boolean START_CONVEYOR_ON_INIT = false;
        static boolean GET_WOBBLE_GOAL = false;
        static boolean GET_STARTER_STACK = false;
        static boolean GO_FOR_LINE = false;
    };

    private enum ALLIANCE {
        RED,
        BLUE,
    };

    private States current_state = States.DRIVE_TO_TARGETS;
    private Trajectory drive_init;
    private Pose2d init_pose;
    private Pose2d drive_to_targets;
    private Pose2d home_pose;
    private Vector2d pos_from_home = new Vector2d(0, 0);

    private Trajectory drive_targets;
    private Pose2d targets_pose;

    private ALLIANCE CurrentAlliance = ALLIANCE.RED;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void init() {
        navigate = new PinkNavigate(hardwareMap);

        //imu initialization
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);



        // Initialization of Each Subsystem's Hardware Map
        Subsystem.robot.init(hardwareMap);

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            telemetry.addData("TFOD", "Activated");
        }

        if (gamepad1.dpad_left) {
            CurrentAlliance = ALLIANCE.BLUE;
        } else if (gamepad1.dpad_right) {
            CurrentAlliance = ALLIANCE.RED;
        }

//        navigate = new PinkNavigate(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");

        tfod.activate();
    }

    @Override
    public void loop() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                for (Recognition r: updatedRecognitions) {
                    telemetry.addData("obj", r.getLabel());
                }

                RobotFunctions(updatedRecognitions);
            } else {
                tfod.activate();
                telemetry.addData("TFOD", "activating");
            }
        } else {
            telemetry.addData("TFOD", "is null");
            tfod.activate();
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void RobotFunctions(List<Recognition> updatedRecognitions) {
        switch (current_state) {
            case INIT:
                telemetry.addData("Status", "Init");

                // init
//                Wobble.wobble_grip();
//                Wobble.wobble_arm_up();

//                Trajectory left = navigate.trajectoryBuilder(new Pose2d(0,0))
//                        .lineTo(new Vector2d(0.5, 0))
//                        .addDisplacementMarker(() -> {
//                            Trajectory forward = navigate.trajectoryBuilder(new Pose2d(-0.5, 0))
//                                    .forward(1)
//                                    .build();
//
//                            navigate.followTrajectory(forward);
//                        })
//                        .build();
//
//                navigate.followTrajectory(left);

//                init_pose = new Pose2d(0.5, 1);
                current_state = States.STOP;
                break;

            case DRIVE_TO_TARGETS:
                if (updatedRecognitions.isEmpty()) {
                    current_state = States.FIND_TARGETS;
                    return;
                }

                telemetry.addData("Status", "Driving to Targets");
                double pos = GetObjectPosition(updatedRecognitions);

                telemetry.addData("pos", pos);
                

//                drive_targets = navigate.trajectoryBuilder(new Pose2d(0, 0))
//                        .forward(pos)
//                        .addDisplacementMarker(() -> {
//                            Trajectory center = navigate.trajectoryBuilder(new Pose2d(0, pos))
//                                    .splineTo(new Vector2d(3, 3), Math.toRadians(0))
//                                    .addDisplacementMarker(() -> {
//                                        Collector.collect_stop();
//                                        Conveyor.conveyor_stop();
//                                    })
//                                    .build();
//
//                            navigate.followTrajectory(center);
//                        })
//                        .build();

                if (Config.START_CONVEYOR_ON_INIT) {
                    Conveyor.collect();
                }

                if (Config.DRIVE_TO_DONUTS) {
                    telemetry.addData("Status", "Going to Donut");
                    
//                    navigate.followTrajectory(drive_targets);
                }

                current_state = States.STOP;
                return;

            case FIND_TARGETS:
                telemetry.addData("Status", "Searching For Object");
                telemetry.addData("updatedRecognitions", updatedRecognitions.toArray());
                
//                Subsystem.robot.leftB_drive.setPower(-0.3);
//                Subsystem.robot.leftF_drive.setPower(-0.3);
//                Subsystem.robot.rightF_drive.setPower(0.3);
//                Subsystem.robot.rightB_drive.setPower(0.3);

                if (!updatedRecognitions.isEmpty()) {
                    current_state = States.DRIVE_TO_TARGETS;
                }
                break;

            case STOP:
                break;
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = Subsystem.robot.webcam;


        ClassFactory.getInstance().getCameraManager().nameForUnknownCamera();

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void AddToHomePosition(double x, double y) {
        pos_from_home.plus(new Vector2d(x, y));
    }

    private double GetObjectPosition(List<Recognition> updatedRecognitions) {
        // step through the list of recognitions and display boundary info.
        float pos = 0.0f;
        for (Recognition recognition : updatedRecognitions) {
            pos = recognition.getLeft() - Presets.CAMERA_TO_INTAKE;
        }

        telemetry.addData("Obj Pos", pos);
        telemetry.update();

        return Presets.encoderTicksToInches(pos);
    }

    private double GetObjectAngle(List<Recognition> updatedRecognitions) {
        double angle = 0.0;

        for (Recognition recognition : updatedRecognitions) {
            angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
        }

        
        return angle;
    }

    private double GetTowerAngle(List<Recognition> updatedRecognitions) {
        float pos = 0.0f;

        for (Recognition r: updatedRecognitions) {
            if (r.getLabel().equals("t")) {
                pos = r.getLeft();
            }
        }

        telemetry.addData("Tower Pos", pos);
        telemetry.update();

        return Presets.encoderTicksToInches(pos);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
