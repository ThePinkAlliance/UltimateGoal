package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Conveyor;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "TensorFlow Webcam", group = "Auto")
//@Disabled
public class TensorFlow extends OpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

//    private enum States {
//        INIT,
//        DRIVE_TO_TARGETS,
//        GO_TO_HOME_POSITION,
//        FIND_TARGETS,
//        SHOOT,
//        STOP
//    };
//
//    public abstract static class Config {
//        /*
//        *    On Auto INIT best to grab Wobble Goal and place it across the wall
//        *    then grab the start stack then shoot at the line then park at the line
//        */
//
//        static boolean SHOOT = false;
//        static boolean DRIVE_TO_DONUTS = false;
//        static boolean START_CONVEYOR_ON_INIT = false;
//        static boolean GET_WOBBLE_GOAL = false;
//        static boolean GET_STARTER_STACK = false;
//        static boolean GO_FOR_LINE = false;
//    };
//
//    private enum ALLIANCE {
//        RED,
//        BLUE,
//    };

//    private States current_state = States.DRIVE_TO_TARGETS;
//    private Trajectory drive_init;
//    private Pose2d init_pose;
//    private Pose2d drive_to_targets;
//    private Pose2d home_pose;
//    private Vector2d pos_from_home = new Vector2d(0, 0);

//    private Trajectory drive_targets;
//    private Pose2d targets_pose;

//    private ALLIANCE CurrentAlliance = ALLIANCE.RED;

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
    private double zoom = 1.0, aspect = 4.3;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void init() {

        //imu initialization
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//         Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//         on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//         and named "imu".
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
            tfod.setZoom(1.0, 4.3);
            telemetry.addData("TFOD", "Activated");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
    }

    @Override
    public void loop() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                for (Recognition r: updatedRecognitions) {
                    telemetry.addData("obj", r.getLabel());
                    telemetry.addData("amount", updatedRecognitions.size());
                }

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

//    private void AddToHomePosition(double x, double y) {
//        pos_from_home.plus(new Vector2d(x, y));
//    }

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
