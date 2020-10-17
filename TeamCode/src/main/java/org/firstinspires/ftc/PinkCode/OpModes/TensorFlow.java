package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Conveyor;
import org.firstinspires.ftc.PinkCode.Subsystems.Shooter;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigate;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "TensorFlow Object Detection Webcam", group = "Concept")
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

    private enum ALLIANCE {
        RED,
        BLUE,
    };
    private States current_state = States.INIT;
    private Trajectory drive_init;
    private Pose2d init_pose;

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

        Subsystem.set_servo_positions();

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).
        }

        if (gamepad1.dpad_left) {
            CurrentAlliance = ALLIANCE.BLUE;
        } else if (gamepad1.dpad_right) {
            CurrentAlliance = ALLIANCE.RED;
        }

//        navigate = new PinkNavigate(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (CurrentAlliance == ALLIANCE.RED && updatedRecognitions != null) {
                switch (current_state) {
                    case INIT:
                        telemetry.addData("Status", "Running");
                        telemetry.update();
                        drive_init = navigate.trajectoryBuilder(new Pose2d())
                                .lineTo(new Vector2d(1.5, 0))
                                .lineTo(new Vector2d(0, 2.5))
                                .splineTo(new Vector2d(-1.8, 1.5), Math.toRadians(-30))
                                .build();

                        init_pose = drive_init.end();
                        current_state = States.DRIVE_TO_TARGETS;
                        break;

                    case DRIVE_TO_TARGETS:
                        if (updatedRecognitions.isEmpty()) {
                            current_state = States.FIND_TARGETS;
                            return;
                        }

                        telemetry.addData("Status", "Running");
                        telemetry.update();
                        double pos = GetObjectPosition(updatedRecognitions);

                        Conveyor.collect();
                        Collector.collect();

                        drive_targets = navigate.trajectoryBuilder(init_pose)
                                .forward(pos)
                                .addSpatialMarker(new Vector2d(0, pos), () -> {
                                    Trajectory center = navigate.trajectoryBuilder(new Pose2d(0, pos))
                                            .splineTo(new Vector2d(3, 3), Math.toRadians(0))
                                            .build();

                                    navigate.followTrajectory(center);
                                })
                                .build();

                        navigate.followTrajectory(drive_targets);
                        break;

                    case FIND_TARGETS:
                        telemetry.addData("Status", "Searching For Object");
                        telemetry.update();
                        Subsystem.robot.leftB_drive.setPower(-0.1);
                        Subsystem.robot.leftF_drive.setPower(-0.1);
                        Subsystem.robot.rightF_drive.setPower(0.1);
                        Subsystem.robot.rightB_drive.setPower(0.1);

                        if (!updatedRecognitions.isEmpty()) {
                            current_state = States.DRIVE_TO_TARGETS;
                        }
                        break;

                    case STOP:
                        break;
                }
            } else if (updatedRecognitions != null && CurrentAlliance == ALLIANCE.BLUE) {
                switch (current_state) {
                    case INIT:
                        if (updatedRecognitions.isEmpty()) {
                            current_state = States.FIND_TARGETS;
                            return;
                        }

                        telemetry.addData("Status", "Running");
                        telemetry.update();
                        drive_init = navigate.trajectoryBuilder(new Pose2d())
                                .forward(0.5)
                                .build();

                        init_pose = drive_init.end();
                        current_state = States.DRIVE_TO_TARGETS;
                        break;

                    case DRIVE_TO_TARGETS:
                        telemetry.addData("Status", "Running");
                        telemetry.update();
                        double pos = GetObjectPosition(updatedRecognitions);

                        drive_targets = navigate.trajectoryBuilder(init_pose)
                                .forward(pos)
                                .build();
                        break;


                    case FIND_TARGETS:
                        telemetry.addData("Status", "Searching For Object");
                        telemetry.update();
                        Subsystem.robot.leftB_drive.setPower(-0.1);
                        Subsystem.robot.leftF_drive.setPower(-0.1);
                        Subsystem.robot.rightF_drive.setPower(0.1);
                        Subsystem.robot.rightB_drive.setPower(0.1);

                        if (!updatedRecognitions.isEmpty()) {
                            current_state = States.DRIVE_TO_TARGETS;
                        }
                        break;

                    case STOP:
                        break;
                }
            }
        }
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

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private double GetObjectPosition(List<Recognition> updatedRecognitions) {
        // step through the list of recognitions and display boundary info.
        float pos = 0.0f;
        for (Recognition recognition : updatedRecognitions) {
            pos = recognition.getLeft();
        }
        return Presets.encoderTicksToInches(pos);
    }

    ;

    private double GetObjectAngle(List<Recognition> updatedRecognitions) {
        double angle = 0.0;

        for (Recognition recognition : updatedRecognitions) {
            angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
        }

        telemetry.update();
        return angle;
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
