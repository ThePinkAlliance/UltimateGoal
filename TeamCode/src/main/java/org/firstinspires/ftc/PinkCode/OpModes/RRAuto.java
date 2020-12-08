package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class RRAuto extends OpMode {
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
        GO_TO_HOME_POSITION,
        FIND_TARGETS,
        RUN,
        STOP
    }

    private States states = States.FIND_TARGETS;

    private Trajectory trajectory;
    private Trajectory trajectory1;
    private SampleMecanumDrive drive;

    @Override
    public void init() {
        Subsystem.robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("robot", "Press Play To Start");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                switch (states) {
                    case FIND_TARGETS:
                        numRings = updatedRecognitions.size();

                        telemetry.addData("number of rings", numRings);
                        telemetry.addData("updatedRecognitions", updatedRecognitions.size());
                        telemetry.update();

                        if (numRings == 4) {
                            ANGLE = -20;
                            DISTANCE = -102;
                            CenteringAngle = 198;
                            CenteringDistance = 49;
                        } else if (numRings == 3) {
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

                        trajectory = drive.trajectoryBuilder(new Pose2d())
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

                        trajectory1 = drive.trajectoryBuilder(new Pose2d())
                                .lineToLinearHeading(new Pose2d(CenteringDistance, -20, Math.toRadians(CenteringAngle)))
                                .addTemporalMarker(7.5, () -> {
                                    Subsystem.robot.conveyor.setPower(1);
                                })
                                .build();

                        states = States.RUN;
                        break;

                    case RUN:
                        drive.followTrajectory(trajectory);
                        drive.followTrajectory(trajectory1);

                        states = States.GO_TO_HOME_POSITION;
                        break;

                    case GO_TO_HOME_POSITION:
                        Pose2d poseEstimate = drive.getPoseEstimate();
                        telemetry.addData("finalX", poseEstimate.getX());
                        telemetry.addData("finalY", poseEstimate.getY());
                        telemetry.addData("finalHeading", poseEstimate.getHeading());
                        telemetry.update();

                        states = States.STOP;
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
            tfod.deactivate();
        }
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

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
}