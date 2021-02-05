package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

class TrajectoryConstants {
    static final double RED_CORNER_START_X = -62.5;
    static final double RED_CORNER_START_Y = -48.8;
    static final double RED_CORNER_START_HEADING = 180; // Backward

    static final double MOVE_TO_SHOOTING_X = 1.0;
    static final double MOVE_TO_SHOOTING_Y = 15.0;
    static final double MOVE_TO_SHOOTING_HEADING = 0.0;

    enum POWERSHOTS {
        FIRST,
        SECOND,
        THIRD
    }

    static POWERSHOTS DEFAULT_POWERSHOT = POWERSHOTS.FIRST;
}

@Autonomous(name = "Powershot Auto", group = "Auto")
public class PinkPowershot extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "pink.tflite";
    private static final String LABEL_FIRST_ELEMENT = "powershot";
    private static final String LABEL_SECOND_ELEMENT = "obj";

    private static ElapsedTime time;

    private enum Stage {
        MoveToShootingRange,
        PrimeShooter,
        ShootOne,
        ShootTwo,
        ShootThree,
        Stop,
    }
    private Stage stage = Stage.MoveToShootingRange;

    private SampleMecanumDrive drive;
    private TFObjectDetector tf;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(TrajectoryConstants.RED_CORNER_START_X, TrajectoryConstants.RED_CORNER_START_Y, TrajectoryConstants.RED_CORNER_START_HEADING));

        initVuforia();
        initTfod(FtcDashboard.getInstance());

        if (tf != null) {
            tf.activate();
            telemetry.addData("tf", "ready");
            telemetry.update();
        }

        waitForStart();

        time = new ElapsedTime();

        time.startTime();

        while (opModeIsActive()) {
            switch (stage) {
                case MoveToShootingRange:
                    Trajectory t = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(TrajectoryConstants.MOVE_TO_SHOOTING_X, TrajectoryConstants.MOVE_TO_SHOOTING_Y), Math.toRadians(TrajectoryConstants.MOVE_TO_SHOOTING_HEADING))
                            .build();

                    drive.followTrajectory(t);
                    stage = Stage.ShootOne;
                    break;

                case ShootOne:
                    double waitTime = time.seconds() + 2;
                    TrajectoryConstants.DEFAULT_POWERSHOT = TrajectoryConstants.POWERSHOTS.FIRST;

                    double angleOne = GetAngle();

                    if (time.seconds() < waitTime) {
                        AimShoot(angleOne);
                        stage = Stage.Stop;
                    }
                    break;

                case ShootTwo:
                    TrajectoryConstants.DEFAULT_POWERSHOT = TrajectoryConstants.POWERSHOTS.SECOND;

                    double angleTwo = GetAngle();

                    AimShoot(angleTwo);
                    break;

                case ShootThree:
                    TrajectoryConstants.DEFAULT_POWERSHOT = TrajectoryConstants.POWERSHOTS.THIRD;

                    double angleThree = GetAngle();

                    AimShoot(angleThree);
                    break;

                case Stop:
                    break;
            }
        }
    }

    private void AimShoot(double angle) {
        Trajectory r = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 0), Math.toRadians(angle))
                .build();

        drive.followTrajectory(r);
    }

    private double GetAngle() {
        double angle = 0.0;
        int index = 0;

        if (TrajectoryConstants.DEFAULT_POWERSHOT == TrajectoryConstants.POWERSHOTS.SECOND) {
            index = 1;
        } else if (TrajectoryConstants.DEFAULT_POWERSHOT == TrajectoryConstants.POWERSHOTS.THIRD) {
            index = 2;
        } else {
            index = 0;
        }

        if (tf != null) {
            List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (int i = 0; i != updatedRecognitions.toArray().length; i++) {
                    Recognition r = updatedRecognitions.get(i);

                    if (i == index) {
                        angle = r.estimateAngleToObject(AngleUnit.RADIANS);

                        telemetry.addData("index", i);
                        telemetry.addData("angle", angle);
                        telemetry.addData("label", r.getLabel());
                        telemetry.update();
                    }
                }
            }
        }

        return angle;
    }

    private void initTfod(FtcDashboard dash) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        dash.startCameraStream(tf, 60);
        tfodParameters.minResultConfidence = 0.7f;
        tf = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tf.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = PinkSubsystem.robot.webcam;


        ClassFactory.getInstance().getCameraManager().nameForUnknownCamera();

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
}
