package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigate;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.VuforiaWebcam;

import static org.firstinspires.ftc.PinkCode.odometry.OdemetryPresets.drive_from_stack;
import static org.firstinspires.ftc.PinkCode.odometry.OdemetryPresets.drive_init;

@Autonomous(name = "Auto", group = "Auto")
public class Auto extends OpMode {
    private VuforiaWebcam webcam;
    private VuforiaLocalizer.Parameters parameters;
    private PinkNavigate drive;

    private enum stages {
        INIT,
        DRIVE_TEST_TWO,
        STOP,
    }

    private Pose2d poseOne = new Pose2d(0, 0);
    private Pose2d poseTwo = new Pose2d(0, 0);
    private Pose2d poseThree = new Pose2d(0, 0);
    private Pose2d poseFour = new Pose2d(0, 0);
    private Pose2d poseFive = new Pose2d(0, 0);

    private stages stage = stages.INIT;

    @Override
    public void init() {
        // Initialization of Each Subsystem's Hardware Map
        Subsystem.robot.init(hardwareMap);
        // Possible That Run Without Encoder is affecting the encoder counts
//        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new PinkNavigate(hardwareMap);

        // change webcam from phone to webcam
        // Telemetry Update to Inform Drivers That the Program is Initialized
        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.update();

        telemetry.addData("Power One", drive.GetMotorPowerOne());
        telemetry.addData("Power Two", drive.GetMotorPowerTwo());
        telemetry.addData("Power Three", drive.GetMotorPowerThree());
        telemetry.update();

        switch (stage) {
            case INIT:
//                drive_init = drive.trajectoryBuilder(new Pose2d())
//                        .lineTo(new Vector2d( -3.5, 2))
//                        .addDisplacementMarker(() -> drive.followTrajectoryAsync(drive_from_stack))
//
//                        .build();

                drive_from_stack = drive.trajectoryBuilder(new Pose2d())
                        .lineTo(new Vector2d(0, 1.5))
                        .build();

                // Put Shooter Code Here

                telemetry.addData("pos", drive_from_stack.end());
                telemetry.update();

//                telemetry.addData("pos", drive_init.end());
//                telemetry.update();

                drive.followTrajectory(drive_from_stack);
                stage = stages.STOP;
                break;

            case STOP:
                telemetry.addData("status", "stop");
                telemetry.update();
                break;
        }
    }

    public String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
