package org.firstinspires.ftc.PinkCode.OpModes;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigate;

import java.util.Timer;
import java.util.jar.Manifest;

@Autonomous(name = "Scrimmage Auto One", group = "Auto")
public class EventAuto extends OpMode {
    private PinkNavigate navigate;
    private States state = States.INIT;
    private boolean init = true;
    private ElapsedTime elapsedTime;

    private enum States {
        INIT,
        GRIP_GOAL,
        LIFT_GOAL,
        DROP_GOAL,
        PARK_AT_LINE,
        STOP,
        MAIN
    };

    public abstract static class Config {
        static Pose2d MoveToLine = new Pose2d(1, 0, Math.toRadians(-90));
        static Vector2d MoveToDrop = new Vector2d(0.7, 0);
        static Pose2d MoveBackToLine = new Pose2d(-1, 0, Math.toRadians(90));
    }

    @Override
    public void init() {
        Subsystem.robot.init(hardwareMap);
        navigate = new PinkNavigate(hardwareMap);
        // Possible That Run Without Encoder is affecting the encoder counts
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Telemetry Update to Inform Drivers That the Program is Initialized
        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (init) {
            elapsedTime.startTime();
            init = false;
        }

        switch (state) {
            case INIT:
                Wobble.wobble_arm_down();
                telemetry.addData("State", "Arm Down");

                state = States.GRIP_GOAL;
                break;

            case GRIP_GOAL:
                if (elapsedTime.seconds() > 2) return;

                Wobble.wobble_grip();
                telemetry.addData("State", "Wobble Grip");

                state = States.LIFT_GOAL;
                break;

            case LIFT_GOAL:
                if (elapsedTime.seconds() < 4) return;

                Wobble.wobble_arm_up();
                telemetry.addData("State", "Arm Up");

                state = States.DROP_GOAL;
                break;

            case MAIN:
                Trajectory moveToBox = navigate.trajectoryBuilder(new Pose2d(0, 0))
                        .lineToLinearHeading(Config.MoveToLine)
                        .addDisplacementMarker(() -> {
                            Trajectory rotate = navigate.trajectoryBuilder(new Pose2d(1, 0, Math.toRadians(-90)))
                                    .lineTo(Config.MoveToDrop)
                                    .addDisplacementMarker(() -> {
                                        Trajectory rotateAgain = navigate.trajectoryBuilder(new Pose2d(0.7, 0, Math.toRadians(-90)))
                                                .lineToLinearHeading(Config.MoveBackToLine)
                                                .build();

                                        navigate.followTrajectory(rotateAgain);
                                    })
                                    .build();

                            navigate.followTrajectory(rotate);
                            telemetry.addData("State", "Rotating");
                        })
                        .build();

                navigate.followTrajectory(moveToBox);
                telemetry.addData("State", "Moving to Wobble Goal Drop");

                state = States.STOP;
                break;

            case STOP:
                break;
        }
        telemetry.update();
    }
}
