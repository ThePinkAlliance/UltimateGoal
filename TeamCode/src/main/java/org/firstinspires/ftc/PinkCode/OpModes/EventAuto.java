package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigate;

@Autonomous(name = "Scrimmage Auto One", group = "Auto")
public class EventAuto extends OpMode {
    private PinkNavigate navigate;
    private States state = States.INIT;
    private enum States {
        INIT,
        GRIP_GOAL,
        LIFT_GOAL,
        DROP_GOAL,
        PARK_AT_LINE,
        STOP
    };

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
        switch (state) {
            case INIT:
                Wobble.wobble_arm_down();
                telemetry.addData("State", "Arm Down");

                state = States.GRIP_GOAL;
                break;

            case GRIP_GOAL:
                Wobble.wobble_grip();
                telemetry.addData("State", "Wobble Grip");

                state = States.LIFT_GOAL;
                break;

            case LIFT_GOAL:
                Wobble.wobble_arm_up();
                telemetry.addData("State", "Arm Up");

                state = States.DROP_GOAL;
                break;

            case DROP_GOAL:
                Trajectory moveToBox = navigate.trajectoryBuilder(new Pose2d(0, 0))
                        .forward(1)
                        .addDisplacementMarker(() -> {
                            Trajectory rotate = navigate.trajectoryBuilder(new Pose2d(0, 1))
                                    .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-90)))
                                    .addDisplacementMarker(() -> {
                                        Trajectory rotateAgain = navigate.trajectoryBuilder(new Pose2d(0, 1, Math.toRadians(-90)))
                                                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-90)))
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

                state = States.PARK_AT_LINE;
                break;

            case PARK_AT_LINE:
                Trajectory moveToLine = navigate.trajectoryBuilder(new Pose2d(0,1, Math.toRadians(180)))
                        .forward(0.5)
                        .build();

                telemetry.addData("State", "Moving To Line");
                navigate.followTrajectory(moveToLine);

                state = States.STOP;
                break;

            case STOP:
                break;
        }
        telemetry.update();
    }
}
