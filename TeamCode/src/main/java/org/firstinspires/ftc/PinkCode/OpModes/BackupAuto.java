package org.firstinspires.ftc.PinkCode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.odometry.PinkNavigate;

@Autonomous(name = "Scrimmage Auto Two", group = "Auto")
public class BackupAuto extends OpMode {
    private PinkNavigate navigate;
    private States state = States.MOVE;
    private enum States {
        MOVE,
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
            case MOVE:
                Trajectory moveToLine = navigate.trajectoryBuilder(new Pose2d(0, 0))
                        .lineTo(new Vector2d(1,0))
                        .addDisplacementMarker(() -> {
                            Trajectory move = navigate.trajectoryBuilder(new Pose2d(1, 0))
                                    .forward(1)
                                    .build();

                            navigate.followTrajectory(move);
                        })
                        .build();

                navigate.followTrajectory(moveToLine);
                state = States.STOP;
                break;

            case STOP:
                break;
        }
    }
}
