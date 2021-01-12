package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Collector;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Wobble;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@Autonomous(name = "PinkTestAuto", group = "Auto")
public class PinkTestAuto extends LinearOpMode {

    public static double DISTANCE = 24;
    public static SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // This is the starting position during auto
        // drive.setPoseEstimate(new Pose2d(-62.5, -48.8, Math.toRadians(180)));

        /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-62.5, -48.8, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-25.0, -56.0, Math.toRadians(160)))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(-25.0, -56.0, Math.toRadians(160)))
                .lineToLinearHeading(new Pose2d(0.0, -52.0, Math.toRadians(110)))
                .build(); */

        /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-62.5, -48.8, Math.toRadians(180)))
                .splineTo(new Vector2d(0.0, -52.0), Math.toRadians(110))
                .build(); */

        // Set all servos to their default position
        Collector.collector_drop();
        Wobble.wobble_grip();
        Wobble.wobble_arm_up();
        Conveyor.flap_close();

        PinkSubsystem.set_servo_positions();

        /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-62.5, -48.8, Math.toRadians(180)), Math.toRadians(-10.0))
                .splineToLinearHeading(new Pose2d(6.0, -54.0, Math.toRadians(110)), Math.toRadians(10))
                .addTemporalMarker(3.3, () -> {
                    // This marker runs after the first splineTo()
                    Wobble.wobble_arm_down();
                    PinkSubsystem.set_servo_positions();
                })
                .addTemporalMarker(3.8, () -> {
                    // This marker runs after the first splineTo()
                    Wobble.wobble_ungrip();
                    PinkSubsystem.set_servo_positions();
                })
                .build();

        /*Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(6.0, -54.0, Math.toRadians(110)))
                .lineToLinearHeading(new Pose2d(-2, -36, Math.toRadians(-3)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d(-2, -36, Math.toRadians(-3)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-38, -21, Math.toRadians(3)), Math.toRadians(180))
                .addTemporalMarker(3.25, () -> {
                    // This marker runs after the first splineTo()
                    Wobble.wobble_grip();
                    PinkSubsystem.set_servo_positions();
                })
                .build();

  Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-38, -21, Math.toRadians(0)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(6.0, -54.0, Math.toRadians(110)), Math.toRadians(10))
                .addTemporalMarker(3.3, () -> {
                    // This marker runs after the first splineTo()
                    Wobble.wobble_arm_down();
                    PinkSubsystem.set_servo_positions();
                })
                .addTemporalMarker(3.8, () -> {
                    // This marker runs after the first splineTo()
                    Wobble.wobble_ungrip();
                    PinkSubsystem.set_servo_positions();
                })
                .build();

*/
        waitForStart();

        /*drive.followTrajectory(trajectory);
        drive.followTrajectory(trajectory2);
        drive.followTrajectory(trajectory3);
        drive.followTrajectory(trajectory4);
*/
        Conveyor.flap_close();
        PinkSubsystem.set_servo_positions();

        Collector.collect();
        Conveyor.collect(1);
        PinkSubsystem.set_motor_powers();

        sleep(250);
        drive.setPoseEstimate(new Pose2d(-38, -21, Math.toRadians(0)));
        drive.turn(Math.toRadians(-45));

        Trajectory trajectory = BuildSimpleTrajectory(-38, -21, -45, -75,
                -2, -36, 0, 10);

        drive.followTrajectory(trajectory);

        while (opModeIsActive() && !isStopRequested()) {





            /*

            Conveyor.collect(0.7);
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), 1550);

            PinkSubsystem.set_motor_powers();

            double currentShooterVelocity = PinkSubsystem.robot.shoot2.getVelocity();
            if(currentShooterVelocity > 1450 && currentShooterVelocity < 1620)
            {
                Conveyor.flap_open();
                PinkSubsystem.set_servo_positions();
            }
*/

        }


    }

    // Generates a quick and dirty trajectory based on a begin and an end position
    public static Trajectory BuildSimpleTrajectory(double beginX, double beginY, double beginHeadingAngle,  double beginTangentAngle,
                                                   double endX, double endY, double endHeadingAngle,  double endTangentAngle)
    {
        Trajectory newTractory =
                drive.trajectoryBuilder(new Pose2d(beginX, beginY, Math.toRadians(beginHeadingAngle)), Math.toRadians(beginTangentAngle))
                        .splineToLinearHeading(new Pose2d(endX, endY, Math.toRadians(endHeadingAngle)), Math.toRadians(endTangentAngle))
                        .build();

        return newTractory;
    }
}