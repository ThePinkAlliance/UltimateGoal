package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

import android.view.animation.LinearInterpolator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.Presets;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Collector;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Wobble;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Config
@Autonomous(name = "PinkAutoStrafe", group = "Auto")
public class PinkAutoStrafe extends LinearOpMode {

    // Ring Mode - will be used for trajectory and positioning
    public enum RingMode { NONE, SINGLE, QUAD }
    public enum StartingPosition { SP_CORNER_RED, SP_MIDDLE_RED, SP_CORNER_BLUE, SP_MIDDLE_BLUE }
    public enum AutonomousSTEPS {
        AS_DROP_FIRST_WOBBLE_DRIVE,
        AS_DROP_FIRST_WOBBLE_DROP,
        AS_DROP_FIRST_WOBBLE_RELEASE,
        AS_SHOOT_FIRST_3_HIGH_DRIVE,
        AS_SHOOT_FIRST_3_SHOOT,
        AS_COLLECT_CENTER_WOBBLE,
        AS_COLLECT_CENTER_GRIP,
        AS_COLLECT_CENTER_LIFT,
        AS_COLLECT_SINGLE_RING_DRIVE,
        AS_COLLECT_QUAD_RING_STRAFE,
        AS_COLLECT_QUAD_RING_DRIVE,
        AS_COLLECT_LAST_QUAD_STRAIGHT_DRIVE,
        AS_COLLECT_LAST_QUAD_STRAIGHT_SHOOT,
        AS_SHOOT_SINGLE_FROM_STACK,
        AS_DROP_SECOND_WOBBLE_DRIVE,
        AS_DROP_SECOND_WOBBLE_DROP,
        AS_DROP_SECOND_WOBBLE_RELEASE,
        AS_QUAD_COLLECT_LAST_DRIVE,
        AS_QUAD_LAST_SHOOT_DRIVE,
        AS_QUAD_LAST_SHOOT,
        AS_PARK,
        AS_AUTONOMOUS_END
    }

    double HIGH_SHOT_SPINDEXER_POWER  = 1.00;


    // Starting Ring mode... None, will be adjusted later
    public static RingMode ringFound = RingMode.NONE;
    public static StartingPosition startingFieldPosition = StartingPosition.SP_CORNER_RED;
    public static AutonomousSTEPS autoStep = AutonomousSTEPS.AS_DROP_FIRST_WOBBLE_DRIVE;

    // Field Start Position Corner
    public static double RED_CORNER_START_X = -62.5;
    public static double RED_CORNER_START_Y = -48.8;
    public static double RED_CORNER_START_HEADING = 180; // Backward
    public static double RED_CORNER_START_TAN_START = -10;

    // RC0 = Red Corner 0 Ring... RC1, RCQ = Red Corner Quad Ring
    public static double RC0_DROP_FIRST_WOB_X = 5;
    public static double RC0_DROP_FIRST_WOB_Y = -54;
    public static double RC0_DROP_FIRST_WOB_HEADING = 110;
    public static double RC0_DROP_FIRST_WOB_TAN_END = 10; // Spline Tangent where this segment end
    public static double RC0_DROP_FIRST_WOB_TAN_BEGIN = 75; // Spline Tangent where this segment begins o the next position

    public static double RC1_DROP_FIRST_WOB_X = 25;
    public static double RC1_DROP_FIRST_WOB_Y = -34;
    public static double RC1_DROP_FIRST_WOB_HEADING = 145;
    public static double RC1_DROP_FIRST_WOB_TAN_END = 45; // Spline Tangent where this segment end
    public static double RC1_DROP_FIRST_WOB_TAN_BEGIN = 190; // Spline Tangent where this segment begins o the next position

    public static double RCQ_DROP_FIRST_WOB_X = 50;
    public static double RCQ_DROP_FIRST_WOB_Y = -54;
    public static double RCQ_DROP_FIRST_WOB_HEADING = 120;
    public static double RCQ_DROP_FIRST_WOB_TAN_END = 0; // Spline Tangent where this segment end
    public static double RCQ_DROP_FIRST_WOB_TAN_BEGIN = 190; // Spline Tangent where this segment begins o the next position

    public static double RC_SHOOT_HIGH_WOB_X = -5;
    public static double RC_SHOOT_HIGH_WOB_Y = -36;
    public static double RC_SHOOT_HIGH_WOB_HEADING = -1;
    public static double RCQ_SHOOT_HIGH_WOB_HEADING = 0;

    public static double RC0_SHOOT_HIGH_WOB_TAN_END = 15;
    public static double RC1_SHOOT_HIGH_WOB_TAN_END = 180; // Different Tangent for second pos but same x/y/heading
    public static double RCQ_SHOOT_HIGH_WOB_TAN_END = 180;

    public static double RC_SHOOT_HIGH_WOB_TAN_BEGIN = 80;
    public static double RCQ_SHOOT_HIGH_WOB_TAN_BEGIN = 100;

    public static double RC_COLLECT_MID_WOB_X = -34.85;
    public static double RC_COLLECT_MID_WOB_Y = -23.5;
    public static double RC_COLLECT_MID_WOB_HEADING = 0;
    public static double RC_COLLECT_MID_WOB_TAN_END = 200;

    public static double RCQ_COLLECT_MID_WOB_X = -36.5;
    public static double RCQ_COLLECT_MID_WOB_Y = -26.5;
    public static double RCQ_COLLECT_MID_WOB_HEADING = 3;
    public static double RCQ_COLLECT_MID_WOB_TAN_END = 210;

    public static double RC0_COLLECT_MID_WOB_TAN_BEGIN = 25;
    public static double RC1_COLLECT_MID_WOB_TAN_BEGIN = -75; // Different Tangent to next position since now going to collect single ring
    public static double RCQ_COLLECT_MID_WOB_TAN_BEGIN = -75;

    public static double RC1_SHOOT_SINGLE_STACK_X = -4;//RC_SHOOT_HIGH_WOB_X;
    public static double RC1_SHOOT_SINGLE_STACK_Y = RC_SHOOT_HIGH_WOB_Y;
    public static double RC1_SHOOT_SINGLE_STACK_HEADING = -5;
    public static double RC1_SHOOT_SINGLE_STACK_TAN_END = 25;
    public static double RC1_SHOOT_SINGLE_STACK_TAN_BEGIN = 0;

    public static double RCQ_STRAFE_RIGHT_X = RCQ_COLLECT_MID_WOB_X -2;
    public static double RCQ_STRAFE_RIGHT_Y = RCQ_COLLECT_MID_WOB_Y - 13;
    public static double RCQ_STRAFE_RIGHT_HEADING = 2;
    public static double RCQ_STRAFE_RIGHT_TAN_END = -45;


    public static double RCQ_SHOOT_SINGLE_STACK_X = -3;//RC_SHOOT_HIGH_WOB_X;
    public static double RCQ_SHOOT_SINGLE_STACK_Y = RC_SHOOT_HIGH_WOB_Y;
    public static double RCQ_SHOOT_SINGLE_STACK_HEADING = RC_SHOOT_HIGH_WOB_HEADING;
    public static double RCQ_SHOOT_SINGLE_STACK_TAN_END = 35;
    public static double RCQ_SHOOT_SINGLE_STACK_TAN_BEGIN = 0;

    public static double RC0_DROP_SECOND_WOB_X = 3;
    public static double RC0_DROP_SECOND_WOB_Y = -45;
    public static double RC0_DROP_SECOND_WOB_HEADING = 90;
    public static double RC0_DROP_SECOND_WOB_TAN_END = -90;
    public static double RC0_DROP_SECOND_WOB_TAN_BEGIN = -25;

    public static double RC1_DROP_SECOND_WOB_X = 17;
    public static double RC1_DROP_SECOND_WOB_Y = -36.5;
    public static double RC1_DROP_SECOND_WOB_HEADING = 180;
    public static double RC1_DROP_SECOND_WOB_TAN_END = 0;
    public static double RC1_DROP_SECOND_WOB_TAN_BEGIN = 0;

    public static double RCQ_DROP_SECOND_WOB_X = 44;
    public static double RCQ_DROP_SECOND_WOB_Y = -44;
    public static double RCQ_DROP_SECOND_WOB_HEADING = 130;
    public static double RCQ_DROP_SECOND_WOB_TAN_END = -60;            ;
    public static double RCQ_DROP_SECOND_WOB_TAN_BEGIN = -205;

    public static double RCQ_STRAFE_RIGHT = 14;
    public static double RCQ_COLLECT_FORWARD = 21;
    public static double RCQ_COLLECT_LAST_FORWARD = 8;

    public static double RC0_SECOND_PARK_X = 5;
    public static double RC0_SECOND_PARK_Y = -36;
    public static double RC0_SECOND_PARK_HEADING = -1;
    public static double RC0_SECOND_PARK_TAN_END = 15;

    public static double RC1_SECOND_PARK_X = 8;
    public static double RC1_SECOND_PARK_Y = -36;
    public static double RC1_SECOND_PARK_HEADING = 0;
    public static double RC1_SECOND_PARK_TAN_END = 90;

    public static double RCQ_COLLECT_LAST_X = 6;//-8;
    public static double RCQ_COLLECT_LAST_Y = -40; //-50;
    public static double RCQ_COLLECT_LAST_HEADING = 2;
    public static double RCQ_COLLECT_LAST_TAN_END = -180; ////-250;

    public static double RCQ_SECOND_PARK_X = 8;
    public static double RCQ_SECOND_PARK_Y = -36;
    public static double RCQ_SECOND_PARK_HEADING = 180;
    public static double RCQ_SECOND_PARK_TAN_END = 180;

    // Reused Trajectory Variables
    public static double start_x = 0, start_y = 0, start_angle = 0, start_tangent_angle = 0;
    public static double end_x = 0, end_y = 0, end_angle = 0, end_tangent_angle = 0;

    // Timestamp that is reset between each autonomous step
    private ElapsedTime runtime = new ElapsedTime();
    public static double markedTime = 0;

    // Vuforia Stuff
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private Vector2d pos_from_home = new Vector2d(0, 0);
    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Road Runner Mecanum Drive
    public static SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // This is the base robot class... all motors and encoders are initialized from this
        drive = new SampleMecanumDrive(hardwareMap);

        initVuforia(FtcDashboard.getInstance());
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("TFOD ON:", 1);
        telemetry.update();
        markedTime = runtime.milliseconds();

        String numRings = "";
        while(!isStarted()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    numRings = "none";
                    for (Recognition r: updatedRecognitions) {
                        numRings = r.getLabel().toLowerCase();
                        telemetry.addData("obj", r.getLabel());
                    }
                    telemetry.addData("updatedRecognitions", updatedRecognitions.toArray());
                    telemetry.update();
                }
            }
        }

        telemetry.update();

        switch (numRings)
        {
            case "quad":
                ringFound = RingMode.QUAD;
                break;
            case "single":
                ringFound = RingMode.SINGLE;
                break;
            default:
                ringFound = RingMode.NONE;
        }

        autoStep = AutonomousSTEPS.AS_DROP_FIRST_WOBBLE_DRIVE;
        markedTime = runtime.milliseconds();

        // Set all servos to their default positions
        Collector.collector_drop();
        Collector.collect_stop();
        Shooter.dont_shoot();
        Shooter.flap_custom(0.0); // Shooter.flap_open() // Don't touch flap yet
        Conveyor.conveyor_stop();
        Wobble.wobble_grip();
        Wobble.wobble_arm_up();
        PinkSubsystem.set_servo_positions();
        Conveyor.top_gate_up();
        Conveyor.flap_open();
        Collector.ringblocker_folded();

        // This is the starting position during auto
        drive.setPoseEstimate(new Pose2d(RED_CORNER_START_X, RED_CORNER_START_Y, Math.toRadians(RED_CORNER_START_HEADING)));

        Trajectory trajectory1 = BuildSimpleTrajectory(RED_CORNER_START_X, RED_CORNER_START_Y, RED_CORNER_START_HEADING, RED_CORNER_START_TAN_START,
                RCQ_DROP_FIRST_WOB_X, RCQ_DROP_FIRST_WOB_Y, RCQ_DROP_FIRST_WOB_HEADING, RCQ_DROP_FIRST_WOB_TAN_END);

        // Wait for Start Button (Phone) to be pressed
        waitForStart();

        Trajectory trajectory = null;
        AutonomousSTEPS lastAutoStep = AutonomousSTEPS.AS_DROP_FIRST_WOBBLE_DRIVE;
        // Loop as long as auto is Active or stopped is pressed
        while (opModeIsActive() && !isStopRequested()) {
            switch(autoStep)
            {
                case AS_DROP_FIRST_WOBBLE_DRIVE:
                    switch(startingFieldPosition)
                    {
                        case SP_CORNER_RED:
                            switch(ringFound)
                            {
                                case NONE:
                                    trajectory = BuildSimpleTrajectory(RED_CORNER_START_X, RED_CORNER_START_Y, RED_CORNER_START_HEADING, RED_CORNER_START_TAN_START,
                                            RC0_DROP_FIRST_WOB_X, RC0_DROP_FIRST_WOB_Y, RC0_DROP_FIRST_WOB_HEADING, RC0_DROP_FIRST_WOB_TAN_END);
                                    break; // NONE
                                case SINGLE:
                                    trajectory = BuildSimpleTrajectory(RED_CORNER_START_X, RED_CORNER_START_Y, RED_CORNER_START_HEADING, RED_CORNER_START_TAN_START,
                                            RC1_DROP_FIRST_WOB_X, RC1_DROP_FIRST_WOB_Y, RC1_DROP_FIRST_WOB_HEADING, RC1_DROP_FIRST_WOB_TAN_END);
                                    break; // SINGLE
                                case QUAD:
                                    trajectory = trajectory1;
                                    break; // QUAD
                            }
                            break; // SP_CORNER_RED:
                        case SP_MIDDLE_RED:
                            break; // SP_CENTER_RED
                        case SP_CORNER_BLUE:
                            break; // SP_CORNER_BLUE
                        case SP_MIDDLE_BLUE:
                            break; // SP_CENTER_BLUE
                    } //  switch(startingFieldPosition)

                    autoStep = AutonomousSTEPS.AS_DROP_FIRST_WOBBLE_DROP;
                    break; // AS_DROP_FIRST_WOBBLE_DRIVE:

                case AS_DROP_FIRST_WOBBLE_DROP:
                    Wobble.wobble_arm_down();
                    Collector.collector_drop();
                    Shooter.flap_custom(Presets.SHOOTER_FLAP_OPEN_AUTO); // Shooter.flap_open()
                    autoStep = AutonomousSTEPS.AS_DROP_FIRST_WOBBLE_RELEASE;
                    break; // AS_DROP_FIRST_WOBBLE_DROP

                case AS_DROP_FIRST_WOBBLE_RELEASE:
                    if(runtime.milliseconds() - markedTime > 300) {
                        Wobble.wobble_ungrip();
                        Collector.collector_drop();
                        Shooter.shootPower(0.92);
                        Conveyor.top_gate_up(); //Conveyor.flap_open();
                        autoStep = AutonomousSTEPS.AS_SHOOT_FIRST_3_HIGH_DRIVE;
                    }
                    break; // AS_DROP_FIRST_WOBBLE_RELEASE

                case AS_SHOOT_FIRST_3_HIGH_DRIVE:
                    if(runtime.milliseconds() - markedTime > 100) {
                        Wobble.wobble_arm_up();
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED:
                                switch (ringFound) {
                                    case NONE:
                                        Shooter.flap_custom(Presets.SHOOTER_FLAP_OPEN_AUTO);
                                        trajectory = BuildSimpleTrajectorySlow(RC0_DROP_FIRST_WOB_X, RC0_DROP_FIRST_WOB_Y, RC0_DROP_FIRST_WOB_HEADING, RC0_DROP_FIRST_WOB_TAN_BEGIN,
                                                RC_SHOOT_HIGH_WOB_X, RC_SHOOT_HIGH_WOB_Y, RC_SHOOT_HIGH_WOB_HEADING, RC0_SHOOT_HIGH_WOB_TAN_END);
                                        break; // NONE
                                    case SINGLE:
                                        trajectory = BuildSimpleTrajectorySlow(RC1_DROP_FIRST_WOB_X, RC1_DROP_FIRST_WOB_Y, RC1_DROP_FIRST_WOB_HEADING, RC1_DROP_FIRST_WOB_TAN_BEGIN,
                                                RC_SHOOT_HIGH_WOB_X, RC_SHOOT_HIGH_WOB_Y, RC_SHOOT_HIGH_WOB_HEADING, RC1_SHOOT_HIGH_WOB_TAN_END);
                                        break; // SINGLE
                                    case QUAD:
                                        trajectory = BuildSimpleTrajectory(RCQ_DROP_FIRST_WOB_X, RCQ_DROP_FIRST_WOB_Y, RCQ_DROP_FIRST_WOB_HEADING, RCQ_DROP_FIRST_WOB_TAN_BEGIN,
                                                RC_SHOOT_HIGH_WOB_X, RC_SHOOT_HIGH_WOB_Y, RCQ_SHOOT_HIGH_WOB_HEADING, RC1_SHOOT_HIGH_WOB_TAN_END);
                                        break; // QUAD
                                }
                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)

                        autoStep = AutonomousSTEPS.AS_SHOOT_FIRST_3_SHOOT;
                    }
                    break; // AS_DROP_FIRST_WOBBLE_DRIVE:

                case AS_SHOOT_FIRST_3_SHOOT:
                    if(runtime.milliseconds() - markedTime < 1350) { // run this for enough time to shoot
                        if(ringFound == RingMode.QUAD) {
                            Conveyor.collect(HIGH_SHOT_SPINDEXER_POWER);
                        } else {
                            Conveyor.collect(0.9);
                        }
                        Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), Presets.TELEOP_HIGH_PID_RPM_TARGET);
                        double currentShooterVelocity = PinkSubsystem.robot.shoot2.getVelocity();

                        if (currentShooterVelocity > Presets.TELEOP_HIGH_PID_RPM_TARGET_LOW && currentShooterVelocity < Presets.TELEOP_HIGH_PID_RPM_TARGET_HIGH) {
                            Conveyor.top_gate_up();//Conveyor.flap_open();
                        }
                    } else {
                        if(ringFound == RingMode.QUAD) {
                            autoStep = AutonomousSTEPS.AS_COLLECT_CENTER_WOBBLE;
                            Shooter.dont_shoot();
                            Conveyor.conveyor_stop();
                        } else {
                            if(runtime.milliseconds() - markedTime > 1850) {
                                autoStep = AutonomousSTEPS.AS_COLLECT_CENTER_WOBBLE;
                                Shooter.dont_shoot();                                //Conveyor.flap_close();
                                Conveyor.conveyor_stop();
                            }
                        }
                    }
                    break; // AS_SHOOT_FIRST_3_SHOOT

                case AS_COLLECT_CENTER_WOBBLE:
                    if(runtime.milliseconds() - markedTime > 100) {
                        Wobble.wobble_arm_down();
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED: // Always start from shoot, so no need for multiple positions
                                switch (ringFound) {
                                    case NONE:
                                    case SINGLE:
                                        trajectory = BuildSimpleTrajectorySlow(RC_SHOOT_HIGH_WOB_X, RC_SHOOT_HIGH_WOB_Y, RC_SHOOT_HIGH_WOB_HEADING, RC_SHOOT_HIGH_WOB_TAN_BEGIN,
                                                RC_COLLECT_MID_WOB_X, RC_COLLECT_MID_WOB_Y, RC_COLLECT_MID_WOB_HEADING, RC_COLLECT_MID_WOB_TAN_END);
                                        break; // SINGLE
                                    case QUAD:
                                        trajectory = BuildSimpleTrajectorySlow(RC_SHOOT_HIGH_WOB_X, RC_SHOOT_HIGH_WOB_Y, RC_SHOOT_HIGH_WOB_HEADING, RCQ_SHOOT_HIGH_WOB_TAN_BEGIN,
                                                RCQ_COLLECT_MID_WOB_X, RCQ_COLLECT_MID_WOB_Y, RCQ_COLLECT_MID_WOB_HEADING, RCQ_COLLECT_MID_WOB_TAN_END);
                                        break; // QUAD
                                }
                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)

                        autoStep = AutonomousSTEPS.AS_COLLECT_CENTER_GRIP;
                    }
                    break; // AS_COLLECT_CENTER_WOBBLE:

                case AS_COLLECT_CENTER_GRIP:
                    Conveyor.top_gate_down();
                    if(ringFound == RingMode.QUAD) {
                        Wobble.wobble_grip();
                        autoStep = AutonomousSTEPS.AS_COLLECT_CENTER_LIFT;
                    } else
                    {
                        if(runtime.milliseconds() - markedTime > 1200) {
                            Wobble.wobble_grip();
                            autoStep = AutonomousSTEPS.AS_COLLECT_CENTER_LIFT;
                        }
                    }
                    break; // AS_DROP_FIRST_WOBBLE_DROP

                case AS_COLLECT_CENTER_LIFT:
                    double liftdelay = 0;
                    if(ringFound == RingMode.QUAD) {
                        liftdelay = 450;
                    } else {
                        liftdelay = 520;
                    }
                    if(runtime.milliseconds() - markedTime > liftdelay) {
                        Wobble.wobble_arm_up();
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED:
                                switch (ringFound) {
                                    case NONE:
                                        autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_DRIVE;
                                        break; // NONE
                                    case SINGLE:
                                        autoStep = AutonomousSTEPS.AS_COLLECT_SINGLE_RING_DRIVE;
                                        break; // SINGLE
                                    case QUAD:
                                        autoStep = AutonomousSTEPS.AS_COLLECT_QUAD_RING_STRAFE;
                                        break; // QUAD
                                }
                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)


                    }
                    break; // AS_DROP_FIRST_WOBBLE_RELEASE

                case AS_COLLECT_QUAD_RING_STRAFE:
                    if(runtime.milliseconds() - markedTime > 120) {
                        // Get ready to collect the ring on the trajectory to shoot
                        Conveyor.top_gate_down(); //Conveyor.flap_close();
                        Collector.collectAt(-0.6);
                        Conveyor.collect(0.90);
                        Conveyor.flap_close();
                        Shooter.shootPower(0.77); // Start spinning up the shooter so it is ready to shoot on the next step.

                       /* trajectory =
                        drive.trajectoryBuilder(new Pose2d(RCQ_COLLECT_MID_WOB_X, RCQ_COLLECT_MID_WOB_Y, Math.toRadians(RCQ_COLLECT_MID_WOB_HEADING)))
                                .strafeRight(RCQ_STRAFE_RIGHT)
                                .build();
*/
                        trajectory = BuildSimpleTrajectory(RCQ_COLLECT_MID_WOB_X, RCQ_COLLECT_MID_WOB_Y, RCQ_COLLECT_MID_WOB_HEADING, 225,
                                RCQ_STRAFE_RIGHT_X, RCQ_STRAFE_RIGHT_Y, RCQ_STRAFE_RIGHT_HEADING, RCQ_STRAFE_RIGHT_TAN_END);

                        autoStep = AutonomousSTEPS.AS_COLLECT_QUAD_RING_DRIVE;
                    }
                    break;

                case AS_COLLECT_SINGLE_RING_DRIVE:
                    if(runtime.milliseconds() - markedTime > 180) {
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED:
                                // Get ready to collect the ring on the trajectory to shoot
                                Conveyor.top_gate_down(); //Conveyor.flap_close();
                                Collector.collectAt(-0.6);
                                Conveyor.collect(1.0);
                                Shooter.shootPower(0.82); // Start spinning up the shooter so it is ready to shoot on the next step.
                                drive.turn(Math.toRadians(-45)); // Turn -45 degrees to face the ring
                                /*trajectory = BuildSimpleTrajectory(RC_COLLECT_MID_WOB_X, RC_COLLECT_MID_WOB_Y, -5, 0,
                                        RC_COLLECT_MID_WOB_X, -24, -45, 0);
                                drive.followTrajectory(trajectory);
                                */
                                trajectory = BuildSimpleTrajectorySlow(RC_COLLECT_MID_WOB_X, RC_COLLECT_MID_WOB_Y, -45, RC1_COLLECT_MID_WOB_TAN_BEGIN,
                                        RC1_SHOOT_SINGLE_STACK_X, RC1_SHOOT_SINGLE_STACK_Y, 6, RC1_SHOOT_SINGLE_STACK_TAN_END);

                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)
                        autoStep = AutonomousSTEPS.AS_SHOOT_SINGLE_FROM_STACK;
                    }
                    break;

                case AS_COLLECT_QUAD_RING_DRIVE:
                    if(runtime.milliseconds() - markedTime > 100) {
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED:
                                Shooter.flap_custom(Presets.SHOOTER_FLAP_OPEN_AUTO + 0.002);// Presets.SHOOTER_FLAP_OPEN_AUTO_FAR);
                                trajectory =
                                        drive.trajectoryBuilder(new Pose2d(RCQ_COLLECT_MID_WOB_X, RCQ_COLLECT_MID_WOB_Y-RCQ_STRAFE_RIGHT, Math.toRadians(RCQ_COLLECT_MID_WOB_HEADING)))
                                                .forward(RCQ_COLLECT_FORWARD)
                                                .build();
                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)
                        autoStep = AutonomousSTEPS.AS_SHOOT_SINGLE_FROM_STACK;
                    }
                    break;

                case AS_SHOOT_SINGLE_FROM_STACK:
                    if(runtime.milliseconds() - markedTime < 1500) { // run this for enough time to shoot
                        Conveyor.collect(1.00);
                        Collector.eject_slow();
                        Conveyor.flap_open();
                        Conveyor.top_gate_up();
                        Shooter.flap_custom(Presets.SHOOTER_FLAP_OPEN_AUTO - 0.003);// Presets.SHOOTER_FLAP_OPEN_AUTO_FAR);
                        Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), Presets.TELEOP_HIGH_PID_RPM_TARGET);
                        double currentShooterVelocity = PinkSubsystem.robot.shoot2.getVelocity();

                        if (currentShooterVelocity > Presets.TELEOP_HIGH_PID_RPM_TARGET_LOW && currentShooterVelocity < Presets.TELEOP_HIGH_PID_RPM_TARGET_HIGH) {
                            Conveyor.flap_open();
                            Conveyor.top_gate_up();
                            if(ringFound != RingMode.QUAD) {
                                if (runtime.milliseconds() - markedTime > 1950)
                                {
                                    Collector.collect_stop();
                                    autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_DRIVE;
                                }
                            } else {
                                if (runtime.milliseconds() - markedTime > 1950) {
                                    autoStep = AutonomousSTEPS.AS_COLLECT_LAST_QUAD_STRAIGHT_DRIVE;
                                }
                            }
                        }
                    } else {
                        if(ringFound != RingMode.QUAD)
                        {
                            Conveyor.conveyor_stop();
                            Shooter.dont_shoot();
                            Collector.collect_stop();
                            autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_DRIVE;
                        } else {
                            autoStep = AutonomousSTEPS.AS_COLLECT_LAST_QUAD_STRAIGHT_DRIVE;
                        }
                    }

                    break;

                case AS_COLLECT_LAST_QUAD_STRAIGHT_DRIVE:
                    if(runtime.milliseconds() - markedTime > 950) { // run this for enough time to shoot
                            Collector.collect();
                            Conveyor.collect(1.0);
                            Shooter.shootPower(0.9);
                            Shooter.flap_custom(Presets.SHOOTER_FLAP_OPEN_AUTO - 0.003); // Shooter.flap_open()
                            trajectory =
                                    drive.trajectoryBuilder(new Pose2d(RCQ_COLLECT_MID_WOB_X + RCQ_COLLECT_FORWARD, RCQ_COLLECT_MID_WOB_Y - RCQ_STRAFE_RIGHT + 2, Math.toRadians(RCQ_COLLECT_MID_WOB_HEADING+1)))
                                            .forward(RCQ_COLLECT_LAST_FORWARD)
                                            .build();

                            autoStep = AutonomousSTEPS.AS_COLLECT_LAST_QUAD_STRAIGHT_SHOOT;
                        }
                    break;

                case AS_COLLECT_LAST_QUAD_STRAIGHT_SHOOT:
                    if(runtime.milliseconds() - markedTime > 950) {
                        if (runtime.milliseconds() - markedTime < 2050) { // run this for enough time to shoot
                            Conveyor.collect(0.87);
                            Shooter.flap_custom(Presets.SHOOTER_FLAP_OPEN_AUTO - 0.003); // Shooter.flap_open()
                            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), Presets.TELEOP_HIGH_PID_RPM_TARGET);
                            double currentShooterVelocity = PinkSubsystem.robot.shoot2.getVelocity();

                            if (currentShooterVelocity > Presets.TELEOP_HIGH_PID_RPM_TARGET_LOW) {
                                Conveyor.top_gate_up(); //Conveyor.flap_open();
                                autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_DRIVE;
                            }
                        } else {
                            Conveyor.conveyor_stop();
                            Collector.collect_stop();
                            autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_DRIVE;
                        }
                    }
                    break;

                case AS_DROP_SECOND_WOBBLE_DRIVE:
                    double secondWaitTime = 100;
                    if(ringFound != RingMode.QUAD) // If Not QUAD.... WAIT
                    {
                        secondWaitTime = 250;
                    }
                    if(runtime.milliseconds() - markedTime > secondWaitTime) {
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED:
                                switch (ringFound) {
                                    case NONE:
                                        trajectory = BuildSimpleTrajectorySlow(RC_COLLECT_MID_WOB_X, RC_COLLECT_MID_WOB_Y, RC_COLLECT_MID_WOB_HEADING, RC0_COLLECT_MID_WOB_TAN_BEGIN,
                                                RC0_DROP_SECOND_WOB_X, RC0_DROP_SECOND_WOB_Y, RC0_DROP_SECOND_WOB_HEADING, RC0_DROP_SECOND_WOB_TAN_END);
                                        break; // NONE
                                    case SINGLE:
                                        trajectory = BuildSimpleTrajectorySlow(RC1_SHOOT_SINGLE_STACK_X, RC1_SHOOT_SINGLE_STACK_Y, RC1_SHOOT_SINGLE_STACK_HEADING, RC1_SHOOT_SINGLE_STACK_TAN_BEGIN,
                                                RC1_DROP_SECOND_WOB_X, RC1_DROP_SECOND_WOB_Y, RC1_DROP_SECOND_WOB_HEADING, RC1_DROP_SECOND_WOB_TAN_END);
                                        break; // SINGLE
                                    case QUAD:
                                        trajectory = BuildSimpleTrajectory(RCQ_COLLECT_MID_WOB_X + RCQ_COLLECT_FORWARD + RCQ_COLLECT_LAST_FORWARD, RCQ_COLLECT_MID_WOB_Y - RCQ_STRAFE_RIGHT, RC1_SHOOT_SINGLE_STACK_HEADING, 0,
                                                RCQ_DROP_SECOND_WOB_X, RCQ_DROP_SECOND_WOB_Y, RCQ_DROP_SECOND_WOB_HEADING, RCQ_DROP_SECOND_WOB_TAN_END);
                                        break; // QUAD
                                }
                                autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_DROP;
                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)

                        autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_DROP;
                    }
                    break; // AS_COLLECT_CENTER_WOBBLE:

                case AS_DROP_SECOND_WOBBLE_DROP:
                    if(ringFound == RingMode.QUAD) {
                        Wobble.wobble_arm_down();
                        Shooter.dont_shoot();
                        Conveyor.top_gate_down();//Conveyor.flap_close();
                        Conveyor.conveyor_stop();

                        autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_RELEASE;
                    } else {
                        if(runtime.milliseconds() - markedTime > 300) {
                            Wobble.wobble_arm_down();
                            Shooter.dont_shoot();
                            Conveyor.top_gate_down();//Conveyor.flap_close();
                            Conveyor.conveyor_stop();

                            autoStep = AutonomousSTEPS.AS_DROP_SECOND_WOBBLE_RELEASE;
                        }
                    }
                    break; // AS_DROP_FIRST_WOBBLE_DROP

                case AS_DROP_SECOND_WOBBLE_RELEASE:
                    if(runtime.milliseconds() - markedTime > 180) {
                        Wobble.wobble_ungrip();
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED:
                                switch (ringFound) {
                                    case NONE:
                                    case SINGLE:
                                        if (runtime.milliseconds() - markedTime > 500) {
                                            autoStep = AutonomousSTEPS.AS_PARK;
                                        }
                                        break; // SINGLE
                                    case QUAD:
                                        autoStep = AutonomousSTEPS.AS_QUAD_COLLECT_LAST_DRIVE;
                                        break; // QUAD
                                }

                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)

                    }
                    break; // AS_DROP_FIRST_WOBBLE_RELEASE

                case AS_QUAD_COLLECT_LAST_DRIVE:
                    Wobble.wobble_arm_up();
                    Conveyor.top_gate_down();//Conveyor.flap_close();
                    Collector.collect_stop();
                    Conveyor.conveyor_stop();
                    //Shooter.flap_power_shot();
                    //Shooter.flap_custom(0.415);
                    //Conveyor.collect(1);
                    //Shooter.shootPower(0.8); // Start spinning up the shooter so it is ready to shoot on the next step.
                    trajectory = BuildSplineTrajectory(RCQ_DROP_SECOND_WOB_X, RCQ_DROP_SECOND_WOB_Y, RCQ_DROP_SECOND_WOB_HEADING, RCQ_DROP_SECOND_WOB_TAN_BEGIN,
                            RCQ_COLLECT_LAST_X, RCQ_COLLECT_LAST_Y, RCQ_COLLECT_LAST_HEADING, RCQ_COLLECT_LAST_TAN_END);

                    autoStep = AutonomousSTEPS.AS_AUTONOMOUS_END;
                    break; // AS_QUAD_COLLECT_LAST_DRIVE

                case AS_QUAD_LAST_SHOOT_DRIVE:
                    //drive.turn(Math.toRadians(-197));
                    autoStep = AutonomousSTEPS.AS_QUAD_LAST_SHOOT;
                    break; // AS_QUAD_LAST_SHOOT_DRIVE

                case AS_QUAD_LAST_SHOOT:
                    if(runtime.milliseconds() - markedTime < 450) { // run this for enough time to shoot
                        Conveyor.collect(1);
                        Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), Presets.TELEOP_HIGH_PID_RPM_TARGET);
                        double currentShooterVelocity = PinkSubsystem.robot.shoot2.getVelocity();

                        if (currentShooterVelocity > Presets.TELEOP_HIGH_PID_RPM_TARGET_LOW) {
                            Conveyor.top_gate_up(); //Conveyor.flap_open();
                            autoStep = AutonomousSTEPS.AS_PARK;
                        }
                    } else {
                        Conveyor.conveyor_stop();
                        Shooter.dont_shoot();
                        Collector.collect_stop();
                        autoStep = AutonomousSTEPS.AS_PARK;
                    }
                    break; // AS_QUAD_LAST_SHOOT

                case AS_PARK:
                    if(runtime.milliseconds() - markedTime > 280) {
                        switch (startingFieldPosition) {
                            case SP_CORNER_RED:
                                switch (ringFound) {
                                    case NONE:
                                        trajectory = BuildSimpleTrajectorySlow(RC0_DROP_SECOND_WOB_X, RC0_DROP_SECOND_WOB_Y, RC0_DROP_SECOND_WOB_HEADING, RC0_DROP_SECOND_WOB_TAN_BEGIN,
                                                RC0_SECOND_PARK_X, RC0_SECOND_PARK_Y, RC0_SECOND_PARK_HEADING, RC0_SECOND_PARK_TAN_END);
                                        autoStep = AutonomousSTEPS.AS_AUTONOMOUS_END;
                                        break; // NONE
                                    case SINGLE:
                                        Wobble.wobble_arm_up();
                                        if(runtime.milliseconds() - markedTime > 1450) {
                                            trajectory = BuildSimpleTrajectorySlow(RC1_DROP_SECOND_WOB_X, RC1_DROP_SECOND_WOB_Y, RC1_DROP_SECOND_WOB_HEADING, RC1_DROP_SECOND_WOB_TAN_BEGIN,
                                                    RC1_SECOND_PARK_X, RC1_SECOND_PARK_Y, RC1_SECOND_PARK_HEADING, RC1_SECOND_PARK_TAN_END);
                                            autoStep = AutonomousSTEPS.AS_AUTONOMOUS_END;
                                        }
                                        break; // SINGLE
                                    case QUAD:
                                        trajectory =
                                                drive.trajectoryBuilder(new Pose2d(RCQ_COLLECT_LAST_X, RCQ_COLLECT_LAST_Y, Math.toRadians(RCQ_COLLECT_LAST_HEADING)), Math.toRadians(RCQ_COLLECT_LAST_TAN_END))
                                                        .forward(8)
                                                        .build();
                                        autoStep = AutonomousSTEPS.AS_AUTONOMOUS_END;
                                        //BuildSimpleTrajectory(RCQ_COLLECT_LAST_X, RCQ_COLLECT_LAST_Y, RCQ_COLLECT_LAST_HEADING, 0,
                                        //RCQ_COLLECT_LAST_X + 8, RCQ_COLLECT_LAST_Y, RCQ_COLLECT_LAST_HEADING, 0);
                                        break; // QUAD
                                }
                                break; // SP_CORNER_RED:
                            case SP_MIDDLE_RED:
                                break; // SP_CENTER_RED
                            case SP_CORNER_BLUE:
                                break; // SP_CORNER_BLUE
                            case SP_MIDDLE_BLUE:
                                break; // SP_CENTER_BLUE
                        } //  switch(startingFieldPosition)


                    }
                    break; // AS_COLLECT_CENTER_WOBBLE:

                case AS_AUTONOMOUS_END:

                    break; // AS_AUTONOMOUS_END
            } // switch(autoStep)

            // Update Servo Values
            PinkSubsystem.set_servo_positions();
            // Update Motor power
            PinkSubsystem.set_motor_powers();

            // If there is a trajectory define... drive it
            if(trajectory != null)
            {
                drive.followTrajectory(trajectory);
                // Reset trajectory to null once driven
                trajectory = null;
            }

            // Reset MarkedTime every time we jump to another step
            if(lastAutoStep != autoStep) {
                markedTime = runtime.milliseconds();
                lastAutoStep = autoStep;
            }

            // Stop the while loop if Autonomous is completed
            if(autoStep == AutonomousSTEPS.AS_AUTONOMOUS_END)
            {
                break;
            }

        } // while opmode is active and stop not requested
    } // Void runopmode()

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

    public static Trajectory BuildSimpleTrajectorySlow(double beginX, double beginY, double beginHeadingAngle,  double beginTangentAngle,
                                                   double endX, double endY, double endHeadingAngle,  double endTangentAngle)
    {
        Trajectory newTractory =
                drive.trajectoryBuilderSlow(new Pose2d(beginX, beginY, Math.toRadians(beginHeadingAngle)), Math.toRadians(beginTangentAngle))
                        .splineToLinearHeading(new Pose2d(endX, endY, Math.toRadians(endHeadingAngle)), Math.toRadians(endTangentAngle))
                        .build();

        return newTractory;
    }

    public static Trajectory BuildSplineTrajectory(double beginX, double beginY, double beginHeadingAngle,  double beginTangentAngle,
                                                   double endX, double endY, double endHeadingAngle,  double endTangentAngle)
    {
        Trajectory newTractory =

                drive.trajectoryBuilder(new Pose2d(beginX, beginY, Math.toRadians(beginHeadingAngle)), Math.toRadians(beginTangentAngle))
                        .splineToSplineHeading(new Pose2d(endX, endY, Math.toRadians(endHeadingAngle)), Math.toRadians(endTangentAngle))
                        //.splineTo(new Vector2d(endX, endY), Math.toRadians(endTangentAngle))
                        .build();

        return newTractory;
    }

    public static Trajectory BuildSplineTrajectorySlow(double beginX, double beginY, double beginHeadingAngle,  double beginTangentAngle,
                                                   double endX, double endY, double endHeadingAngle,  double endTangentAngle)
    {
        Trajectory newTractory =
                drive.trajectoryBuilderSlow(new Pose2d(beginX, beginY, Math.toRadians(beginHeadingAngle)), Math.toRadians(beginTangentAngle))
                        .splineToSplineHeading(new Pose2d(endX, endY, Math.toRadians(endHeadingAngle)), Math.toRadians(endTangentAngle))
                        //.splineTo(new Vector2d(endX, endY), Math.toRadians(endTangentAngle))
                        .build();

        return newTractory;
    }

    public static Trajectory BuildSimpleTrajectory2(double beginX, double beginY, double beginHeadingAngle,  double beginTangentAngle,
                                                    double endX, double endY, double endHeadingAngle,  double endTangentAngle)
    {
        Trajectory newTractory =
                drive.trajectoryBuilder(new Pose2d(beginX, beginY, Math.toRadians(beginHeadingAngle)), Math.toRadians(beginTangentAngle))
                        .splineToSplineHeading(new Pose2d(endX, endY, Math.toRadians(endHeadingAngle)), Math.toRadians(endTangentAngle))
                        .build();

        return newTractory;
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        dash.startCameraStream(tfod, 20);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia(FtcDashboard dash) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = PinkSubsystem.robot.webcam;


        ClassFactory.getInstance().getCameraManager().nameForUnknownCamera();
//        dash.startCameraStream(vuforia, 0);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}