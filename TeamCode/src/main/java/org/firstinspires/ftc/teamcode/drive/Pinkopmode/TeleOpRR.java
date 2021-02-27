package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

//FIRST-provided Imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;

//Imu Imports
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Subsystem Imports
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Collector;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Base;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Controls;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Wobble;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Arrays;


// Class for Player-Controlled Period of the Game Which Binds Controls to Subsystems
@TeleOp(name = "TeleOpRR", group = "TeleOpRR")
@Disabled
public class TeleOpRR extends Controls {

    //Variables
    double telemTemp = 0;
    boolean isPressed = false;
    double shootTemp = 0;
    double shootTime = 0;
    double sTimeTemp = 0;

    double HIGH_SHOT_SPINDEXER_POWER  = 0.80;//0.765; // Good Safe Value

    Boolean X_button_pressed;

    //    private double previousHeading = 0;
//    private double integratedHeading = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;

    SampleMecanumDrive drive = null;

    Trajectory shootingTrajectory;
    Boolean initTrajectory = false;

    Vector2d targetBVector = new Vector2d(10.0, 10.0);

    double targetAngle = Math.toRadians(0);

    // Code to Run Once When the Drivers Press Init
    public void init() {
        DriveConstants.MAX_VEL = 120;
        DriveConstants.MAX_ACCEL = 120;
        //DriveConstants.MAX_ANG_ACCEL = 360;
        //DriveConstants.MAX_ANG_ACCEL = 360;
        drive = new SampleMecanumDrive(hardwareMap);

        shootingTrajectory = drive.trajectoryBuilder(new Pose2d())
                // This spline is limited to 15 in/s and will be slower
                .splineTo(
                        new Vector2d(30, 30), 0,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(240),
                                        new MecanumVelocityConstraint(35, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

      /*  shootingTrajectory = drive.trajectoryBuilder(new Pose2d())
                // This spline is limited to 15 in/s and will be slower
                .lineTo(new Vector2d(24,24 ))
                .build();
*/
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //imu initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialization of Each Subsystem's Hardware Map and setup of motors
        PinkSubsystem.robot.init(hardwareMap);
        PinkSubsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PinkSubsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PinkSubsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PinkSubsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Robot setup for wobble servos and collector
        Wobble.wobble_arm_up();
        Wobble.wobble_grip();
        Collector.collector_drop();
//        Scorer.score_rotate_to_position(Presets.SCORER_STOW);

        // Telemetry Update to Inform Drivers That the Program is Initialized
        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
        telemetry.update();
    }

    // Code to Run Constantly After the Drivers Press Play and Before They Press Stop
    public void loop() {
        X_button_pressed = gamepad1.x;

        if(X_button_pressed == true)
        {
            if(initTrajectory == false) {
                initTrajectory = true;
            }
        } else {
            initTrajectory = false;
        }
            // Drive Train Control
        //If any movement occurs on base controller, use math to power motor appropriately
        if (gamepad1.left_stick_y > .1  ||
                gamepad1.left_stick_y < .1  ||
                gamepad1.left_stick_x > .1  ||
                gamepad1.left_stick_x < -.1 ||
                gamepad1.right_stick_x > .1 ||
                gamepad1.right_stick_x < -.1) {

            //r is the hypotenuse of (x,y) coordinate of left stick, robotAngle = angleTheta of (x,y) coordinate of left stick. rightX = turning speed
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            //Equations below is motor speed for each wheel
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) + rightX;
            double v3 = r * Math.sin(robotAngle) - rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            //If not turning give each wheel full power
            if (gamepad1.right_stick_x == 0) {
                v1 += v1 / 3;
                v2 += v2 / 3;
                v3 += v3 / 3;
                v4 += v4 / 3;
            }

            //Base subsystem drive command passing variables from math above
            //Base.drive_by_command(-v1, -v2, -v3, -v4);

            if(X_button_pressed == false)
            {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }
        } else {
            //If no movement on base controller, stop base
            //Base.drive_stop();

                if (X_button_pressed == false)
                {
                drive.setWeightedDrivePower(
                        new Pose2d(0.0, 0.0, 0.0
                        )
                );
            }
        }

        // This will force a drive to trajectory
        if(X_button_pressed == true)
        {
            Pose2d poseEstimate = drive.getPoseEstimate();

           /* shootingTrajectory = drive.trajectoryBuilder(poseEstimate)
                    .lineTo(targetBVector)
                    .build();


*/

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            this.imu.getPosition();
// and save the heading
            double curHeading = angles.firstAngle;

            drive.turnAsync(Angle.normDelta(targetAngle - Math.toRadians(curHeading)));//poseEstimate.getHeading()));

            /* shootingTrajectory = drive.trajectoryBuilder(poseEstimate)
                    .splineTo(targetBVector, targetAngle)
                    .build();

             drive.followTrajectoryAsync(shootingTrajectory);
*/

            telemetry.addData("X BUTTON PRESSED", curHeading);
        }

        drive.update();

        // Collector Controls and Conveyor.
        // Collect if right bumper on base controller is pressed, eject if left bumper on base controller is pressed, otherwise stop the collector
        if (base_right_bumper(false)) {
            Collector.collect();
        } else if (base_left_bumper(false)) {
            Collector.eject();
        } else {
            Collector.collect_stop();
        }
        //Shoot Commands, if bumper or right trigger is used spin up motors
        if(gamepad2.right_bumper || (gamepad2.right_trigger >= 0.2)) {
            /* if running conveyor based on time instead of PD
            if(sTimeTemp == 0) {
                sTimeTemp = 1;
                shootTime = runtime.milliseconds();
            } */
            //Shoot by pd command passing current velocity and target velocity, shootPower is below, pass a power for the shooter motors to use
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), 1620);
//            Shooter.shootPower(1);
            Shooter.flap_open();
            //Power shot code
        } else if (gamepad2.left_bumper) { /// *** POWER SHOT ****
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), 1550);
//            Shooter.shoot();
            Shooter.flap_power_shot();
        } else {
            shootTemp = 0;
            Shooter.dont_shoot();
        }


        // Conveyor and shooter Controls
        // Check for right bumper pressed and within pd thresholds
        if (gamepad2.right_bumper && PinkSubsystem.robot.shoot2.getVelocity() > 1500 && PinkSubsystem.robot.shoot2.getVelocity() < 1700 && shootTemp != 1) {
            //if right trigger pressed, start shooting.
            if (gamepad2.right_trigger >= 0.2) {
                shootTemp = 1;
            }
//            Conveyor.flap_open();
//            Conveyor.collect(.7);
            //Uncomment for time instead of pd Also comment shootbypd in shoot section, and first if statement in controls.
//        if(gamepad2.right_bumper && runtime.milliseconds() - markedTime2 > 2500) {
            //Power shot code
        } else if (gamepad2.left_bumper && PinkSubsystem.robot.shoot2.getVelocity() > 1450 && PinkSubsystem.robot.shoot2.getVelocity() < 1550) {
            Conveyor.flap_open();
            Conveyor.collect(.65);
            //start shooting if shootTemp = 1 from previous code
        } else if (shootTemp == 1) {
            Conveyor.flap_open();
            Conveyor.collect(HIGH_SHOT_SPINDEXER_POWER);
            //if left bumper pressed, eject rings
        } else if(gamepad1.left_bumper) {
            Conveyor.flap_open();
            Conveyor.eject();
            //if right bumper is pressed collect rings
        } else if (gamepad1.right_bumper) {
            Conveyor.collect(1);
            Conveyor.flap_close();
            //stop conveyor if nothing is pressed
        }else {
            Conveyor.flap_close();
            Conveyor.conveyor_stop();
        }


        //Wobble controls
        //Gamepad 2 X is grip, B is release, A is down, Y is up
        if(gamepad2.x) {
            Wobble.wobble_grip();
        } else if (gamepad2.b) {
            Wobble.wobble_ungrip();
        } else if(gamepad2.a) {
            Wobble.wobble_arm_down();
        } else if (gamepad2.y){
            Wobble.wobble_arm_up();
        }

        // Set Motor Powers and Servos to Their Commands
        PinkSubsystem.set_motor_powers_no_base();
        PinkSubsystem.set_servo_positions();



        if (gamepad2.dpad_down) {
            telemTemp = 0;
        } else if (gamepad2.dpad_up) {
            telemTemp = 1;
        }

        // Add Telemetry to Phone for Debugging and Testing if it is Activated
        if(telemTemp == 1) {
            telemetry.addData("Status: ", "Running Telemetry");
            telemetry.addData("Press down on tower dpad to close telemetry", "");
            telemetry.addData("_______________Base__________________","");
            telemetry.addData("LYAXIS: ", gamepad1.left_stick_y);
            telemetry.addData("LXAXIS: ", gamepad1.left_stick_x);
            telemetry.addData("RXAXIS: ", gamepad1.right_stick_x);
            telemetry.addData("Base RightF Power: ", PinkSubsystem.robot.rightF_drive.getPower());
            telemetry.addData("Base RightB Power: ", PinkSubsystem.robot.rightB_drive.getPower());
            telemetry.addData("Base LeftF Power: ", PinkSubsystem.robot.leftF_drive.getPower());
            telemetry.addData("Base LeftB Power: ", PinkSubsystem.robot.leftB_drive.getPower());
            telemetry.addData("______________Shooter_________________", "");
            telemetry.addData("DcMotorEx Velocity", PinkSubsystem.robot.shoot2.getVelocity());
            telemetry.addData("Shooter2Power", PinkSubsystem.robot.shoot2.getPower());
            telemetry.addData("Shooter1power", PinkSubsystem.robot.shoot1.getPower());
            telemetry.update();
        } else {
            // Telemetry Update to Inform Drivers That the Program is Running and how to Access Telemetry
            telemetry.addData("Status:", "Waiting for user");
            telemetry.addData("Press up on tower dpad to view telemetry", "");
            telemetry.update();
        }

    }

    // Code to Run Once When the Drivers Press Stop
    public void stop() {

        // Stop Sending Commands to Each Subsystem
        Base.drive_stop();
        Shooter.dont_shoot();
        Conveyor.conveyor_stop();
        Collector.collect_stop();

        // Set Motor Powers and Servos to Their Commands
        PinkSubsystem.set_motor_powers();
        PinkSubsystem.set_servo_positions();

        // Telemetry Update to Inform Drivers That the Program is Stopped
        telemetry.addData("Status: ", "Stopped");
        telemetry.update();

    }

    /* Imu heading not exceeding 90 or -90; reset to 0 if passing 90 or - 90
    private double getMathHeading(double x) {

        double angle = 0;
        if(x > 0)
             angle = (x - ((Math.round((x / 360) - .5)) * 360));
        else
            angle = (x - ((Math.round((x / 360) + .5)) * 360));

        return angle;
    } */

    /* Heading of imu from infinity to -infinity
    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    } */

}