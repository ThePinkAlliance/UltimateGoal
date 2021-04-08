package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

//FIRST-provided Imports
import com.acmerobotics.roadrunner.control.PIDFController;
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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Subsystem Imports
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.Presets;
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

    private boolean isShootingHigh = false;

    // Align to target stuff
    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private double TargetHeadingYPosition = Presets.TELEOP_AUTOAIM_POS;
    private Vector2d targetPosition = new Vector2d(74, TargetHeadingYPosition);


    private double TargetHeadingPowerShotPosition = Presets.TELEOP_AUTOAIM_POS;




    // Code to Run Once When the Drivers Press Init
    public void init() {
        DriveConstants.MAX_VEL = 120;
        DriveConstants.MAX_ACCEL = 120;
        //DriveConstants.MAX_ANG_ACCEL = 360;
        //DriveConstants.MAX_ANG_ACCEL = 360;
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        PoseStorage.currentPose = new Pose2d(0,-12);
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

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
        //Collector.collector_drop();
        Collector.collector_hold();
        Conveyor.top_gate_down();
        Conveyor.flap_open();
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

        // Driver DPAD is for autoaim of PowerShot
        // Left PowerShot
        if(gamepad1.dpad_left == true)
        {
            TargetHeadingYPosition = Presets.TELEOP_AUTOAIM_POWERSHOT_RED_LEFT;
            targetPosition = new Vector2d(74, TargetHeadingYPosition);
        } else
        if(gamepad1.dpad_up == true)
        {
            TargetHeadingYPosition = Presets.TELEOP_AUTOAIM_POWERSHOT_RED_MIDDLE;
            targetPosition = new Vector2d(74, TargetHeadingYPosition);
        } else
        if(gamepad1.dpad_right == true)
        {
            TargetHeadingYPosition = Presets.TELEOP_AUTOAIM_POWERSHOT_RED_RIGHT;
            targetPosition = new Vector2d(74, TargetHeadingYPosition);
        }

        if (gamepad2.left_stick_y > .1  ||
                gamepad2.left_stick_y < .1  ||
                gamepad2.left_stick_x > .1  ||
                gamepad2.left_stick_x < -.1)
        {

            TargetHeadingYPosition -= gamepad2.left_stick_x / 20.0;

            targetPosition = new Vector2d(74, TargetHeadingYPosition);
        }

        // Reset Target position on Driver left stick click
        if(gamepad2.left_stick_button == true)
        {
            TargetHeadingYPosition = Presets.TELEOP_AUTOAIM_POS;
            targetPosition = new Vector2d(74, TargetHeadingYPosition);
        }

        // SET Robot Pose (Initial Shooting Position)
        if(gamepad1.y == true)
        {
            PoseStorage.currentPose = new Pose2d(0,-12); // This is where the robot must be on the field to remember its position
            drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

            TargetHeadingYPosition = Presets.TELEOP_AUTOAIM_POS;
            targetPosition = new Vector2d(74, TargetHeadingYPosition);
        }

        telemetry.addData("X Heading", TargetHeadingYPosition);
            // Drive Train Control
        //If any movement occurs on base controller, use math to power motor appropriately
        if (gamepad1.left_stick_y > .1  ||
                gamepad1.left_stick_y < .1  ||
                gamepad1.left_stick_x > .1  ||
                gamepad1.left_stick_x < -.1 ||
                gamepad1.right_stick_x > .1 ||
                gamepad1.right_stick_x < -.1) {

            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();

            telemetry.addData("mode", currentMode);


            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if `a` is pressed
                    if (initTrajectory) {
                        currentMode = Mode.ALIGN_TO_POINT;
                    }

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );
                    break;
                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if `b` is pressed
                    if (!initTrajectory) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );


                    break;
            }

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());



        } else {
            Base.drive_stop();
        }

        drive.update();


        // Collector Controls and Conveyor.
        // Collect if right bumper on base controller is pressed, eject if left bumper on base controller is pressed, otherwise stop the collector
        if (base_right_bumper(false)) {
            Collector.collect();
            Conveyor.top_gate_down();
        } else if (base_left_bumper(false)) {
            Collector.eject();
        } else {
            Collector.collect_stop();
        }
        //Shoot Commands, if bumper or right trigger is used spin up motors
        isShootingHigh = false;
        if(gamepad2.right_bumper || (gamepad2.right_trigger >= 0.2)) {

            //Shoot by pd command passing current velocity and target velocity, shootPower is below, pass a power for the shooter motors to use
            isShootingHigh = true;
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), Presets.TELEOP_HIGH_PID_RPM_TARGET);
            Shooter.flap_open();
            //Power shot code
        } else if (gamepad2.left_bumper) { /// *** POWER SHOT ****
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), Presets.TELEOP_POWERSHOT_PID_RPM_TARGET);
//            Shooter.shoot();
            Shooter.flap_power_shot();
            Conveyor.top_gate_up();
        } else {
            shootTemp = 0;
            Shooter.dont_shoot();
        }

        // Conveyor and shooter Controls
        // Check for right bumper pressed and within pd thresholds
        if (gamepad2.right_bumper && PinkSubsystem.robot.shoot2.getVelocity() > Presets.TELEOP_HIGH_PID_RPM_TARGET_LOW &&
                PinkSubsystem.robot.shoot2.getVelocity() < Presets.TELEOP_HIGH_PID_RPM_TARGET_HIGH && shootTemp != 1) {
            //if right trigger pressed, start shooting.
            if (gamepad2.right_trigger >= 0.2) {
                shootTemp = 1;
            }
//            Conveyor.flap_open();
//            Conveyor.collect(.7);
            //Uncomment for time instead of pd Also comment shootbypd in shoot section, and first if statement in controls.
//        if(gamepad2.right_bumper && runtime.milliseconds() - markedTime2 > 2500) {
            //Power shot code
        } else if (gamepad2.left_bumper && PinkSubsystem.robot.shoot2.getVelocity() > Presets.TELEOP_POWERSHOT_PID_RPM_TARGET_LOW && PinkSubsystem.robot.shoot2.getVelocity() < Presets.TELEOP_POWERSHOT_PID_RPM_TARGET_HIGH) {
            Conveyor.flap_open();
            //Conveyor.top_gate_up();
            Conveyor.collect(.49);
            //start shooting if shootTemp = 1 from previous code
        } else if (shootTemp == 1) {
            Conveyor.flap_open();
            Conveyor.top_gate_up();
            //Conveyor.top_gate_up();
            Conveyor.collect(HIGH_SHOT_SPINDEXER_POWER);
            //if left bumper pressed, eject rings
        } else if(gamepad1.left_trigger > 0.01) {           // ----- EJECT ALL RINGS FROM BOT
            Conveyor.flap_open();
            Conveyor.top_gate_up();
            Conveyor.eject();
            //if right bumper is pressed collect rings
        } else if (gamepad1.right_bumper) {         // ----- DRIVER COLLECTION OF RINGS
            Conveyor.collect(1);
            Conveyor.flap_close();
            Conveyor.top_gate_down();
            //stop conveyor if nothing is pressed
        }else {
            if(isShootingHigh == false) {
                Conveyor.conveyor_stop();
                Conveyor.flap_close();
            } else {
                Conveyor.flap_open();       // ------ Let the rings go into the spindexer
                Conveyor.collect(1.0);
            }
        }

        if(gamepad1.left_bumper)
        {
            Collector.eject();
        }

        // Ring Blocker
        if(gamepad2.dpad_down == true || shootTemp == 1)
        {
            Collector.ringblocker_down();
        } else {
            Collector.ringblocker_up();
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

