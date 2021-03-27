package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

//FIRST-provided Imports
import android.view.textclassifier.ConversationAction;

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

//Subsystem Imports
import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.Presets;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Collector;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Base;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Controls;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Wobble;


// Class for Player-Controlled Period of the Game Which Binds Controls to Subsystems
@TeleOp(name = "TeleOp", group = "TeleOp")
@Disabled
public class Teleop extends Controls {

    //Variables
    double telemTemp = 0;
    boolean isPressed = false;
    double shootTemp = 0;
    double shootTime = 0;
    double sTimeTemp = 0;

    double HIGH_SHOT_SPINDEXER_POWER  = 1.00;//0.70; // Good Safe Value
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isShootingHigh = false;

    // Code to Run Once When the Drivers Press Init
    public void init() {
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
                Base.drive_by_command(-v1, -v2, -v3, -v4);

        } else {
            //If no movement on base controller, stop base
            Base.drive_stop();
        }

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
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), 1500);
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
        } else if (gamepad2.left_bumper && PinkSubsystem.robot.shoot2.getVelocity() > 1425 && PinkSubsystem.robot.shoot2.getVelocity() < 1600) {
            Conveyor.flap_open();
            //Conveyor.top_gate_up();
            Conveyor.collect(.65);
            //start shooting if shootTemp = 1 from previous code
        } else if (shootTemp == 1) {
            Conveyor.flap_open();
            Conveyor.top_gate_up();
            //Conveyor.top_gate_up();
            Conveyor.collect(HIGH_SHOT_SPINDEXER_POWER);
            //if left bumper pressed, eject rings
        } else if(gamepad1.left_bumper) {           // ----- EJECT ALL RINGS FROM BOT
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
        PinkSubsystem.set_motor_powers();
        PinkSubsystem.set_servo_positions();



        if (gamepad1.dpad_down) {
            telemTemp = 0;
        } else if (gamepad1.dpad_up) {
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