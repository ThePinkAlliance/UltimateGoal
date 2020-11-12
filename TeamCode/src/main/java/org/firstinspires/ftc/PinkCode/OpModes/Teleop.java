package org.firstinspires.ftc.PinkCode.OpModes;

import android.net.IpSecManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Conveyor;
import org.firstinspires.ftc.PinkCode.Subsystems.Shooter;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Base;
import org.firstinspires.ftc.PinkCode.Robot.Controls;
import org.firstinspires.ftc.PinkCode.Subsystems.Wobble;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Timer;
import java.util.TimerTask;
import java.util.zip.CheckedOutputStream;

import javax.security.auth.Subject;

// Class for Player-Controlled Period of the Game Which Binds Controls to Subsystems
@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends Controls {

    double markedTime2 = 0;
    int x = 0;
    double robotTheta;
    double sinTheta;
    double cosTheta;
    double wheel0Pos, wheel1Pos, wheel2Pos, wheelDisplacePerEncoderCount, Theta;
    double delt_m0, delt_m1, delt_m2, dev_m0, dev_m1, dev_m2, delt_Xr, delt_Yr, delt_Xf, delt_Yf, X, Y,
            displ_m0, displ_m1, displ_m2, lastM0, lastM1, lastM2, lastX, lastY, displ_averageY, displ_averageX;

    double previousPos;

    private double previousHeading = 0;
    private double integratedHeading = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;

    // Code to Run Once When the Drivers Press Init
    public void init() {

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



        // Initialization of Each Subsystem's Hardware Map
        Subsystem.robot.init(hardwareMap);
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        Scorer.score_rotate_to_position(Presets.SCORER_STOW);
        // Telemetry Update to Inform Drivers That the Program is Initialized
        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
        telemetry.update();
    }

    // Code to Run Constantly After the Drivers Press Play and Before They Press Stop
    public void loop() {

        if (Subsystem.robot.collector_drop.getPosition() == 0) {
            Collector.collector_drop();
        }


        wheelDisplacePerEncoderCount = 1/1125;
        wheel0Pos = Subsystem.robot.rightF_drive.getCurrentPosition();
        wheel1Pos = Subsystem.robot.rightB_drive.getCurrentPosition();
        wheel2Pos = Subsystem.robot.leftF_drive.getCurrentPosition();
        //Compute change in encoder positions
        delt_m0 = wheel0Pos - lastM0;
        delt_m1 = wheel1Pos - lastM1;
        delt_m2 = wheel2Pos - lastM2;
        //Compute displacements for each wheel
        displ_m0 = delt_m0 * wheelDisplacePerEncoderCount;
        displ_m1 = delt_m1 * wheelDisplacePerEncoderCount;
        displ_m2 = delt_m2 * wheelDisplacePerEncoderCount;
        //Compute the average displacement in order to untangle rotation from displacement
        displ_averageY = (displ_m1 + displ_m2) / 2.0;
        displ_averageX = (displ_m2 - displ_m1) / 2.0;
        //Compute the component of the wheel displacements that yield robot displacement
        if (getMathHeading() > 0)
            dev_m0 = displ_m0 - displ_averageX;
        else
            dev_m0 = displ_m0 + displ_averageX;
        dev_m1 = displ_m1 - displ_averageX;
        dev_m2 = displ_m2 - displ_averageX;
        //Compute the displacement of the holonomic drive, in robot reference frame
        delt_Xr = (dev_m0);
        delt_Yr = (dev_m1 - dev_m2);
        //Move this holonomic displacement from robot to field frame of reference
        robotTheta = getMathHeading();
        sinTheta = Math.sin(robotTheta);
        cosTheta = Math.cos(robotTheta);
        delt_Xf = delt_Xr * cosTheta - delt_Yr * sinTheta;
        delt_Yf = delt_Yr * cosTheta + delt_Xr * sinTheta;
        //Update the position
        X = lastX + delt_Xf;
        Y = lastY + delt_Yf;
        Theta = robotTheta;
        lastM0 = wheel0Pos;
        lastM1 = wheel1Pos;
        lastM2 = wheel2Pos;

        // Drive Train Control
        if (gamepad1.left_stick_y > .1 ||
                gamepad1.left_stick_y < -.1 ||
                gamepad1.left_stick_x > .1 ||
                gamepad1.left_stick_x < -.1 ||
                gamepad1.right_stick_x > .1 ||
                gamepad1.right_stick_x < -.1) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) + rightX;
            double v3 = r * Math.sin(robotAngle) - rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            if (gamepad1.right_stick_x == 0) {
                v1 += v1 / 3;
                v2 += v2 / 3;
                v3 += v3 / 3;
                v4 += v4 / 3;
            }
            Base.drive_by_command(false, -v1, -v2, -v3, -v4);
        } else {
            Base.drive_stop();
        }

        // Collector Controls and Conveyor
        if (base_right_bumper(false)) {
            Collector.collect();
        } else if (base_left_bumper(false)) {
            Collector.eject();
        } else {
            Collector.collect_stop();
        }


        if(gamepad2.right_bumper) {
            if(x == 0) {
                x = 1;
                markedTime2 = runtime.milliseconds();
            }
            Shooter.shoot_by_pd(Subsystem.robot.shoot2.getVelocity(), 1550);
//            Shooter.shoot();
            Shooter.flap_open();
        } else if (gamepad2.left_bumper) {
            Shooter.shoot_by_pd(Subsystem.robot.shoot2.getVelocity(), 1420);
            Shooter.flap_power_shot();
        } else {
            x = 0;
            Shooter.dont_shoot();
            Shooter.flap_close();
        }


        // Conveyor and shooter Controls
        if (gamepad2.right_bumper && Subsystem.robot.shoot2.getVelocity() > 1520 && Subsystem.robot.shoot2.getVelocity() < 1600) {
//        if(gamepad2.right_bumper && runtime.milliseconds() - markedTime2 > 2500) {
            Conveyor.flap_open();
            Conveyor.collect(.7);
        } else if (gamepad2.left_bumper && Subsystem.robot.shoot2.getVelocity() > 1405 && Subsystem.robot.shoot2.getVelocity() < 1440) {
            Conveyor.flap_open();
            Conveyor.collect(.7);
        } else if(gamepad1.left_bumper) {
            Conveyor.flap_open();
            Conveyor.eject();
        } else if (gamepad1.right_bumper) {
            Conveyor.collect(1);
            Conveyor.flap_close();
        }else {
            Conveyor.flap_close();
            Conveyor.conveyor_stop();
        }

        //Wobble controls
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
        Subsystem.set_motor_powers();
        Subsystem.set_servo_positions();

        // Add Telemetry to Phone for Debugging and Testing if it is Activated
        if (gamepad1.start) {
            telemetry.addData("Status: ", "Running Teleop");
            telemetry.addData("Powers: ", "");
            telemetry.addData("LYAXIS: ", gamepad1.left_stick_y);
            telemetry.addData("LXAXIS: ", gamepad1.left_stick_x);
            telemetry.addData("RXAXIS: ", gamepad1.right_stick_x);
            telemetry.addData("Base RightF Power: ", Subsystem.robot.rightF_drive.getPower());
            telemetry.addData("Base RightB Power: ", Subsystem.robot.rightB_drive.getPower());
            telemetry.addData("Base LeftF Power: ", Subsystem.robot.leftF_drive.getPower());
            telemetry.addData("Base LeftB Power: ", Subsystem.robot.leftB_drive.getPower());
            telemetry.update();
        } else {
            // Telemetry Update to Inform Drivers That the Program is Running and how to Access Telemetry
            telemetry.addData("Status: ", "Running Teleop");
            telemetry.addData("Press Start on the Tower Gamepad for Telemetry", "");
            double currentPos = Subsystem.robot.shoot2.getCurrentPosition();
            double linearShootSpeed = previousPos - currentPos;
            previousPos = currentPos;
            telemetry.addData("pidf", Subsystem.robot.shoot2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("DcMotorEx Velocity", Subsystem.robot.shoot2.getVelocity());
            telemetry.addData("Shooter2Power", Subsystem.robot.shoot2.getPower());
            telemetry.addData("Shooter1power", Subsystem.robot.shoot1.getPower());
            telemetry.addData("Shooter2 should be power", (-(Subsystem.robot.shoot2.getVelocity()/5067))+.3);
            telemetry.addData("X", X);
            telemetry.addData("Y", Y);
        }

    }
    // Code to Run Once When the Drivers Press Stop
    public void stop() {
        // Stop Sending Commands to Each Subsystem
        Base.drive_stop();
        Collector.collect_stop();

        // Set Motor Powers and Servos to Their Commands
        Subsystem.set_motor_powers();
        Subsystem.set_servo_positions();

        // Telemetry Update to Inform Drivers That the Program is Stopped
        telemetry.addData("Status: ", "Stopped");
        telemetry.update();

    }

    private double getMathHeading() {

        double x = getIntegratedHeading();
        double test = getIntegratedHeading();
        double angle = 0;
        if(test > 0)
             angle = (x - ((Math.round((x / 360) - .5)) * 360));
        else
            angle = (x - ((Math.round((x / 360) + .5)) * 360));

        return angle;
    }
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
    }

}