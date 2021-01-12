package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Collector;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Base;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Controls;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.Wobble;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

// Class for Player-Controlled Period of the Game Which Binds Controls to Subsystems
@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends Controls {

    double markedTime2 = 0;
    int x = 0;
    double sinTheta;
    double backCoderAngleNeg;
    double cosTheta;
    double previousPos;
    double X, Y, backCoder, leftCoder, rightCoder, theta, thetaDiff, preX, preY, thetaTest;
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
        PinkSubsystem.robot.init(hardwareMap);
        PinkSubsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinkSubsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PinkSubsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PinkSubsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PinkSubsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Wobble.wobble_arm_up();
        Wobble.wobble_grip();
//        Scorer.score_rotate_to_position(Presets.SCORER_STOW);
        // Telemetry Update to Inform Drivers That the Program is Initialized
        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
        telemetry.update();
    }

    // Code to Run Constantly After the Drivers Press Play and Before They Press Stop
    public void loop() {

        if (PinkSubsystem.robot.collector_drop.getPosition() == 0) {
            Collector.collector_drop();
        }

        backCoder = -PinkSubsystem.robot.rightF_drive.getCurrentPosition();
        leftCoder = PinkSubsystem.robot.rightB_drive.getCurrentPosition();
        rightCoder = PinkSubsystem.robot.leftF_drive.getCurrentPosition();
        thetaTest = ((((rightCoder - leftCoder) / 2) / 153.5) * Math.PI)/180;
        backCoderAngleNeg = (((rightCoder - leftCoder) / 2) / 153.5) * 105.5;
        thetaDiff = (rightCoder - leftCoder) / 2;
        cosTheta = Math.cos(thetaTest);
        sinTheta = Math.sin(thetaTest);
        preX = backCoder - backCoderAngleNeg;
        preY = ((rightCoder + leftCoder) / 2) - thetaDiff;

        X = (preX * cosTheta) + (preY * sinTheta);
        Y = (preY * cosTheta) + (preX * sinTheta);
        X = X/1125;
        Y = Y/1125;



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

        // Collector Controls and Conveyor.
        if (base_right_bumper(false)) {
            Collector.collect();
        } else if (base_left_bumper(false)) {
            Collector.eject();
        } else {
            Collector.collect_stop();
        }


        if(gamepad2.right_bumper || (gamepad2.right_trigger >= 0.2)) {
//            if(x == 0) {
//                x = 1;
//                markedTime2 = runtime.milliseconds();
//            }
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), 1550);
//            Shooter.shoot();
            Shooter.flap_open();
        } else if (gamepad2.left_bumper) {
            Shooter.shoot_by_pd(PinkSubsystem.robot.shoot2.getVelocity(), 1420);
//            Shooter.shoot();
            Shooter.flap_power_shot();
        } else {
            x = 0;
//            Shooter.flap_close();
            Shooter.dont_shoot();
        }


        // Conveyor and shooter Controls
        if (gamepad2.right_bumper && PinkSubsystem.robot.shoot2.getVelocity() > 1450 && PinkSubsystem.robot.shoot2.getVelocity() < 1620 && x != 1) {
            if (gamepad2.right_trigger >= 0.2) {
                x = 1;
            }
//            Conveyor.flap_open();
//            Conveyor.collect(.7);
//        if(gamepad2.right_bumper && runtime.milliseconds() - markedTime2 > 2500) {
        } else if (gamepad2.left_bumper && PinkSubsystem.robot.shoot2.getVelocity() > 1350 && PinkSubsystem.robot.shoot2.getVelocity() < 1500) {
            Conveyor.flap_open();
            Conveyor.collect(.7);
        } else if (x == 1) {
            Conveyor.flap_open();
            Conveyor.collect(.85);
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
        PinkSubsystem.set_motor_powers();
        PinkSubsystem.set_servo_positions();

        // Add Telemetry to Phone for Debugging and Testing if it is Activated
        if (gamepad1.start) {
            telemetry.addData("Status: ", "Running Teleop");
            telemetry.addData("Powers: ", "");
            telemetry.addData("LYAXIS: ", gamepad1.left_stick_y);
            telemetry.addData("LXAXIS: ", gamepad1.left_stick_x);
            telemetry.addData("RXAXIS: ", gamepad1.right_stick_x);
            telemetry.addData("Base RightF Power: ", PinkSubsystem.robot.rightF_drive.getPower());
            telemetry.addData("Base RightB Power: ", PinkSubsystem.robot.rightB_drive.getPower());
            telemetry.addData("Base LeftF Power: ", PinkSubsystem.robot.leftF_drive.getPower());
            telemetry.addData("Base LeftB Power: ", PinkSubsystem.robot.leftB_drive.getPower());
            telemetry.update();
        } else {
            // Telemetry Update to Inform Drivers That the Program is Running and how to Access Telemetry
            telemetry.addData("Status: ", "Running Teleop");
            telemetry.addData("Press Start on the Tower Gamepad for Telemetry", "");
            double currentPos = PinkSubsystem.robot.shoot2.getCurrentPosition();
            double linearShootSpeed = previousPos - currentPos;
            previousPos = currentPos;
            telemetry.addData("pidf", PinkSubsystem.robot.shoot2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("DcMotorEx Velocity", PinkSubsystem.robot.shoot2.getVelocity());
            telemetry.addData("Shooter2Power", PinkSubsystem.robot.shoot2.getPower());
            telemetry.addData("Shooter1power", PinkSubsystem.robot.shoot1.getPower());
            telemetry.addData("X", X);
            telemetry.addData("Y", Y);
            telemetry.addData("ImuHeading", getIntegratedHeading());
            telemetry.addData("MathHeading", thetaTest);
            telemetry.addData("getMathHeadingTets", theta);
            telemetry.addData("MathHeadingCounts", (rightCoder - leftCoder) / 2);
            telemetry.addData("BackCoder Counts", backCoder);
            telemetry.addData("x:", x);
        }

    }
    // Code to Run Once When the Drivers Press Stop
    public void stop() {
        // Stop Sending Commands to Each Subsystem
        Base.drive_stop();
        Collector.collect_stop();

        // Set Motor Powers and Servos to Their Commands
        PinkSubsystem.set_motor_powers();
        PinkSubsystem.set_servo_positions();

        // Telemetry Update to Inform Drivers That the Program is Stopped
        telemetry.addData("Status: ", "Stopped");
        telemetry.update();

    }

    private double getMathHeading(double x) {

        double angle = 0;
        if(x > 0)
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