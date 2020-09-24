package org.firstinspires.ftc.PinkCode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Class to Define the Hardware of the Robot
public class Hardware {
    // Motors
    public DcMotor rightF_drive; // Port 3 Control Hub
    public DcMotor rightB_drive; // Port 2 Control Hub
    public DcMotor leftF_drive; // Port 0 Control Hub
    public DcMotor leftB_drive; // Port 1 Control Hub
    public DcMotor collect;
    public DcMotor conveyor;
    public DcMotor shoot1;
    public DcMotor shoot2;

    public DcMotorEx encoder_left; // Port Unknown
    public DcMotorEx encoder_right; // Port Unknown
    public DcMotorEx encoder_center; // Port Unknown

    // Servos
    //public Servo scorer_rotate;

    // Local OpMode Members
    private HardwareMap hwMap = null;

    // Method Called When Referencing Robot Hardware in Subsystems
    public void init (HardwareMap ahwMap) {
        // Reference to Hardware Map
        hwMap = ahwMap;

        // Motors
        rightF_drive = hwMap.get(DcMotor.class, "rightF_drive");
        rightB_drive = hwMap.get(DcMotor.class, "rightB_drive");
        leftF_drive = hwMap.get(DcMotor.class, "leftF_drive");
        leftB_drive = hwMap.get(DcMotor.class, "leftB_drive");
//        collect = hwMap.get(DcMotor.class, "collect_left");
//        conveyor = hwMap.get(DcMotor.class, "conveyor");
        shoot1 = hwMap.get(DcMotor.class, "shoot1");
        shoot2 = hwMap.get(DcMotor.class, "shoot2");

        // Odometry Encoders
        encoder_center = hwMap.get(DcMotorEx.class, "encoder_center");
        encoder_left = hwMap.get(DcMotorEx.class, "encoder_left");
        encoder_right = hwMap.get(DcMotorEx.class, "encoder_right");

        // Motor Configuration
        rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        collect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        collect.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightF_drive.setDirection(DcMotor.Direction.REVERSE);
        rightB_drive.setDirection(DcMotor.Direction.REVERSE);
        leftF_drive.setDirection(DcMotor.Direction.FORWARD);
        leftB_drive.setDirection(DcMotor.Direction.FORWARD);
//        collect.setDirection(DcMotor.Direction.FORWARD);
//        conveyor.setDirection(DcMotor.Direction.FORWARD);
        shoot2.setDirection(DcMotor.Direction.FORWARD);
        shoot2.setDirection(DcMotor.Direction.FORWARD);

        rightF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        collect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightF_drive.setPower(0);
        rightB_drive.setPower(0);
        leftF_drive.setPower(0);
        leftB_drive.setPower(0);
//        collect.setPower(0);
//        conveyor.setPower(0);
        shoot1.setPower(0);
        shoot2.setPower(0);


        encoder_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Servos
        //scorer_rotate = hwMap.get(Servo.class, "scorer_rotate");
    }
}