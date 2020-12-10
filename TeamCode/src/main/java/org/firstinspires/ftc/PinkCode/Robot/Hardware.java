package org.firstinspires.ftc.PinkCode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// Class to Define the Hardware of the Robot
public class Hardware {
    // Motors
    public DcMotorEx rightF_drive; // Port 3 Expan Hub
    public DcMotorEx rightB_drive; // Port 2 Expan Hub
    public DcMotorEx leftF_drive; // Port 0 Expan Hub
    public DcMotorEx leftB_drive; // Port 1 Expan Hub
    public DcMotor collect; // Port _ Control Hub
    public DcMotor conveyor; // Port _ Control Hub
    public DcMotor shoot1; // Port _ Control Hub
    public DcMotorEx shoot2; // Port _ Control Hub

    public Encoder encoder_left; // unknown
    public Encoder encoder_right; // RB
    public Encoder encoder_center; // RF

    public WebcamName webcam; // Port Unknown

    // Servos
    public Servo shoot_flap;
    public Servo conveyor_flap;
    public Servo collector_drop;
    public Servo wobble_arm;
    public Servo wobble_grip;

    // Local OpMode Members
    private HardwareMap hwMap = null;

    // Method Called When Referencing Robot Hardware in Subsystems
    public void init (HardwareMap ahwMap) {
        // Reference to Hardware Map
        hwMap = ahwMap;

        // Motors
        rightF_drive = hwMap.get(DcMotorEx.class, "rightF_drive");
        rightB_drive = hwMap.get(DcMotorEx.class, "rightB_drive");
        leftF_drive = hwMap.get(DcMotorEx.class, "leftF_drive");
        leftB_drive = hwMap.get(DcMotorEx.class, "leftB_drive");
        collect = hwMap.get(DcMotor.class, "collector");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
        shoot1 = hwMap.get(DcMotor.class, "shoot1");
        shoot2 = hwMap.get(DcMotorEx.class, "shoot2");

        // Odometry Encoders
        
        // Port for the center encoder
        encoder_center = new Encoder(hwMap.get(DcMotorEx.class, "rightF_drive"));

        // Port for the left encoder
        encoder_left = new Encoder(hwMap.get(DcMotorEx.class, "rightB_drive"));

        // Port for the right encoder
        encoder_right = new Encoder(hwMap.get(DcMotorEx.class, "leftF_drive"));

        webcam = hwMap.get(WebcamName.class, "webcam");

        // Motor Configuration
        rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collect.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightF_drive.setDirection(DcMotor.Direction.REVERSE);
        rightB_drive.setDirection(DcMotor.Direction.REVERSE);
        leftF_drive.setDirection(DcMotor.Direction.FORWARD);
        leftB_drive.setDirection(DcMotor.Direction.FORWARD);
        collect.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.FORWARD);
        shoot1.setDirection(DcMotor.Direction.FORWARD);
        shoot2.setDirection(DcMotor.Direction.FORWARD);

        rightF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightF_drive.setPower(0);
        rightB_drive.setPower(0);
        leftF_drive.setPower(0);
        leftB_drive.setPower(0);
        collect.setPower(0);
        conveyor.setPower(0);
        shoot1.setPower(0);
        shoot2.setPower(0);

        shoot2.setVelocityPIDFCoefficients(1.2,.12,0,12.0);


        // Servos
        shoot_flap = hwMap.get(Servo.class, "shoot_flap");
        collector_drop = hwMap.get(Servo.class, "collector_drop");
        conveyor_flap = hwMap.get(Servo.class, "conveyor_flap");
        wobble_arm = hwMap.get(Servo.class, "wobble_arm");
        wobble_grip = hwMap.get(Servo.class, "wobble_grip");

    }
}