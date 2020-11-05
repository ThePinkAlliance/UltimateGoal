package org.firstinspires.ftc.PinkCode.Subsystems;

import org.firstinspires.ftc.PinkCode.OpModes.Teleop;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;

// Class Which Defines Variables Used in Other Subsystems and Sets Powers and Commands for Teleop/Auto
public abstract class Subsystem extends Teleop {
    // Define Class Members
    public static Hardware robot = new Hardware();
    static double front_left_wheel_command;
    static double back_left_wheel_command;
    static double front_right_wheel_command;
    static double back_right_wheel_command;
    static double collect_command;
    static double conveyor_command;
    static double shooter_command1;
    static double shooter_command2;
    static double conveyor_flap_command;
    static double shooter_flap_command;
    static double collector_drop_command;
    static double wobble_arm_command;
    static double wobble_grip_command;

    // Method Which Sends the Motor Powers to the Motors
    public static void set_motor_powers() {
        // Set Motor Powers
        robot.rightF_drive.setPower(Subsystem.front_right_wheel_command);
        robot.rightB_drive.setPower(Subsystem.back_right_wheel_command);
        robot.leftF_drive.setPower(Subsystem.front_left_wheel_command);
        robot.leftB_drive.setPower(Subsystem.back_left_wheel_command);
        robot.collect.setPower(Subsystem.collect_command);
        robot.conveyor.setPower(Subsystem.conveyor_command);
        robot.shoot1.setPower(Subsystem.shooter_command1);
        robot.shoot2.setPower(Subsystem.shooter_command2);
    }

    // Method Which Sends the Servo Positions to the Servos
    public static void set_servo_positions() {
        // Set Servo Positions
//        robot.scorer_rotate.setPosition(score_target_position);
        robot.shoot_flap.setPosition(shooter_flap_command);
        robot.conveyor_flap.setPosition(conveyor_flap_command);
        robot.collector_drop.setPosition(collector_drop_command);
        robot.wobble_arm.setPosition(wobble_arm_command);
        robot.wobble_grip.setPosition(wobble_grip_command);

    }

}