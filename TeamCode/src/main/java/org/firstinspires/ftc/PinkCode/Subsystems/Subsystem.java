package org.firstinspires.ftc.PinkCode.Subsystems;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
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
    static double shoot_command;

    // Method Which Sends the Motor Powers to the Motors
    public static void set_motor_powers() {
        // Set Motor Powers
        robot.rightF_drive.setPower(Subsystem.front_right_wheel_command);
        robot.rightB_drive.setPower(Subsystem.back_right_wheel_command);
        robot.leftF_drive.setPower(Subsystem.front_left_wheel_command);
        robot.leftB_drive.setPower(Subsystem.back_left_wheel_command);
//        robot.collect.setPower(Subsystem.collect_command);
//        robot.conveyor.setpower(Subsystem.conveyor_command);
    }

    // Method Which Sends the Servo Positions to the Servos
    public static void set_servo_positions() {
        // Set Servo Positions
//        robot.scorer_rotate.setPosition(score_target_position);
        robot.collectorHolder.setPosition(Presets.COLLECTOR_HOLDER_INIT_POSITION);
        robot.conveyor_regulator.setPosition(0);
    }

}