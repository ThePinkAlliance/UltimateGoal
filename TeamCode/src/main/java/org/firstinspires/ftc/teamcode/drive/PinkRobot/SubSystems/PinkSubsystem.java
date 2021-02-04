package org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems;

import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.Presets;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.PinkHardware;

// Class Which Defines Variables Used in Other Subsystems and Sets Powers and Commands for Teleop/Auto
public class PinkSubsystem {
    // Define Class Members
    public static PinkHardware robot = new PinkHardware();
    static double front_left_wheel_command;
    static double back_left_wheel_command;
    static double front_right_wheel_command;
    static double back_right_wheel_command;
    static double collect_command;
    static double conveyor_command;
    static double shooter_command1;
    static double shooter_command2;
    static double conveyor_flap_command = Presets.CONVEYOR_FLAP_OPEN;
    static double shooter_flap_command = Presets.SHOOTER_FLAP_OPEN;
    static double collector_drop_command = Presets.COLLECTOR_HOLDER_HOLD;
    static double wobble_arm_command = Presets.WOBBLE_UP;
    static double wobble_grip_command = Presets.WOBBLE_GRIP;

    // Method Which Sends the Motor Powers to the Motors
    public static void set_motor_powers() {
        // Set Motor Powers
        robot.rightF_drive.setPower(front_right_wheel_command);
        robot.rightB_drive.setPower(back_right_wheel_command);
        robot.leftF_drive.setPower(front_left_wheel_command);
        robot.leftB_drive.setPower(back_left_wheel_command);
        robot.collect.setPower(collect_command);
        robot.conveyor.setPower(conveyor_command);
        robot.shoot1.setPower(shooter_command1);
        robot.shoot2.setPower(shooter_command2);
    }

    // Method Which Sends the Motor Powers to the Motors
    public static void set_motor_powers_no_base() {
        // Set Motor Powers
        robot.collect.setPower(collect_command);
        robot.conveyor.setPower(conveyor_command);
        robot.shoot1.setPower(shooter_command1);
        robot.shoot2.setPower(shooter_command2);
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