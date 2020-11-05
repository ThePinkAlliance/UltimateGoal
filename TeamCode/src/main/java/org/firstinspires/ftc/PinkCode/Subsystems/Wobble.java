package org.firstinspires.ftc.PinkCode.Subsystems;


import org.firstinspires.ftc.PinkCode.Calculations.Presets;

// Abstract Class to Define the Methods of the Collector Subsystem
public abstract class Wobble extends Subsystem{

    // Method for shoot
    public static void wobble_arm_up() {
        // Define Commands
        wobble_arm_command = Presets.WOBBLE_UP;
    }

    // Method for stop
    public static void wobble_arm_down() {
        // Define Commands
        wobble_arm_command = Presets.WOBBLE_DOWN;
    }

    public static void wobble_grip() {
        wobble_grip_command = Presets.WOBBLE_GRIP;
    }

    public static void wobble_ungrip() {
        wobble_grip_command = Presets.WOBBLE_UNGRIP;
    }

}