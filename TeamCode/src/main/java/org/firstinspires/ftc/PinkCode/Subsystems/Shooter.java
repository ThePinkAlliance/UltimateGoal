package org.firstinspires.ftc.PinkCode.Subsystems;


import org.firstinspires.ftc.PinkCode.Calculations.Presets;

// Abstract Class to Define the Methods of the Collector Subsystem
public abstract class Shooter extends Subsystem{

    // Method for shoot
    public static void shoot() {
        // Define Commands
        shooter_command1 = Presets.SHOOTER_SHOOT_POWER;
        shooter_command2 = Presets.SHOOTER_SHOOT_POWER;
    }

    // Method for stop
    public static void dont_shoot() {
        // Define Commands
        shooter_command1 = Presets.SHOOTER_STOP_POWER;
        shooter_command2 = Presets.SHOOTER_STOP_POWER;
    }

    public static void flap_open() {
        shooter_flap_command = Presets.SHOOTER_FLAP_OPEN;
    }

    public static void flap_close() {
        shooter_flap_command = Presets.SHOOTER_FLAP_CLOSE;
    }
}