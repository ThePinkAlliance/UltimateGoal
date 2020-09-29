package org.firstinspires.ftc.PinkCode.Subsystems;


import org.firstinspires.ftc.PinkCode.Calculations.Presets;

// Abstract Class to Define the Methods of the Collector Subsystem
public abstract class Conveyor extends Subsystem{

    // Method for Collecting
    public static void collect() {
        // Define Commands
        conveyor_command = Presets.CONVEYOR_COLLECT_POWER;
    }

    // Method for Ejecting
    public static void eject() {
        // Define Commands
        conveyor_command = Presets.CONVEYOR_EJECT_POWER;
    }

    // Method for Stopping the Collector
    public static void collect_stop() {
        // Define Commands
        collect_command = 0;
    }
}