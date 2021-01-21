package org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems;

import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.Presets;

// Abstract Class to Define the Methods of the Collector Subsystem
public abstract class Collector extends PinkSubsystem{

    // Method for Collecting
    public static void collect() {
        // Command Assigned to preset value
        collect_command = Presets.COLLECTOR_COLLECT_POWER;
    }

    // Method for Ejecting
    public static void eject() {
        // Command assigned to preset value
        collect_command = Presets.COLLECTOR_EJECT_POWER;
    }

    // Method for Stopping the Collector
    public static void collect_stop() {
        // Command assigned to preset value
        collect_command = Presets.COLLECTOR_STOP_POWER;
    }

    public static void collector_hold() {
        // Command assigned to preset value
        collector_drop_command = Presets.COLLECTOR_HOLDER_HOLD;
    }

    public static void collector_drop() {
        // Command assigned to preset value
        collector_drop_command = Presets.COLLECTOR_HOLDER_RELEASE;
    }
}