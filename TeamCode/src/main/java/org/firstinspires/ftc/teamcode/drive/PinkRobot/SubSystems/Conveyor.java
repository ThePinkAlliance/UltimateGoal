package org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems;

import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.Presets;

// Abstract Class to Define the Methods of the Collector Subsystem
public abstract class Conveyor extends PinkSubsystem{

    // Method for Collecting
    public static void collect(double power) {
        // Define Commands
        conveyor_command = power;
    }

    public static void Regulator_on() {
//        robot.conveyor_regulator.setPosition(0);
    }

//    public static void Regulator_off() {
//        robot.conveyor_regulator.setPosition(30);
//    }

    // Method for Ejecting
    public static void eject() {
        // Define Commands
        conveyor_command = Presets.CONVEYOR_EJECT_POWER;
    }

    // Method for Stopping the Collector
    public static void conveyor_stop() {
        // Define Commands
        conveyor_command = Presets.CONVEYOR_STOP_POWER;
    }

    public static void flap_open() {
        //Define Commands
        conveyor_flap_command = Presets.CONVEYOR_FLAP_OPEN;
    }

    public static void flap_close() {
        //Define Commands
        conveyor_flap_command = Presets.CONVEYOR_FLAP_CLOSE;
    }

    public static void top_gate_down() {
        top_gate_command = Presets.TOP_GATE_DOWN;
    }

    public static void top_gate_up() {
        top_gate_command = Presets.TOP_GATE_UP;
    }
}