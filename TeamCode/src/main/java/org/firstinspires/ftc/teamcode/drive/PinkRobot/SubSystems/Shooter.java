package org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems;

import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.PD;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations.Presets;

// Abstract Class to Define the Methods of the Collector Subsystem
public abstract class Shooter extends PinkSubsystem {

    // Method for shoot
    public static void shoot_by_pd(double current, double target) {
        // Define Commands
        shooter_command1 = PD.ShooterPD(current, target);
        shooter_command2 = shooter_command1;
    }

    //Method for shoot
    public static void shoot_by_PIDF(double velocity) {
        shooter_command1 = PinkSubsystem.robot.shoot2.getPower();
        shooter_command2 = shooter_command1;
    }

    public static void shoot(){
        shooter_command1 = Presets.SHOOTER_SHOOT_POWER;
        shooter_command2 = Presets.SHOOTER_SHOOT_POWER;
    }

    public static void shootPower(double power){
        shooter_command1 = power;
        shooter_command2 = power;
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

    public static void flap_power_shot() {
        shooter_flap_command = Presets.SHOOTER_FLAP_POWER_SHOT;
    }

    public static void flap_custom(double flap_pos) {
        shooter_flap_command = flap_pos;
    }


    public static void flap_close() {
        shooter_flap_command = Presets.SHOOTER_FLAP_CLOSE;
    }
}