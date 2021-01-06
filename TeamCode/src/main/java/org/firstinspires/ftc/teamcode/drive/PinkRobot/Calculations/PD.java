package org.firstinspires.ftc.teamcode.drive.PinkRobot.Calculations;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;

// Abstract Class to Find the Motor Commands for Each Subsystem Using a PD
public abstract class PD {
    // Use a PD to Determine the Lift Motor Command

    public static double ShooterPD (double currentVel, double targetVel) {
        double motorCmd;
        double error = targetVel - currentVel;
        double diff = 1 - (targetVel/2420); //2420 prev.
        double kp = -(currentVel/(targetVel*(1/diff)))+diff;
        double kd = (currentVel/targetVel)-1;
        motorCmd = kp + (targetVel/2420) - kd;
        motorCmd = Range.clip(motorCmd, -1.0, 1.0);

        return motorCmd;
    }

    public static double getMotorCmd (double Kp, double Kd, double error, double currentVel) {
        double motorCmd;
        motorCmd = (Kp * error) - (Kd * currentVel);
        return motorCmd;
    }
}