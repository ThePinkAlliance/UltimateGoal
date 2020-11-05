package org.firstinspires.ftc.PinkCode.Calculations;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;

// Abstract Class to Find the Motor Commands for Each Subsystem Using a PD
public abstract class PD {
    // Use a PD to Determine the Lift Motor Command

    public static double ShooterPD (double currentVel, double targetVel)
    {
        double motorCmd;
        double error = targetVel - currentVel;
        double diff = 1 - (targetVel/2090);
        double kp = -(currentVel/(targetVel*(1/diff)))+diff;
        double kd = (currentVel/targetVel)-1;
//        motorCmd = ((-(Subsystem.robot.shoot2.getVelocity()/5067))+.3) + .7;
        motorCmd = kp + (targetVel/2090) - kd;
        motorCmd = Range.clip(motorCmd, -1.0, 1.0);

        return motorCmd;
    }
}