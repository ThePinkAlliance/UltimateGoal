package org.firstinspires.ftc.PinkCode.Subsystems;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;

public abstract class Shooter extends Subsystem {
    public static void Shoot() {
        robot.shoot1.setPower(Presets.MAX_SHOOTER_POWER);
    }

    public static void Stop() {
        robot.shoot1.setPower(0);
    }
}
