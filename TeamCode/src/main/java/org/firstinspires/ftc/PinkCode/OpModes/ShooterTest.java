//package org.firstinspires.ftc.PinkCode.OpModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
//
//@TeleOp(name = "Shooter Flap Test", group = "telop")
//public class ShooterTest extends OpMode {
//    private int scorer_rotate_position = 1;
//
//    @Override
//    public void init() {
//        // Initialization of Each Subsystem's Hardware Map
//        Subsystem.robot.init(hardwareMap);
//        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
////        Scorer.score_rotate_to_position(Presets.SCORER_STOW);
//        // Telemetry Update to Inform Drivers That the Program is Initialized
//        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.a) {
//            Subsystem.robot.scorer_rotate.setPosition(scorer_rotate_position + 1);
//        }
//
//        if (gamepad1.b) {
//            Subsystem.robot.scorer_rotate.setPosition(-scorer_rotate_position + 1);
//        }
//    }
//
//}
