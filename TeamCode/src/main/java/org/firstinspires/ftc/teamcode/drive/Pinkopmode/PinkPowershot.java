package org.firstinspires.ftc.teamcode.drive.Pinkopmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.PinkRobot.SubSystems.PinkSubsystem;

import java.util.List;

@Autonomous(name = "Powershot Auto")
public class PinkPowershot extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "pink.tflite";
    private static final String LABEL_FIRST_ELEMENT = "powershot";
    private static final String LABEL_SECOND_ELEMENT = "obj";

    private TFObjectDetector tf;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "AU5HdoL/////AAABmdflEYY1uEgKvLLnXhuUKQEiOh/Swf8w1NP3fjwJ0L5KhNZjEBmtqvcb1vRriuL7dxpTimmKsrPxVN0GSemDm1z0zZHiuEDJjN6is0gE5cC8eCf5/w4A9J9xygAQMiK4UOje3lWQjKpyMbqNeKgy1I6PZqyXBae1+6/gecIRmHuDjcqGFcEnRKmf8e6iPrFIdaC53DkmQUxJWRalVEqWsdmwmLm69AsaoG+aL7D0xkupVo7U23C2fdDkl66qsFO7v7jf0ONGEdmNjg1TTEKQmrip86/iMst+I7mdLA/pYsY00EjAjgPJ8YdXEqR5pKR2CK4DNmVU+c2A7T+w+KhGwxJ8us9j9FpYTd1yC0wRQD0R";


    @Override
    public void runOpMode() throws InterruptedException {
        if (tf != null) {
            tf.activate();
            telemetry.addData("tf", "ready");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (tf != null) {
                List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (Recognition r: updatedRecognitions) {
                        telemetry.addData("powershots", r.getLabel());
                    }
                    telemetry.addData("updatedRecognitions", updatedRecognitions.toArray());
                    telemetry.update();
                }
            }
        }
    }

    private void initTfod(FtcDashboard dash) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        dash.startCameraStream(tf, 60);
        tfodParameters.minResultConfidence = 0.6f;
        tf = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tf.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = PinkSubsystem.robot.webcam;


        ClassFactory.getInstance().getCameraManager().nameForUnknownCamera();

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}
