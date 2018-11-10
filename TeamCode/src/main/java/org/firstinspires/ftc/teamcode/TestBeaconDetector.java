package org.firstinspires.ftc.teamcode;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Vince on 11/9/2016.
 */

@Autonomous(name="Test Beacon Detector", group="Autonomous")

public class TestBeaconDetector extends OpenCVLinearOpModeBase {
    //Declare data logger
    private DataLogger dl;

    final static boolean useFullRes = true;
    AllianceColor allianceColor;

    public long startTime; // Define robot start time
    public int heading1 = 46; // Turn towards blue line (changed 10/8)
    public TestBeaconDetector(){
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Context context = hardwareMap.appContext;
        cameraManager.initialize(context, useFullRes, this);
        imageProcessor.initialize(useFullRes, this, true);
        imageProcessor.setAlliance(allianceColor);


        //Setup robot motor
//        motorController = new MotorMethods(hardwareMap);
        //Setup data logger

        waitForStart();
        startTime = System.currentTimeMillis();

        //Pivot right with left motors
  //      motorController.pivotLeftEncoders(heading1, 1, telemetry, dl);

//        imageProcessor.takePicture();
        while (opModeIsActive()) {
            imageProcessor.takePicture();
            sleep(50);
        }

        cameraManager.stop(context);
        imageProcessor.stop();


    }
}
