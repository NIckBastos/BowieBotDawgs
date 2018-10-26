package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

/**
 * Created by Vince on 10/10/2016.
 */

//@Autonomous(name="OpenCV Linear Opmode", group="Vision")
public class OpenCVLinearOpModeBase extends LinearOpMode implements  CameraBridgeViewBase.CvCameraViewListener2{
    CameraManager cameraManager;
    ImageProcessor imageProcessor;
    public enum AllianceColor {
        RED,
        BLUE,
    }

    // default to blue for testing
    public OpenCVLinearOpModeBase() {
        this(AllianceColor.BLUE);
    }

    public OpenCVLinearOpModeBase(AllianceColor allianceColor) {
        super();
        cameraManager = new CameraManager();
        imageProcessor = new ImageProcessor(allianceColor);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Context context = hardwareMap.appContext;
        //ALL OF imageProccessor is code for beacon, what can I delete. There is a lot of stuff
//        and I dont know what is necessary and what I can delete since I am reusing code
        cameraManager.initialize(context, true, this);
        imageProcessor.initialize();

        waitForStart();

        while (opModeIsActive()) {
            //How does .takePicture() give me the value of LEFT, MID, or RIGHT?
            imageProcessor.takePicture();
            while(opModeIsActive() && !imageProcessor.isFrameReady()){
                idle();
            }

        }
        cameraManager.stop(context);
        imageProcessor.stop();
//        public static void min(double a, double b, double c) {
//            lowestBlue = Math.min(Math.min(a, b), c);
//        }
        //Here add the three paths

    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        cameraManager.start(width, height);
        Log.d("CAMERA", "STARTED");
    }

    @Override
    public void onCameraViewStopped() {
        Log.d("CAMERA", "STOPPED");
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat retval = imageProcessor.processFrame(inputFrame);
        if (retval != null) return retval;
        return inputFrame.rgba();
    }
}
