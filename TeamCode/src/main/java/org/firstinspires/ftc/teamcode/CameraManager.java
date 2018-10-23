package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.hardware.Camera;
import android.util.Log;
import android.widget.LinearLayout;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Vince on 10/9/2016.
 */

public class CameraManager {
//    private static final int initialMaxSize = 1200;
    public static MyJavaCameraView openCVCamera;
    private static boolean initialized = false;
    private static boolean openCVInitialized = false;
    private static boolean viewRemovalDone = false;
    private LinearLayout cameraMonitorView;

//    final static int CameraSide = Camera.CameraInfo.CAMERA_FACING_FRONT;
    final static int CameraSide = Camera.CameraInfo.CAMERA_FACING_BACK;

    public int width, height;

    public CameraManager() {
        initialized = false;
        openCVCamera = null;
        cameraMonitorView = null;
    }

    public void initialize(Context appContext, final boolean useFullResolution, CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        //Initialize camera view
        BaseLoaderCallback openCVLoaderCallback = null;
        final Activity activity = (Activity) appContext;
        final CameraBridgeViewBase.CvCameraViewListener2 mCameraViewListener = cameraViewListener;

        if (!openCVInitialized) {
            try {
                openCVLoaderCallback = new BaseLoaderCallback(appContext) {
                    @Override
                    public void onManagerConnected(int status) {
                        switch (status) {
                            case LoaderCallbackInterface.SUCCESS: {
                                Log.d("OpenCV", "OpenCV Manager connected!");
                                openCVInitialized = true;
                            }
                            break;
                            default: {
                                super.onManagerConnected(status);
                            }
                            break;
                        }
                    }
                };
            } catch (NullPointerException e) {
                Log.e("CameraManager", "Could not find OpenCV Manager\r\n");
            }


            if (!OpenCVLoader.initDebug()) {
                Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
                boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, appContext, openCVLoaderCallback);
                if (!success) {
                    Log.e("OpenCV", "Asynchronous initialization failed!");
                } else {
                    Log.d("OpenCV", "Asynchronous initialization succeeded!");
                }
            } else {
                Log.d("OpenCV", "OpenCV library found inside package. Using it!");
                if (openCVLoaderCallback != null)
                    openCVLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
                else {
                    Log.e("OpenCV", "Failed to load OpenCV from package!");
                    return;
                }
            }

            while (!openCVInitialized) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        if (openCVCamera == null) {
            initialized = false;
            activity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    cameraMonitorView = (LinearLayout) activity.findViewById(R.id.cameraMonitorViewId);
                    openCVCamera = new MyJavaCameraView(activity, CameraSide);
                    openCVCamera.setFullResolution(useFullResolution);

                    cameraMonitorView.addView(openCVCamera, new LinearLayout.LayoutParams(
                            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.MATCH_PARENT
                    ));
                    openCVCamera.disableView();
                    openCVCamera.setCameraIndex(CameraSide);
                    openCVCamera.enableView();

                    openCVCamera.setCvCameraViewListener(mCameraViewListener);
                    initialized = true;
                }
            });

        while (!initialized) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        } else {
            openCVCamera.connectCamera(width, height);
            openCVCamera.enableView();
        }
    }

    public void start(int width, int height) {
        this.width = width;
        this.height = height;

    }

    public void stop(Context appContext) {
        if (openCVCamera != null) {
            openCVCamera.disableView();
            openCVCamera.disconnectCamera();
        }

        if (cameraMonitorView != null) {
            viewRemovalDone = false;
            final Activity activity = (Activity) appContext;
            activity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    cameraMonitorView.removeAllViews();
                    viewRemovalDone = true;
                }
            });

            while (!viewRemovalDone) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        initialized = false;
        openCVCamera = null;

    }

    public Mat previewRotate(Mat src) {
        Mat dst = src.t();
        Core.flip(src.t(),dst,1);
        Imgproc.resize(dst,dst,src.size());
        return dst;
    }

}
