package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Created by Vince on 11/3/2016.
 */

public class ImageProcessor {
    /*
     * Flag indicating whether to process the next available frame.  No need to waste
     * cycles looking at every frame.
     */
    private volatile boolean processNextFrame;

    /*
     * Flag indicating whether the most recently requested frame has been processed.
     */
    private volatile boolean frameReady;
    OpenCVLinearOpModeBase op;
    public enum State {
        RED_IS_LEFT,
        RED_IS_RIGHT,
        BLUE_IS_LEFT,
        BLUE_IS_RIGHT,
        UNKNOWN,
    }

    private State beacon1State;
    private State beacon2State;

    int redPixels;
    double midHValue;
    double midSValue;
    double midVValue;

    public boolean isFrameReady(){
        return frameReady;
    }
    final static boolean useFullScreen = false;


    final static int BEACONWIDTH = 50;
    final static int BEACONHEIGHT = 50;
    private OpenCVLinearOpModeBase.AllianceColor allianceColor;
    private Point beacon1Location;
    private Point beacon2Location;
    private Point beacon1Size;
    private Point beacon2Size;
    final static Point RED_BEACON1_LOCATION = new Point(943,512);//(931, 488);//(964, 458);//(811, 333);
    final static Point RED_BEACON1_SIZE = new Point(60,45);
    final static Point RED_BEACON2_LOCATION = new Point(596, 505);//(565, 459);//624,431); //(463, 301);
    final static Point RED_BEACON2_SIZE = new Point(30,30);
    final static Point BLUE_BEACON1_LOCATION = new Point(258,510);//(205, 510);//(249, 501);//(262, 484);//(102, 355);
    final static Point BLUE_BEACON1_SIZE = new Point(70,40); //70, 40
    final static Point BLUE_BEACON2_LOCATION = new Point(623, 491);//(582, 484);//(606, 479);//(617, 453);//(480, 323);
    final static Point BLUE_BEACON2_SIZE = new Point(30,30);


    Point largestBlue = null;
    Point largestRed = null;

    Point beacon1r = null;
    Point beacon1b = null;
    Point beacon2r = null;
    Point beacon2b = null;

    Point blueCenter1 = null;
    Point redCenter1 = null;
    Point blueCenter2 = null;
    Point redCenter2 = null;

    private Mat blue1;
    private Mat redL1;
    private Mat redH1;
    private Mat red1;
    private Mat leftMineralMat = new Mat();
    private Mat midMineralMat = new Mat();
    private Mat rightMineralMat = new Mat();

    public double leftMineralBlue = 0.0;
    public double midMineralBlue= 0.0;
    public double rightMineralBlue = 0.0;


    double WIDTH_SCALING;
    double HEIGHT_SCALING;

    boolean continuous = false;

    Point textPoint;
    //private int beaconDist = 150;
    public static int xb;
    public static int yb;
    public static int xr;
    public static int yr;
    public static int minArea = 0;

    public ImageProcessor(OpenCVLinearOpModeBase.AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE) {
            beacon1Location = BLUE_BEACON1_LOCATION;
            beacon2Location = BLUE_BEACON2_LOCATION;
            beacon1Size = BLUE_BEACON1_SIZE;
            beacon2Size = BLUE_BEACON2_SIZE;
        } else if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.RED) {
            beacon1Location = RED_BEACON1_LOCATION;
            beacon2Location = RED_BEACON2_LOCATION;
            beacon1Size = RED_BEACON1_SIZE;
            beacon2Size = RED_BEACON2_SIZE;
        }
    }

    public void initialize(){
        beacon1State = beacon2State = State.UNKNOWN;
    }
    public void initialize(boolean useFullRes, OpenCVLinearOpModeBase op, boolean cont) {
        WIDTH_SCALING = (useFullRes)?1280.0/384:1;
        HEIGHT_SCALING = (useFullRes)?960.0/288:1;

        this.op = op;
        red1 = new Mat();
        redL1 = new Mat();
        redH1 = new Mat();
        blue1 = new Mat();

        continuous = cont;

        beacon1State = beacon2State = State.UNKNOWN;
    }

    public void setAlliance(OpenCVLinearOpModeBase.AllianceColor color) {
        allianceColor = color;
        if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE) {
            beacon1Location = BLUE_BEACON1_LOCATION;
            beacon2Location = BLUE_BEACON2_LOCATION;
            beacon1Size = BLUE_BEACON1_SIZE;
            beacon2Size = BLUE_BEACON2_SIZE;
        } else if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.RED) {
            beacon1Location = RED_BEACON1_LOCATION;
            beacon2Location = RED_BEACON2_LOCATION;
            beacon1Size = RED_BEACON1_SIZE;
            beacon2Size = RED_BEACON2_SIZE;
        }
    }

    public void stop() {
        // Explicitly deallocate Mats

        /*if (red != null){
            red.release();
        }
        if (redL != null){
            redL.release();
        }
        if (redH != null){
            redH.release();
        }
        if (blue != null){
            blue.release();
        }

        red = null;
        redL = null;
        redH = null;
        blue = null;*/
    }


    private void setRandomValues() {
        Random rand = new Random();
        beacon1State = (rand.nextBoolean()) ? State.RED_IS_LEFT : State.RED_IS_RIGHT;
        beacon2State = (rand.nextBoolean()) ? State.RED_IS_LEFT : State.RED_IS_RIGHT;
    }

    public void takePicture() {
        frameReady = false;
        processNextFrame = true;
        setRandomValues();
    }

    public boolean isBeacon1State(State state) {
        double cameraWaitStartTime = System.nanoTime();
        // wait 2 seconds at most
        while (!frameReady && ((cameraWaitStartTime + 2e9) > System.nanoTime())) {
        }
        if (!frameReady) return false;

        if ((state == State.RED_IS_LEFT) || (state == State.BLUE_IS_RIGHT)) {
            return ((beacon1State == State.RED_IS_LEFT) || (beacon1State == State.BLUE_IS_RIGHT));
        }
        if ((state == State.RED_IS_RIGHT) || (state == State.BLUE_IS_LEFT)) {
            return ((beacon1State == State.RED_IS_RIGHT) || (beacon1State == State.BLUE_IS_LEFT));
        }
        return false;
    }

    public boolean isBeacon2State(State state) {
        double cameraWaitStartTime = System.nanoTime();
        // wait 2 seconds at most
        while (!frameReady && ((cameraWaitStartTime + 2e9) > System.nanoTime())) {
        }
        if (!frameReady) return false;

        if ((state == State.RED_IS_LEFT) || (state == State.BLUE_IS_RIGHT)) {
            return ((beacon2State == State.RED_IS_LEFT) || (beacon2State == State.BLUE_IS_RIGHT));
        }
        if ((state == State.RED_IS_RIGHT) || (state == State.BLUE_IS_LEFT)) {
            return ((beacon2State == State.RED_IS_RIGHT) || (beacon2State == State.BLUE_IS_LEFT));
        }
        return false;
    }

    public void displayBeaconState(Telemetry telemetry) {
        if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE) {
            if (isBeacon1State(State.BLUE_IS_LEFT)) {
                telemetry.addData("b1 state:", "Blue is left");
            } else if (isBeacon1State(State.BLUE_IS_RIGHT)) {
                telemetry.addData("b1 state:", "Blue is right");
            } else {
                telemetry.addData("b1 state:", "Unknown");
            }
            if (isBeacon2State(State.BLUE_IS_LEFT)) {
                telemetry.addData("b2 state:", "Blue is left");
            } else if (isBeacon2State(State.BLUE_IS_RIGHT)) {
                telemetry.addData("b2 state:", "Blue is right");
            } else {
                telemetry.addData("b2 state:", "Unknown");
            }
        } else if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.RED) {
            if (isBeacon1State(State.RED_IS_LEFT)) {
                telemetry.addData("b1 state:", "Red is left");
            } else if (isBeacon1State(State.RED_IS_RIGHT)) {
                telemetry.addData("b1 state:", "Red is right");
            } else {
                telemetry.addData("b1 state:", "Unknown");
            }
            if (isBeacon2State(State.RED_IS_LEFT)) {
                telemetry.addData("b2 state:", "Red is left");
            } else if (isBeacon2State(State.RED_IS_RIGHT)) {
                telemetry.addData("b2 state:", "Red is right");
            } else {
                telemetry.addData("b2 state:", "Unknown");
            }
        }
    }

    private Mat editMat(Mat start, Scalar low, Scalar high, Mat mat2edit, boolean red){
        inspectMatrix(start);
        Imgproc.cvtColor(start, mat2edit, Imgproc.COLOR_RGB2HSV);
        inspectMatrix(mat2edit);
        Core.inRange(mat2edit, low, high, mat2edit);

        /*if(red) {
            redPixels = redPixels + Core.countNonZero(mat2edit);
        }*/
        //erosion
        //Imgproc.erode(mat2edit,mat2edit, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5)));
        //dilate
        Imgproc.dilate(mat2edit,mat2edit, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5)), new Point (-1,-1), 3);

        return mat2edit;
    }

    private Point findCenter(MatOfPoint contour){
        int x = 0, y =0;
        Moments p = Imgproc.moments(contour);
        x = (int)(p.get_m10()/p.get_m00());
        y = (int)(p.get_m01()/p.get_m00());
        return new Point(x,y);
    }

    private Point findClosest(Point target, List<MatOfPoint> contours) {
        Point retval = null;
        double minDist = Double.MAX_VALUE;// beaconDist * beaconDist; // was Double.MAX_VALUE
        for (MatOfPoint contour : contours) {
            Point center = findCenter(contour);
            double dist = (target.x - center.x) * (target.x - center.x) + (target.y - center.y) * (target.y - center.y);
            if (dist < minDist) {
                retval = center;
                minDist = dist;
            }
        }
        return retval;
    }

    private Point findLargest(List<MatOfPoint> contours) {
        double largestArea = Double.MIN_VALUE;
        Point retval = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestArea) {
                largestArea = area;
                retval = findCenter(contour);
            }
        }
        return retval;
    }

    private List<MatOfPoint> clearExtra(List<MatOfPoint> contours){
        List<MatOfPoint> contoursDraw = new ArrayList<>();
        for (int i = 0; i != contours.size(); i++)
        {
            Mat contour = contours.get(i);
            double contourArea = Imgproc.contourArea(contour);

            if (contourArea > minArea)
            {
                contoursDraw.add(contours.get(i));
            }

        }

        return contoursDraw;
    }

    private void findBeacons1(List<MatOfPoint> blueList, List<MatOfPoint> redList) {
        Point beacon1Red = findClosest((useFullScreen)?beacon1Location : new Point(70, 60), redList);
        Point beacon1Blue = findClosest((useFullScreen)?beacon1Location : new Point(70, 60), blueList);
        beacon1b = beacon1Blue;
        beacon1r = beacon1Red;

        if ((beacon1Red == null) && (beacon1Blue == null)) {
            beacon1State = State.UNKNOWN;
        } else if (beacon1Blue == null) {
            if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE) {
                beacon1State = State.BLUE_IS_LEFT;
            } else {
                beacon1State = State.RED_IS_LEFT;
            }
        } else if (beacon1Red == null) {
            if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE) {
                beacon1State = State.RED_IS_LEFT;
            } else {
                beacon1State = State.BLUE_IS_LEFT;
            }
        } else if (beacon1Red.x > beacon1Blue.x) {
            beacon1State = State.RED_IS_LEFT;
        } else {
            beacon1State = State.BLUE_IS_LEFT;
        }
    }
    private void findBeacons2(List<MatOfPoint> blueList, List<MatOfPoint> redList) {
        Point beacon2Red = findClosest((useFullScreen)?beacon2Location : new Point(BEACONWIDTH,BEACONHEIGHT), redList);
        Point beacon2Blue = findClosest((useFullScreen)?beacon2Location : new Point(BEACONWIDTH,BEACONHEIGHT), blueList);
        beacon2b = beacon2Blue;
        beacon2r = beacon2Red;

        if ((beacon2Red == null) && (beacon2Blue == null)) {
            beacon2State = State.UNKNOWN;
        }else if(beacon2Blue == null) {
            if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE) {
                beacon2State = State.BLUE_IS_LEFT;
            } else {
                beacon2State = State.RED_IS_LEFT;
            }
        }else if (beacon2Red == null){
            if(allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE){
                beacon2State = State.RED_IS_LEFT;
            }else{
                beacon2State = State.BLUE_IS_LEFT;
            }
        } else if (beacon2Red.x > beacon2Blue.x) {
            beacon2State = State.RED_IS_LEFT;
        } else {
            beacon2State = State.BLUE_IS_LEFT;
        }

        largestBlue = findLargest(blueList);
        largestRed = findLargest(redList);
    }

    public Point getLargestRed() {
        return largestRed;
    }

    public Point getLargestBlue() {
        return largestBlue;
    }

    private Point findCenter(List<MatOfPoint> contours){

        int x = 0, y =0;
        List<Moments> mu = new ArrayList<Moments>(contours.size());
        for(int i = 0; i < contours.size(); i++){
            mu.add(i, Imgproc.moments(contours.get(i),false));
            Moments p = mu.get(i);
            x = (int)(p.get_m10()/p.get_m00());
            y = (int)(p.get_m01()/p.get_m00());

        }
        return new Point(x,y);
    }
    private Mat filterGreen (Mat start){
        Mat lowGreen = new Mat();
        Imgproc.cvtColor(start, lowGreen, Imgproc.COLOR_RGB2BGR);

        Scalar low = new Scalar(0,0,0);
        Scalar high = new Scalar(255, 192, 255);
        Core.inRange(lowGreen, low, high, lowGreen);
        Mat noGreen = new Mat();


        List<MatOfPoint> contoursSat = new ArrayList<>();
        Imgproc.findContours(lowGreen, contoursSat, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Scalar elems = Core.sumElems(lowGreen);


        int nonZero = Core.countNonZero(lowGreen);
        op.telemetry.addData("elems", elems);
        op.telemetry.addData("nonZero", nonZero);
        if(nonZero == 0){
            midHValue = 120;
            midSValue =75;
            midVValue = 75;
        }else {
            midHValue = elems.val[0] / nonZero;
            midSValue = elems.val[1] / nonZero;
            midVValue = elems.val[2] / nonZero;
        }

        List<Mat> mv = new ArrayList<Mat>();
        Core.split(start, mv);
        Core.multiply(mv.get(0), lowGreen, mv.get(0));
        Core.multiply(mv.get(1), lowGreen, mv.get(1));
        Core.multiply(mv.get(2), lowGreen, mv.get(2));
        Core.merge(mv, noGreen);

        //Imgproc.cvtColor(noGreen,noGreen,Imgproc.COLOR_BGR2RGB);
        return noGreen;
    }

    private Mat filterHighSat(Mat start){
        Mat highSat = new Mat();
        Mat hsv = new Mat();
        Mat retval = new Mat();

        Imgproc.cvtColor(start, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar mean = Core.mean(hsv);
        Scalar low = new Scalar (0, mean.val[1], mean.val[2]);
        Scalar high = new Scalar(180, 255, 255);
/*
        midHValue = mean.val[0];
        midSValue = mean.val[1];
        midVValue = mean.val[2];
        op.telemetry.addData("H", mean.val[0]);
        op.telemetry.addData("S", mean.val[1]);
        op.telemetry.addData("V", mean.val[2]);
*/
        Core.inRange(hsv, low, high, highSat);

        //Scalar elems = Core.sumElems(highSat);

        List<MatOfPoint> contours = new ArrayList<>();
        //Imgproc.findContours(highSat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        /*int nonZero = Core.countNonZero(highSat);
        if(nonZero == 0){
            midHValue = 120;
            midSValue =75;
            midVValue = 75;
        }else {
            midHValue = elems.val[0] / nonZero;
            midSValue = elems.val[1] / nonZero;
            midVValue = elems.val[2] / nonZero;
        }*/
        //inspectMatrix(highSat);
        Scalar hsvMean  = Core.mean(hsv, highSat);

        midHValue = hsvMean.val[0];
        midSValue = hsvMean.val[1];
        midVValue = hsvMean.val[2];

        List<Mat> mv = new ArrayList<>();
        Core.split(start, mv);
        Core.bitwise_and(mv.get(0), highSat, mv.get(0));
        Core.bitwise_and(mv.get(1), highSat, mv.get(1));
        Core.bitwise_and(mv.get(2), highSat, mv.get(2));
        Core.merge(mv, retval);

        return retval;
    }
    private Point scaleFullResToScreen(int x, int y, Size size) {
        return new Point(x * size.width / MyJavaCameraView.getFullResolutionWidth(), y * size.height / MyJavaCameraView.getFullResolutionHeight());
    }

    public Mat processFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        //Blue filter goes here
        Mat rgba = inputFrame.rgba();

        if (!processNextFrame){
            Mat mRgba = inputFrame.rgba();
            leftMineralMat.create(rgba.size(), 0);
            midMineralMat.create(rgba.size(), 0);
            rightMineralMat.create(rgba.size(), 0);

            Rect leftMineral = new Rect(new Point(255, 0), new Point (555, 50));
            Rect rightMineral = new Rect(new Point(810, 0), new Point (1100, 50));
            Rect midMineral = new Rect(new Point(1365, 0), new Point (1665, 50));

            rgba.submat(leftMineral).copyTo(leftMineralMat);
            rgba.submat(midMineral).copyTo(midMineralMat);
            rgba.submat(rightMineral).copyTo(rightMineralMat);

            Scalar leftMineralMean = Core.mean(leftMineralMat);
            leftMineralBlue = leftMineralMean.val[2];

            Scalar midMineralMean = Core.mean(midMineralMat);
            midMineralBlue = midMineralMean.val[2];

            Scalar rightMineralMean = Core.mean(rightMineralMat);
            rightMineralBlue = rightMineralMean.val[2];

            return rgba;
        }
        frameReady = true;
    return rgba;
    }

    private boolean inspectMatrix(Mat start){
        byte b[] = new byte[start.channels()];
        int array[][]= new int[start.rows()][start.cols()];

        for(int i = 0; i < start.rows(); i++){
            for (int j = 0; j < start.cols(); j++){
                start.get(i,j,b);

                array[i][j]= b[0];
            }
        }

        return true;

    }

    private void filterWindow(Mat start, int beaconNumber){
        Mat pictureHSV = new Mat();
        Imgproc.cvtColor(start, pictureHSV, Imgproc.COLOR_RGB2HSV);
        Scalar mean;
        mean = Core.mean(pictureHSV);
        Scalar low = new Scalar(0, mean.val[1]*0.5, mean.val[2]*0.5);
        Scalar high = new Scalar(180, 255,255);
        Mat filterHighSat = new Mat();
        Core.inRange(pictureHSV, low, high, filterHighSat);

        Scalar filteredMean;
        filteredMean = Core.mean(pictureHSV, filterHighSat);

        //blue
        Scalar lowb = new Scalar(90, mean.val[1]*0.5, mean.val[2]*0.5);
        Scalar highb = new Scalar(filteredMean.val[0], 255,255);

        Mat highSatBlue = new Mat();
        Core.inRange(pictureHSV, lowb, highb, highSatBlue);
        List<MatOfPoint> blueContours = new ArrayList<>();
        Imgproc.findContours(highSatBlue, blueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(start, blueContours, -1, new Scalar(0,0,255), 3);

        //red
        low = new Scalar(filteredMean.val[0], mean.val[1]*0.5, mean.val[2]*0.5);
        high = new Scalar(180, 255,255);
        Mat highSatRedH = new Mat();
        Core.inRange(pictureHSV, low, high, highSatRedH);

        low = new Scalar(0, mean.val[1]*0.5, mean.val[2]*0.5);
        high = new Scalar(10, 255,255);
        Mat highSatRedL = new Mat();
        Core.inRange(pictureHSV, low, high, highSatRedL);

        Mat highSatRed = new Mat();
        Core.addWeighted(highSatRedH, 1.0, highSatRedL, 1.0, 0.0, highSatRed);

        List<MatOfPoint> redContours = new ArrayList<>();
        Imgproc.findContours(highSatRed, redContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(start, redContours, -1, new Scalar(255, 0,0), 3);

        Point blueCenter = findLargest(blueContours);
        Point redCenter = findLargest(redContours);
        findBeaconOrientation(blueCenter,redCenter, beaconNumber);

       if(blueCenter != null) Imgproc.circle(start, blueCenter, 2, new Scalar( 0, 255, 0), 4);
        if(redCenter != null) Imgproc.circle(start, redCenter, 2, new Scalar( 0, 255, 0), 4);


        textPoint = new Point(.1*start.width(), .5*start.height());
        if(blueCenter != null && redCenter != null) {
            if (blueCenter.x - redCenter.x < 0) {
                Imgproc.putText(start, "Red is Left", textPoint, 2, 1, new Scalar(0, 255, 0));
            } else if (blueCenter.x - redCenter.x > 0) {

                Imgproc.putText(start, "Blue is Left", textPoint, 2, 1, new Scalar(0, 255, 0));
            }
        }





    }

    private void findBeaconOrientation(Point blue, Point red, int beacon){
        State beaconState;
        if ((red == null) && (blue == null)) {
            beaconState = State.UNKNOWN;
        }else if(blue == null) {
            if (allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE) {
                beaconState = State.BLUE_IS_LEFT;
            } else {
                beaconState = State.RED_IS_LEFT;
            }
        }else if (red == null){
            if(allianceColor == OpenCVLinearOpModeBase.AllianceColor.BLUE){
                beaconState = State.RED_IS_LEFT;
            }else{
                beaconState = State.BLUE_IS_LEFT;
            }
        } else if (red.x > blue.x) {
            beaconState = State.RED_IS_LEFT;
        } else {
            beaconState = State.BLUE_IS_LEFT;
        }

        if(beacon == 1){
            beacon1State = beaconState;
        }else if(beacon == 2){
            beacon2State = beaconState;
        }


    }

    public Mat processFrame2(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        // rotate the frame
///*        Mat mRgba = inputFrame.rgba();
//        Mat mRgbaT = mRgba.t();
//        Core.flip(mRgba.t(), mRgbaT, 1);
//        Size mSize = new Size(mRgba.size().width, mRgba.size().height);
//        Imgproc.resize(mRgbaT, mRgbaT, mSize);
        Mat rgba = null;



        processNextFrame = false;

        frameReady = true;

        return rgba;
    }
    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);
        return new Scalar(pointMatRgba.get(0, 0));
    }
}
