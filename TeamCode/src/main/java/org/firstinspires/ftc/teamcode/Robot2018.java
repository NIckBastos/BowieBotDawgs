package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot2018 {
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    //DcMotor hangMotor;
    //ModernRoboticsI2cGyro gyroSensor;
    BNO055IMU imuSensor;
    Orientation lastAngles = new Orientation();
    DataLogger dataLogger;
    double globalAngle;
    private static double testingNumber = 1;
    private static double ticksPerRev = 28*40/testingNumber /*The divided by testingNumber is just for testing*/;

    private int leftMotorPosition, rightMotorPosition;
    private static int wheelSize = 4;
    private double inchesMoved = 0;
    private double zAxisOrientation;

    public Robot2018(HardwareMap hardwareMap) throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imuSensor = hardwareMap.get(BNO055IMU.class, "imu");
        frontRightMotor = hardwareMap.dcMotor.get("FrontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("BackRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("BackLeftMotor");
        //hangMotor = hardwareMap.dcMotor.get("MotorHang");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //hangMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imuSensor.initialize(parameters);
        dataLogger = new DataLogger("Robot2018");
        // make sure the imu gyro is calibrated before continuing.
        resetAngle();
        resetEncoders();

    }

    public void setPowerRobot(double rightPower, double leftPower) {
        frontRightMotor.setPower(rightPower);
        frontLeftMotor.setPower(leftPower);
        backRightMotor.setPower(rightPower);
        backLeftMotor.setPower(leftPower);
    }

    public void resetEncoders () {
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveRobot(String movement, double amount, LinearOpMode linearOpMode) {
        switch(movement) {
            case "Forward":
                frontRightMotor.setPower(1);
                frontLeftMotor.setPower(1);
                backRightMotor.setPower(1);
                backLeftMotor.setPower(1);
                while (inchesMoved <= amount && linearOpMode.opModeIsActive()) {
                    double averageEncoders = (leftMotorPosition + rightMotorPosition)/2;
                    inchesMoved = wheelSize * Math.PI * (averageEncoders/ticksPerRev);
                    leftMotorPosition = backLeftMotor.getCurrentPosition();
                    rightMotorPosition = backRightMotor.getCurrentPosition();
                    linearOpMode.telemetry.addData("Left Encoder", leftMotorPosition);
                    linearOpMode.telemetry.addData("Right Encoder", rightMotorPosition);
                    linearOpMode.telemetry.addData("Inches Moved", inchesMoved);
                    linearOpMode.telemetry.update();
                    dataLogger.addField(leftMotorPosition);
                    dataLogger.addField(rightMotorPosition);
                    dataLogger.addField(inchesMoved);
                }
                break;
            case "Reverse":
                frontRightMotor.setPower(-1);
                frontLeftMotor.setPower(-1);
                backRightMotor.setPower(-1);
                backLeftMotor.setPower(-1);
                while (inchesMoved >= -amount && linearOpMode.opModeIsActive()) {
                    double averageEncoders = (leftMotorPosition + rightMotorPosition)/2;
                    inchesMoved = wheelSize * Math.PI * (averageEncoders/ticksPerRev);
                    leftMotorPosition = backLeftMotor.getCurrentPosition();
                    rightMotorPosition = backRightMotor.getCurrentPosition();
                    linearOpMode.telemetry.addData("Left Encoder", leftMotorPosition);
                    linearOpMode.telemetry.addData("Right Encoder", rightMotorPosition);
                    linearOpMode.telemetry.addData("Left Encoder F", frontLeftMotor.getCurrentPosition());
                    linearOpMode.telemetry.addData("Right Encoder F", frontRightMotor.getCurrentPosition());
                    linearOpMode.telemetry.addData("Inches Moved", inchesMoved);
                    linearOpMode.telemetry.update();
                    dataLogger.addField(leftMotorPosition);
                    dataLogger.addField(rightMotorPosition);
                    dataLogger.addField(inchesMoved);
                }
                break;
            case "Stop":
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                resetEncoders();
                resetAngle();
                leftMotorPosition = 0;
                rightMotorPosition = 0;
                inchesMoved = 0;
                break;
            case "CenterRotateRight":
                frontRightMotor.setPower(1);
                frontLeftMotor.setPower(-1);
                backRightMotor.setPower(1);
                backLeftMotor.setPower(-1);
                while (zAxisOrientation >= -amount && linearOpMode.opModeIsActive()) {
                    zAxisOrientation = getAngle();
                    linearOpMode.telemetry.addData("Z Axis Orientation", zAxisOrientation);
                }
                break;
            case "CenterRotateLeft":
                frontRightMotor.setPower(-1);
                frontLeftMotor.setPower(1);
                backRightMotor.setPower(-1);
                backLeftMotor.setPower(1);
                while (zAxisOrientation <= amount && linearOpMode.opModeIsActive()) {
                    zAxisOrientation = getAngle();
                    linearOpMode.telemetry.addData("Z Axis Orientation", zAxisOrientation);
                }
                break;
            /*case "RotateRightBack":
                frontRightMotor.setPower(-1);
                frontLeftMotor.setPower(0);
                break;
            case "RotateLeftBack":
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(-1);
                break;
            case "RotateRightForward":
                frontRightMotor.setPower(1);
                frontLeftMotor.setPower(0);
                break;
            case "RotateLeftForward":
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(1);
                break;*/
        }
    }
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void resetAngle()
    {
        lastAngles = imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /*public void initializeGyro(LinearOpMode opmode)/* throws InterruptedException {*/

        //ModernRoboticsI2cGyro gyroSensor = new ModernRoboticsI2cGyro(I2cDeviceSynch);
        //ElapsedTime time = new ElapsedTime();


        /*// initialize the gyroscope
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
          //  Thread.sleep(50);
        }
        gyroSensor.resetZAxisIntegrator();
  //      while (gyroSensor.getHeading() != 0) {
        //} //Waits in init phase until gyro is zeroed*//*


        opmode.telemetry.log().add("Gyro Calibrating. Do Not Move!");
        gyroSensor.calibrate();

        // Wait until the gyro calibration is complete
        time.reset();
        while (!opmode.isStopRequested() && gyroSensor.isCalibrating())  {
            opmode.telemetry.addData("calibrating", "%s", Math.round(time.seconds())%2==0 ? "|.." : "..|");
            opmode.telemetry.update();
            opmode.sleep(50);
        }

        gyroSensor.resetZAxisIntegrator();

        opmode.telemetry.log().clear(); opmode.telemetry.log().add("Gyro Calibrated");
        opmode.telemetry.clear(); opmode.telemetry.update();

    }*/

}
