package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "throttleTestMinimal", group = "TeleOp")

public class throttleTestMinimal extends OpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double leftJoyStick, rightJoyStick, leftMotorPower, rightMotorPower, liftMotorPower, clampMotorPower, leftServoPower, rightServoPower;

    final private static double JOYSTICK_DEADBAND = 0.1;

    //Encoder Ticks Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private BotDawg robot;

    @Override
    public void init() {

        robot = new BotDawg();
        robot.init(hardwareMap);
//        robot.scoopMotor.setTargetPosition(0);
    }

    //Code that resets the elapsed time once the driver hits play
    @Override
    public void start() {
        runtime.reset();
    }


    public void loop() {

        // assign the value of the joystick to the throttle variables.
//        double rightThrottle = -gamepad2.left_stick_y;
//        double leftThrottle = -gamepad2.right_stick_y;
        robot.leftBackMotor.setPower(gamepad2.left_stick_y);
        robot.leftFrontMotor.setPower(gamepad2.left_stick_y);

        robot.rightBackMotor.setPower(gamepad2.right_stick_y);
        robot.rightFrontMotor.setPower(gamepad2.right_stick_y);

        // Setting the power of the lift motor to the y value of the gamepad1 right joystick
        robot.liftMotor.setPower(gamepad1.right_stick_y);
        robot.scoopMotor.setPower(1);
        // Setting the power of the scoop motor to the bumpers of the gamepad1
        if(!gamepad1.dpad_down && gamepad1.dpad_up){
          robot.scoopMotor.setTargetPosition(0);
        }else if (gamepad1.dpad_down && !gamepad1.dpad_up){
          robot.scoopMotor.setTargetPosition(-72);
        }

        if(gamepad1.right_bumper && !gamepad1.left_bumper){
            robot.leftLiftServo.setPosition(0.25);
            robot.rightLiftServo.setPosition(0.25);
        }else if(!gamepad1.right_bumper && gamepad1.left_bumper){
            robot.leftLiftServo.setPosition(0.0);
            robot.rightLiftServo.setPosition(0.0);
        }



        //I MIGHT NEED TO ADD JOYSTICK_DEADBAND


        //Telemetry is not used to control the robot, it is purely to help debug by showing
        //Information on the phone
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", gamepad2.left_stick_y, gamepad2.right_stick_y);
        telemetry.addData("Dpad up","Up, Down", gamepad1.dpad_up,gamepad1.dpad_down);
        telemetry.addData("Scoop power", robot.scoopMotor.getPowerFloat());
    }
}