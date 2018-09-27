package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "throttleTestMinimal", group = "TeleOp")

public class throttleTestMinimal extends OpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double leftJoyStick, rightJoyStick, leftMotorPower, rightMotorPower, liftMotorPower, clampMotorPower, leftServoPower, rightServoPower;

    final private static double JOYSTICK_DEADBAND = 0.1;

    //Encoder Ticks Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 / 3 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private BotDawg robot;

    @Override
    public void init() {

        robot = new BotDawg();
        robot.init(hardwareMap);

    }

    //Code that resets the elapsed time once the driver hits play
    @Override
    public void start() {
        runtime.reset();
    }


    public void loop() {

        // assign the value of the joystick to the throttle variables.
        double rightThrottle = -gamepad2.left_stick_y;
        double leftThrottle = -gamepad2.right_stick_y;

        //I MIGHT NEED TO ADD JOYSTICK_DEADBAND
        robot.rightFrontMotor.setPower(rightThrottle);
        robot.rightBackMotor.setPower(rightThrottle);
        robot.rightFrontMotor.setPower(rightThrottle);
        robot.rightBackMotor.setPower(rightThrottle);

        //Telemetry is not used to control the robot, it is purely to help debug by showing
        //Information on the phone
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftThrottle, rightThrottle);
    }
}