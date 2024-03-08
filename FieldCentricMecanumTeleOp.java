package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder

    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.

    static final double WHEEL_DIAMETER_INCHES = 3.77952;     // For figuring circumference

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    boolean slowMode = false;


    private DcMotorEx elbow;
    private DcMotorEx elbow2;





    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftEncoder");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontEncoder");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightEncoder");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow2 = hardwareMap.get(DcMotorEx.class, "elbow2");


        Servo clawL = hardwareMap.servo.get("clawL");

        Servo clawR = hardwareMap.servo.get("clawR");


        CRServo hook = hardwareMap.crservo.get("hook");


        Servo wrist = hardwareMap.servo.get("wrist");
        Servo droneRelease = hardwareMap.servo.get("droneRelease");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow2.setDirection(DcMotorSimple.Direction.REVERSE);


        droneRelease.setDirection(Servo.Direction.REVERSE);







        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            double triggerVal = gamepad1.left_trigger;
            double powerFactor = slowMode ? 0.2 : 1.0;


            if (triggerVal > 0.1) {
                // Map trigger value from [0.1, 1] to [0.5, 1] for finer control
                powerFactor = 0.2 + 0.2 * (1 - triggerVal);
            }






            if (Math.abs(gamepad2.left_stick_y) > .1) {

                elbow.setPower(gamepad2.left_stick_y * 0.7);
                elbow2.setPower(gamepad2.left_stick_y * 0.7);
            } else{
                elbow.setPower(0);
                elbow2.setPower(0);
            }







            if (gamepad2.left_bumper) {
                clawL.setPosition(0.05);
            } else {
                clawL.setPosition(0.4);
            }

            if (gamepad2.right_bumper) {
                clawR.setPosition(0.6);
            } else{
                clawR.setPosition(0.4);
            }




            if (gamepad1.y){
                droneRelease.setPosition(1);
            } else {
                droneRelease.setPosition(0);
            }





            if (gamepad2.x){
                wrist.setPosition(0.17);

            } else if(gamepad2.a){
                wrist.setPosition(0.5);
            }  else if (gamepad2.b) {
                wrist.setPosition(0.73);
            }  else {
                    wrist.setPosition(0.85);

                }



            if (gamepad2.y){
                hook.setPower(-1);
            } else {
                hook.setPower(0);
            }







            double y = -gamepad1.left_stick_y ; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x ;
            double rx = gamepad1.right_stick_x ;
            y *= powerFactor;
            x *= powerFactor;
            rx *= powerFactor;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);





        }
    }
}



