package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.drawable.GradientDrawable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Autonomous (name = "BlueScanLong", group = "AUTO")
public class BlueScanLong extends LinearOpMode {

    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;




    DcMotor elbow;
    DcMotor elbow2;


    Servo clawL;
    Servo clawR;

    Servo wrist;

    OpenCvCamera webcam;


    private ElapsedTime runtime = new ElapsedTime();


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77952;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);





    BlueSightPipeline pipeline = new BlueSightPipeline(telemetry);

    @Override
    public void runOpMode() {
        //initialize robot hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(startPose);


        Trajectory traj1BLL = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-30,1.5,Math.toRadians(90)))
                .build();

        Trajectory traj2BLL = drive.trajectoryBuilder(traj1BLL.end())
                .lineToLinearHeading(new Pose2d(-52,-3,Math.toRadians(270)))
                .build();

        Trajectory traj3BLL = drive.trajectoryBuilder(traj2BLL.end())
                .lineToLinearHeading(new Pose2d(-52,-70,Math.toRadians(270)))
                .build();
        Trajectory traj4BLL = drive.trajectoryBuilder(traj3BLL.end())
                .lineToLinearHeading(new Pose2d(-18,-83.8,Math.toRadians(270)))
                .build();
        Trajectory traj5BLL =  drive.trajectoryBuilder(traj4BLL.end())
                .strafeLeft(20)
                .build();




        Trajectory traj1BCL = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-51,16,Math.toRadians(180)))
                .build();
        Trajectory traj2BCL = drive.trajectoryBuilder(traj1BCL.end())
                .lineToLinearHeading(new Pose2d(-52,-2,Math.toRadians(180)))
                .build();
        Trajectory traj3BCL = drive.trajectoryBuilder(traj2BCL.end())
                .lineToLinearHeading(new Pose2d(-51,-10,Math.toRadians(273)))
                .build();
        Trajectory traj4BCL = drive.trajectoryBuilder(traj3BCL.end())
                .lineToLinearHeading(new Pose2d(-51,-60, Math.toRadians(273)))
                .build();
        Trajectory traj5BCL = drive.trajectoryBuilder(traj4BCL.end())
                .lineToLinearHeading(new Pose2d(-22,-84,Math.toRadians(273)))
                .build();
        Trajectory traj6BCL = drive.trajectoryBuilder(traj5BCL.end())
                .strafeLeft(26)
                .build();






        Trajectory traj1BRL = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44,7,Math.toRadians(187)))
                .build();
        Trajectory traj2BRL = drive.trajectoryBuilder(traj1BRL.end())
                .lineToLinearHeading(new Pose2d(-53,-10,Math.toRadians(270)))
                .build();
        Trajectory traj3BRL = drive.trajectoryBuilder(traj2BRL.end())
                .lineToLinearHeading(new Pose2d(-53,-65,Math.toRadians(270)))
                .build();
        Trajectory traj4BRL = drive.trajectoryBuilder(traj3BRL.end())
                .lineToLinearHeading(new Pose2d(-38.25,-82.3,Math.toRadians(270)))
                .build();

        Trajectory traj5BRL = drive.trajectoryBuilder(traj4BRL.end())
                .strafeLeft(30)
                .build();


        fr = hardwareMap.get(DcMotor.class, "frontEncoder");
        bl = hardwareMap.get(DcMotor.class, "leftEncoder");
        br = hardwareMap.get(DcMotor.class, "rightEncoder");
        fl = hardwareMap.get(DcMotor.class, "fL");


        elbow = hardwareMap.dcMotor.get("elbow");
        elbow2 = hardwareMap.dcMotor.get("elbow2");

        wrist = hardwareMap.servo.get("wrist");

        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");






        clawL.setPosition(0.4);
        clawR.setPosition(0);





        elbow.setDirection(DcMotor.Direction.FORWARD);
        elbow2.setDirection(DcMotorSimple.Direction.REVERSE);




        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);











        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);









        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);




        //FOR THE WEBCAM
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * Webcam stream goes to RC phone
         */


        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously which allows faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                //320px x 340px
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                /*
                 * Specify the image processing pipeline we wish to invoke upon receipt
                 * of a frame from the camera. Note that switching pipelines on-the-fly
                 * (while a streaming session is in flight) *IS* supported.
                 */
                webcam.setPipeline(pipeline);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("errorCode", errorCode);
            }
        });
        // Tell telemetry to update faster than the default 250ms period :)
        //telemetry.setMsTransmissionInterval(20);

        // telemetry.addLine("Waiting for start");
        // telemetry.update();

        //Wait for the user to press start on the Driver Station

        waitForStart();

        //Manages Telemetry and stopping the stream
        while (opModeIsActive()) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            switch (pipeline.getAnalysis()) {
                case LEFT:

                    sleep(800);
                    wrist.setPosition(0.24);
                    sleep(400);
                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elbowDrive(1,-12,1.5);
                    drive.followTrajectory(traj1BLL);
                    clawR.setPosition(0.6);
                    sleep(400);
                    wrist.setPosition(0.8);
                    clawR.setPosition(0.4);
                    drive.followTrajectory(traj2BLL);
                    drive.followTrajectory(traj3BLL);
                    drive.followTrajectory(traj4BLL);
                    sleep(500);
                    elbowDrive(0.8,45,3.75);
                    clawL.setPosition(0.05);
                    sleep(500);
                    elbowDrive(0.8,-20,1.5);
                    drive.followTrajectory(traj5BLL);





                    break;
                case CENTER:
                    sleep(800);
                    wrist.setPosition(0.24);
                    sleep(400);
                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elbowDrive(1,-12,1.5);
                    drive.followTrajectory(traj1BCL);
                    drive.followTrajectory(traj2BCL);

                    clawR.setPosition(0.6);
                    sleep(400);
                    wrist.setPosition(0.8);
                    clawR.setPosition(0.4);
                    sleep(500);
                    drive.followTrajectory(traj3BCL);
                    drive.followTrajectory(traj4BCL);
                    drive.followTrajectory(traj5BCL);
                    sleep(500);
                    elbowDrive(0.8,45,3.75);
                    clawL.setPosition(0.05);
                    sleep(400);
                    elbowDrive(0.8,-40,1.5);
                    drive.followTrajectory(traj6BCL);



                    break;
                case RIGHT:
                    sleep(800);
                    wrist.setPosition(0.24);
                    sleep(400);
                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elbowDrive(1,-12,1.5);
                    drive.followTrajectory(traj1BRL);
                    clawR.setPosition(0.6);
                    sleep(400);
                    wrist.setPosition(0.8);
                    clawR.setPosition(0.4);
                    drive.followTrajectory(traj2BRL);
                    drive.followTrajectory(traj3BRL);
                    drive.followTrajectory(traj4BRL);
                    sleep(500);
                    elbowDrive(0.8,45,3.75);
                    clawL.setPosition(0.05);
                    sleep(400);;
                    elbowDrive(0.8,-20,1.5);
                    drive.followTrajectory(traj5BRL);


                    break;
            }

            //reminder to use the KNO3 auto transitioner once this code is working

            webcam.stopStreaming();
            webcam.closeCameraDevice();
            break;
        }
    }






    public void elbowDrive(double speed,
                           double elbowenco, double timeoutS) {

        int newElbow;
        int newElbow2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newElbow = elbow.getCurrentPosition() + (int) (elbowenco * COUNTS_PER_INCH);
            newElbow2 = elbow2.getCurrentPosition() + (int) (elbowenco * COUNTS_PER_INCH);

            elbow.setTargetPosition(newElbow);
            elbow2.setTargetPosition(newElbow2);

            // Turn On RUN_TO_POSITION


            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            elbow.setPower(Math.abs(speed));
            elbow2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elbow.isBusy() && elbow2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", "%7d: %7d", newElbow, newElbow2);
                telemetry.addData("Currently at", " at %7d :%7d", newElbow, newElbow2,

                        elbow.getCurrentPosition(),
                        elbow2.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION


            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            elbow.setPower(0);
            elbow2.setPower(0);

            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            sleep(250);   // optional pause after each move.
        }
    }
}