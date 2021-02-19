/**
 * This file contains an minimal example of a Linear Autonomous "OpMode".
 *
 * This particular OpMode just executes a basic Autonomous driving for time, not using encoders
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Don't forget to comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Example AutoByTime", group="Examples")  // @TeleOp(...) is the other common choice
@Disabled
public class TestAutoDriveByTime extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorArm = null;

    //servos
    Servo armServo = null;
    Servo servoHandL = null;
    Servo servoHandR = null;

    //Create and set default hand positions variables. To be determined based on your build
    double CLOSED = 0.2;
    double OPEN = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
         motorLeft  = hardwareMap.dcMotor.get("motorL");
         motorRight = hardwareMap.dcMotor.get("motorR");
         motorArm = hardwareMap.dcMotor.get("motorArm");
         armServo = hardwareMap.servo.get("armServo");     //autonomous code example below uses servo for arm instead/in addition to motor
         servoHandL = hardwareMap.servo.get("servoHandL"); //assuming a pushBot configuration of two servo grippers
         servoHandR = hardwareMap.servo.get("servoHandR");
        
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
         motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
         motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
         motorArm.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        
        //Set servo hand grippers to open position. 
         servoHandL.setPosition(OPEN);
         servoHandR.setPosition(OPEN);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * Autonomous Code Below://
         *************************/
        DriveForwardTime(DRIVE_POWER, 2000);
        TurnLeft(DRIVE_POWER, 1000);
        StopDrivingTime(2000);

        DriveForwardTime(DRIVE_POWER, 2000);
        TurnRight(DRIVE_POWER, 1000);
        StopDrivingTime(2000);

        //RaiseArm();
        //DriveForwardTime(DRIVE_POWER, 4000);
        //StopDriving();

       

    }//runOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
        StopDrivingTime(500);
    }

    public void StopDriving()
    {
        DriveForward(0);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        DriveForwardTime(0, time);
    }

    public void TurnLeft(double power, long time) throws InterruptedException
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        Thread.sleep(time);
        StopDrivingTime(500);
    }

    public void TurnRight(double power, long time) throws InterruptedException
    {
        TurnLeft(-power, time);
    }

    public void RaiseArm()
    {
        armServo.setPosition(.8); //note: uses servo instead of motor.
    }

    public void LowerArm()
    {
        armServo.setPosition(.2);
    }


}//TestAutoDriveByTime
