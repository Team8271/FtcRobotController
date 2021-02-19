/*
   Holonomic/Mecanum concept autonomous program. Driving motors for TIME

   Robot wheel mapping:
          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ExampleCode.ExampleHardwareSetupHolonomic;

@Autonomous(name="WobbleGoalHolonomicTime", group="Concept")
//@Disabled
public class WobbleGoalHolonomicTime extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    /* Define Hardware setup */
    // assumes left motors are reversed
    Bot8271HolonomicHardwareSetup robot = new Bot8271HolonomicHardwareSetup();
    /**
     * Constructor
     */
    public WobbleGoalHolonomicTime() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Setup

        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

            /************************
             * Autonomous Code Below://
             *************************/
            DriveForwardTime(DRIVE_POWER, 5000);

    }//runOpMode

    /** Below: Wobble Goal Methods **/

    public void None() throws InterruptedException
    {
        DriveForwardTime(DRIVE_POWER, 2000);
        StrafeRight(DRIVE_POWER, 500);
        StrafeLeft(DRIVE_POWER, 1000);
        StopDriving();
    }

    public void Single() throws InterruptedException
    {

    }

    public void Quad() throws InterruptedException
    {

    }

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);//still need to test motor directions for desired movement
        robot.motorFrontLeft.setPower(-power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(-power);
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

    public void StrafeLeft(double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);
        robot.motorFrontLeft.setPower(-power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
        StopDrivingTime(500);
    }

    public void StrafeRight(double power, long time) throws InterruptedException
    {
        StrafeLeft(-power, time);
    }

    public void SpinRight (double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(-power);
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
        StopDrivingTime(500);
    }

    public void SpinLeft (double power, long time) throws InterruptedException
    {
        SpinRight(-power, time);
    }


/*** Currently no Servo configured in Holonomic Hardware setup

    public void RaiseArm()
    {
        robot.armServo.setPosition(.8); //note: uses servo instead of motor.
    }

    public void LowerArm()
    {
        robot.armServo.setPosition(.2);
    }
*/


}//TestAutoDriveByTime
