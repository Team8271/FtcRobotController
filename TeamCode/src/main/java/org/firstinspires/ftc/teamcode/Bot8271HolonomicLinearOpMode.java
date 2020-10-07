package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExampleCode.ExampleHardwareSetupHolonomic;


/**
 *
 * This is a Linear version program (i.e. uses runOpMode() and waitForStart() methods,  instead of init, loop and stop)
 * for TeleOp control with a single controller
 */

/*
   Holonomic concepts from:
   http://www.vexforum.com/index.php/12370-holonomic-drives-2-0-a-video-tutorial-by-cody/0
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
@TeleOp(name = "Bot8271HolonomicLinearOpMode", group = "LinearHolonomic")
//@Disabled
public class Bot8271HolonomicLinearOpMode extends LinearOpMode
{

    // create timer
    private ElapsedTime runtime = new ElapsedTime();

    //  DON'T FORGET TO RENAME HARDWARE CONFIG FILE NAME HERE!!!!!!
    Bot8271HolonomicHardwareSetup robot = new Bot8271HolonomicHardwareSetup();

    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and configuration file.
         */
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class


        waitForStart();
        runtime.reset(); // starts timer once start button is pressed

        while(opModeIsActive())
        {
            // left stick: X controls Strafe & Y controls Spin Direction
            // right stick: Y controls drive Forward/Backward
            float gamepad1LeftY = -gamepad1.left_stick_y;   // drives spin left/right
            float gamepad1LeftX = gamepad1.left_stick_x;    // strafe direction (side to side)
            float gamepad1RightY = gamepad1.right_stick_y;  //drives forwards and backwards

            // holonomic formulas
            float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightY;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightY;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightY;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightY;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            // write the clipped values from the formula to the motors
            robot.motorFrontRight.setPower(FrontRight);
            robot.motorFrontLeft.setPower(FrontLeft);
            robot.motorBackLeft.setPower(BackLeft);
            robot.motorBackRight.setPower(BackRight);

            /*
             * Display Telemetry for debugging
             */
            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightY));
            telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));

        }
    }
}