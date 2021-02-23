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
            float maxSpeed = 0.3f + gamepad1.right_trigger/2;

            // left stick: X controls Strafe & forward/backward
            float gamepad1LeftY = -gamepad1.right_stick_x;   // drives spin
            float gamepad1LeftX = -gamepad1.left_stick_x;    // strafe direction (side to side)

            // right stick: Y controls spin direction
            float gamepad1RightX = gamepad1.left_stick_y;  // drives forward/back

            // holonomic formulas
            float FrontLeft = -gamepad1RightX - gamepad1LeftX - gamepad1LeftY;
            float FrontRight = gamepad1RightX - gamepad1LeftX - gamepad1LeftY;
            float BackRight = gamepad1RightX + gamepad1LeftX - gamepad1LeftY;
            float BackLeft = -gamepad1RightX + gamepad1LeftX - gamepad1LeftY;

            // clip the right/left values so that the values never exceed +/- 1
           FrontRight = Range.clip(FrontRight, -maxSpeed, maxSpeed);
           FrontLeft = Range.clip(FrontLeft, -maxSpeed, maxSpeed);
           BackLeft = Range.clip(BackLeft, -maxSpeed, maxSpeed);
           BackRight = Range.clip(BackRight, -maxSpeed, maxSpeed);

            // write the clipped values from the formula to the motors
            robot.motorFrontRight.setPower(FrontRight);
            robot.motorFrontLeft.setPower(FrontLeft);
            robot.motorBackLeft.setPower(BackLeft);
            robot.motorBackRight.setPower(BackRight);

            // Disc Controls \\

            robot.frontDiscLaunch.setPower(gamepad2.right_trigger/1.5);


            // Intake Controls \\

            if(gamepad2.x)
            {
                // is this open or closed????????????????????????????????????
                robot.stopServo.setPosition(.3);
            }

            else
            {
                // What about here ????????????????? Duhhhhhh
                robot.stopServo.setPosition(.155);
            }

            if(gamepad2.left_bumper)
            {
                robot.intakeServo.setPosition(.7);
                robot.intakeMotor.setPower(1);
                robot.backDiscLaunch.setPower(1);
            }

            else if(gamepad2.right_bumper)
            {
                robot.intakeMotor.setPower(-1);
                robot.intakeServo.setPosition(.1);
                robot.backDiscLaunch.setPower(-1);
            }

            else
                {
                    robot.intakeMotor.setPower(0);
                    robot.intakeServo.setPosition(.5);
                    robot.backDiscLaunch.setPower(0);
                }



            // Arm controls \\

            if(gamepad2.a)
            {
                robot.commaClaw.setPosition(.8);
            }
            else
            {
                robot.commaClaw.setPosition(.2);
            }

            robot.armMotor.setPower(-gamepad2.right_stick_x);

            /*
             * Display Telemetry for debugging
             */
            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
            telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));
            telemetry.update();
        }
    }
}