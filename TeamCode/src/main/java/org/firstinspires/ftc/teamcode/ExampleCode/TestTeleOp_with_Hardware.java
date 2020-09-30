package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 *  Created by TeameurekaRobotics on 12/30/2016
 *
 * This file contains an minimal example of a Linear Tele "OpMode".
 * The hardware configuration uses MyBotHardwareSetup.java
 *
 * This particular OpMode just executes a basic Tank Drive, an Arm motor and 2 Servos similar to a PushBot
 * It includes all the skeletal structure that a linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Example: TestHardwareTeleOp", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class TestTeleOp_with_Hardware extends LinearOpMode {

    TestBotHardwareSetup robot = new TestBotHardwareSetup(); //set up remote to robot hardware configuration

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap); //initializes hardware mapping from remote hardware configuration

        // Display current status to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /************************
         * TeleOp Code Below://
         *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)

            // Display gamepad values to DS
            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);
            telemetry.addLine("Butons | ")
                    .addData("Button_A", gamepad1.a)
                    .addData("Button_B", gamepad1.b);
            telemetry.update();

            // tank drive set drive motor powers to Y-stick value
            robot.motorLeft.setPower(gamepad1.left_stick_y);
            robot.motorRight.setPower(gamepad1.right_stick_y);

            // Arm Control - Uses dual buttons to control motor direction
            if(gamepad1.right_bumper) {
                robot.motorArm.setPower(-gamepad1.right_trigger); // if both Bumper + Trigger, then negative power, runs arm down
            }
            else {
                robot.motorArm.setPower(gamepad1.right_trigger);  // else trigger positive value, runs arm up
            }

            //servo commands
            if(gamepad1.a) {
                robot.servoHandR.setPosition(robot.OPEN);  // Note: to change position value, go to HardwareSetup, or create new value or variable in this OpMode
                robot.servoHandL.setPosition(robot.OPEN);
            }
            else if (gamepad1.b) {
                robot.servoHandR.setPosition(robot.CLOSED);
                robot.servoHandL.setPosition(robot.CLOSED);
            }

            idle(); // Allows other parallel processes to run before loop repeats
        }
    }
}
