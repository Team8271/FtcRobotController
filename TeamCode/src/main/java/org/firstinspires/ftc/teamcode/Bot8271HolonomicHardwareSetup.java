package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by TeameurekaRobotics on 12/30/2016, updated 10/1/2019
 *
 * This file contains an example Hardware Setup Class for a 4 motor Holonomic drive.
 *
 * It can be customized to match the configuration of your Bot by adding/removing hardware, and then used to instantiate
 * your bot hardware configuration in all your OpModes. This will clean up OpMode code by putting all
 * the configuration here, needing only a single instantiation inside your OpModes and avoid having to change configuration
 * in all OpModes when hardware is changed on robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *
 */

public class Bot8271HolonomicHardwareSetup {

   /* Declare Public OpMode members.
    *these are the null statements to make sure nothing is stored in the variables.
    */

    //Drive motors
    public DcMotor motorFrontRight = null;
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;

    //Accessories motors
    public DcMotor armMotor = null;

    //servos
        //Add servos here

    //sensors
        //Add sensors here

    /* local OpMode members. */
    HardwareMap hwMap        = null;

    //Create and set default servo positions & MOTOR STOP variables.
    //Possible servo values: 0.0 - 1.0  For CRServo 0.5=stop greater or less than will spin in that direction
    //final static double CLOSED = 0.2;
    //final static double OPEN = 0.8;
    final static double MOTOR_STOP = 0.0; // sets motor power to zero

    //CR servo variables
        //Add servo variable here

   /* Constructor   // this is not required as JAVA does it for you, but useful if you want to add
    * function to this method when called in OpModes.
    */
    public Bot8271HolonomicHardwareSetup() {
    }

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /************************************************************
         * MOTOR SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        motorFrontLeft = hwMap.dcMotor.get("motorFL");
        motorFrontRight = hwMap.dcMotor.get("motorFR");
        motorBackLeft = hwMap.dcMotor.get("motorBL");
        motorBackRight = hwMap.dcMotor.get("motorBR");

        armMotor = hwMap.dcMotor.get ("armMotor");

        // Set the drive motor directions:
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        //Keep the motors from moving during initialize.
        motorFrontLeft.setPower(MOTOR_STOP);
        motorFrontRight.setPower(MOTOR_STOP);
        motorBackLeft.setPower(MOTOR_STOP);
        motorBackRight.setPower(MOTOR_STOP);

        /************************************************************
         * SERVO SECTION
         ************************************************************/

            //Add servo configuration

        /************************************************************
         * SENSOR SECTION
         ************************************************************/
            //Add sensors

   }

}

