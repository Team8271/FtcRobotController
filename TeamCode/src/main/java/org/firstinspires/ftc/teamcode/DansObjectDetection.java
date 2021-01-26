package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="DansObjectDetection", group="Concept")
//@Disabled
public class DansObjectDetection extends LinearOpMode {

    Bot8271HolonomicHardwareSetup robot = new Bot8271HolonomicHardwareSetup();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AX6CxDD/////AAABmVjo2GqFc0O5jG5+7W55SRFYkiAcsL11yqI/4PpvcyEDOtKytbQJhgh0DDoUAWOvaQRRLd/Kxbg02+U2Ig4kn4jaBKy1H26aaffnseu75xOG8/xVM+oAlFDLB4O4hyuXgNlPZXNaW6dhMN4Zhc9eO5dLJztF1wdacWhkqVDpGnuiaJDKCHPLh6bERTQpgZtBGds28Hv33BYH7t53dthvunOIWRAF1dGauPuluMVlf7Hh9Vp7woG3FFxSeaGSJ3oFJfCuS1cNm48crnPI+oe7gjB/cjp0oFIiha5fnm5UW/AdvgQfLy1tI6PoW2xABiYMp3r1jSaJOKnvbh321yVXtJCt0elSEzw/C50o+4n2fQb7";
    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public String analyseStack() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                // Loop through found ring stacks. Theoretically should only ever be one stack found
                for (Recognition recognition : updatedRecognitions) {
                    //Verify the stack that was found meets minimum confidence value
                    if (recognition.getConfidence() > .8) {
                        if (recognition.getLabel().equals("Single")) {
                            //Single ring stack found so that means we need to go to target zone B
                            return "B";
                        } else {
                            //4 ring stack found so that means we need to go to target zone C
                            return "C";
                        }
                    }
                }
            }
        }

        // If we got this far, it means that either we did not find a ring stack (i.e. no rings in the stack
        // or something went wrong. In either case we should go for target zone A
        return "A";
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        String ringStack = analyseStack();

        if (ringStack.equals("C")) {
            // Code for what to do for target zone C goes here
            StrafeRight(DRIVE_POWER, 1000);
            DriveForwardTime(DRIVE_POWER, 5000);
        } else if (ringStack.equals("B")) {
            // Code for what to do for target zone B goes here
            StrafeLeft(DRIVE_POWER, 1000);
            DriveForwardTime(DRIVE_POWER, 5000);
        } else {
            // Code for what to do for target zone A goes here
            DriveForwardTime(DRIVE_POWER, 5000);
        }
    }

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);//still need to test motor directions for desired movement
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
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

}
