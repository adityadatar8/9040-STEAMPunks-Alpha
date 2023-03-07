package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import java.io.File;

import java.util.List;

@Autonomous(name = "VuforiaTest")
public class VuforiaTest extends LinearOpMode
{
    //private static final String TFOD_MODEL = "secondModel.tflite"; // for secondModel (Cup vs. NoCup)
    private static final String TFOD_MODEL = "POG.tflite"; // for POG (Purple, Orange, Green)

    //private static final String[] LABELS = {"Cup", "NoCup"}; // for secondModel
    private static final String[] LABELS = {"Orange", "Green", "Purple"}; // for POG

    private static final String VUFORIA_KEY =
            "AZT1B8D/////AAABmT2pn3VwEkaGtYCADVhRAWk6ni9EfaE/artjzjPnlASY6ItH3QfKaR/+a7RxPseAm0dxIehk6oCezM6+j/cv9ZxbI/fnvvMvI8t9yhb1vwvlNmwWhp59piQEnSAXm8xDlBrWsQVClmWnASCgyn8upaRNCRXYaiwMwHihKXt5r1COsD679Q/p5cGC0qm7PXlSELrebqEDurulLIwdrkcZeGXiZZYVZ/IToj0DphoR5wzbI7JyQa0pIfLqdUfdIbn43ghAcYB8dhyHswudzYK0Cc6Zlf1wCzZaE58r9PcUEVKUYlPDylaECYTQCljm7y/FBQvND2p9rUD30upWRsKL3D1cPN/SIwnl075GoxQra0a3";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;

    @Override
    public void runOpMode()
    {
        int startTime;
        int endTime;
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setDirection(Direction.FORWARD);
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setDirection(Direction.FORWARD);
        backright = hardwareMap.get(DcMotor.class, "back right");
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setDirection(Direction.REVERSE); // right motors go reverse
        frontright = hardwareMap.get(DcMotor.class, "front right");
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setDirection(Direction.REVERSE); // right motors go reverse
        initVuforia();
        telemetry.addData("initVuforia", "Successful");
        telemetry.update();
        initTfod();
        telemetry.addData("initTfod", "Successful");
        telemetry.update();

        if (tfod != null)
        {
            tfod.activate();
            tfod.setZoom(10.0, 16.0/9.0);
        }
        telemetry.addData("tfod.activate()", "successful");
        telemetry.update();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        startTime = (int)(System.nanoTime() / 1000000000); // the time the opMode starts in seconds

        if (opModeIsActive())
        {
            endTime = (int)(System.nanoTime() / 1000000000); // initialize the endTime to current time so that elapsed time can be computed
            while (endTime - startTime <= 29) // if 29 seconds have passed, none of the stuff below should execute
            {
                if (tfod != null)
                {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); // puts the objects TensorFlow detects into a list
                    if (updatedRecognitions != null) // only executes if there are objects detected
                    {   /* code for secondModel
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions)
                        {
                            telemetry.addData("" + recognition.getLabel(), "" + recognition.getConfidence() * 100 + "%");
                            if (recognition.getLabel().equals("Cup"))
                            {
                                telemetry.addData("Object", "Cup");
                            }
                        }
                        telemetry.update();  */

                        // code for POG
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions)
                        {
                            telemetry.addData("" + recognition.getLabel(), "" + recognition.getConfidence() * 100 + "%");
                            if (recognition.getLabel().equals("Orange"))
                            {
                                telemetry.addData("Cup", "Orange"); // update telemetry

                                // Set target distance to 20 inches using encoders
                                frontleft.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
                                backleft.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));
                                frontright.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));
                                backright.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));

                                // Initialize the motors to go to the target positions
                                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                // Strafe right
                                backleft.setPower(-0.25);
                                frontright.setPower(-0.25);
                                backright.setPower(0.25);
                                frontleft.setPower(0.25);
                            }
                            else if (recognition.getLabel().equals("Green"))
                            {
                                telemetry.addData("Cup", "Green"); // update telemetry

                                // Set target distance to 20 inches using encoders
                                frontleft.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));
                                backleft.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
                                frontright.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
                                backright.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));

                                // Initialize motors to go to the target positions
                                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                // Strafe left
                                backleft.setPower(0.25);
                                frontright.setPower(0.25);
                                backright.setPower(-0.25);
                                frontleft.setPower(-0.25);
                            }
                            else
                            {
                                // update telemetry and stay put
                                telemetry.addData("Cup", "Purple"); // update telemetry and stay put
                            }
                        }
                        telemetry.update();
                        // Program finds out which color is detected and updates the telemetry accordingly
                    }
                    else
                    {
                        telemetry.addData("updatedRecognitions", "null");
                        telemetry.update(); // gives data to telemetry so that we can identify the error
                    }
                }
                else
                {
                    telemetry.addData("tfod object", "null");
                    telemetry.update(); // gives data to telemetry so that we can identify the error
                }

                // Cut off power just in case
                backleft.setPower(0.0);
                frontright.setPower(0.0);
                backright.setPower(0.0);
                frontleft.setPower(0.0);

                // recalculates endtime so that elapsed time can be updated for the while loop
                endTime = (int)(System.nanoTime() / 1000000000);
            }
        }
    }

    private void initVuforia()
    {
        // Gives Vuforia access to the camera stream through the camera id (hopefully)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Gets camera monitor id and initializes a vuforia parameters object with the id so that camera preview is enabled
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Cool Cam");

        // Sets the localization to be based on x, y, and z axes
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.useExtendedTracking = true; // continues tracking even after the object moves away

        // initializes vuforia with all specified parameters
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        // Gives tfod access to camera data from vuforia
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        // Objects must be 50% or higher to be considered recognized objects
        tfodParameters.minResultConfidence = 0.50f;

        // Makes a decision after collecting 300 frames from camera stream (5 seconds for 60 fps)
        tfodParameters.inputSize = 300;
        tfodParameters.useObjectTracker = true; // enables movement tracking for objects
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        telemetry.addData("Everything but model file", "successful");
        telemetry.update();

        // To User: Make sure the model is uploaded to FTC Remote Control Dashboard
        //          and modify the name above in the variable TF_MODEL
        tfod.loadModelFromFile(TFOD_MODEL, LABELS);
        telemetry.addData("Model", "successful");
        telemetry.update();
    }
}
