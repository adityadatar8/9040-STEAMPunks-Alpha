package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TysonDoesBlocks")
public class TysonDoesBlocks extends LinearOpMode {

    private ColorSensor ColorSensor_ColorSensor;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        ColorSensor_ColorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backright = hardwareMap.get(DcMotorEx.class, "back right");
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        frontright = hardwareMap.get(DcMotorEx.class, "front right");

        // Put initialization blocks here.
        waitForStart();
        if (0 < ColorSensor_ColorSensor.blue()) {
            backleft.setPower(-0.23);
            backright.setPower(0.25);
            frontleft.setPower(-0.23);
            frontright.setPower(0.25);
            sleep(2640);
            frontright.setPower(0);
            frontleft.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        ColorSensor_ColorSensor.enableLed(true);
        if (false) {
            if (true) {
                return;
            }
        }
        if (ColorSensor_ColorSensor.blue() > ColorSensor_ColorSensor.green() && ColorSensor_ColorSensor.blue() > ColorSensor_ColorSensor.red()) {
            backleft.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);
            backright.setPower(0);
            telemetry.addData("purple", ColorSensor_ColorSensor.blue());
        } else if (ColorSensor_ColorSensor.green() > ColorSensor_ColorSensor.blue() && ColorSensor_ColorSensor.green() > ColorSensor_ColorSensor.red()) {
            backleft.setPower(-0.25);
            backright.setPower(-0.25);
            frontleft.setPower(0.25);
            frontright.setPower(0.25);
            sleep(4100);
            backleft.setPower(0);
            backright.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(-0.23);
            frontright.setPower(0.23);
            frontleft.setPower(-0.23);
            backright.setPower(0.23);
            sleep(500);
            backleft.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);
            backright.setPower(0);
            telemetry.addData("green", ColorSensor_ColorSensor.green());
        } else {
            if (ColorSensor_ColorSensor.red() > ColorSensor_ColorSensor.blue() && ColorSensor_ColorSensor.red() > ColorSensor_ColorSensor.green()) {
                backleft.setPower(0.25);
                frontleft.setPower(-0.25);
                backright.setPower(0.25);
                frontright.setPower(-0.25);
                sleep(4000);
                backleft.setPower(0);
                frontright.setPower(0);
                backright.setPower(0);
                frontleft.setPower(0);
                telemetry.addData("orange", ColorSensor_ColorSensor.red());
                backleft.setPower(-0.23);
                frontright.setPower(0.23);
                frontleft.setPower(-0.23);
                backright.setPower(0.23);
                sleep(800);
                backleft.setPower(0);
                frontleft.setPower(0);
                frontright.setPower(0);
                backright.setPower(0);
            } else {
                telemetry.addData("None Detected", ColorSensor_ColorSensor.red());
            }
        }
        telemetry.update();
    }
}
