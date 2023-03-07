package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "LightWork")
public class LightWork extends LinearOpMode
{

    private ColorSensor ColorSensor_ColorSensor;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx slideExtender;
    private Servo claw;
    private boolean isPurple;
    private boolean isOrange;
    private boolean isGreen;
    private double xPos; // x position of robot on the field
    private double yPos; // y position of robot on the field
    private int slidePos; // position of the linear slides
    private ElapsedTime timer;
    private final static double CLAW_HOME = -0.3;
    private final static double CLAW_MIN_RANGE = .65;
    private final static double CLAW_MAX_RANGE = .8;
    private  final double CLAW_SPEED = 0.1;
    private final double ROBOT_SPEED = 0.35; // Speed of robot
    private final double GROUND_JUNCTION = 2.0; // Length slides must move for ground junction
    private final double LOW_JUNCTION = 13.5; // Length slides must move for low junction
    private final double MEDIUM_JUNCTION = 23.5; // Length slides must move for medium junction
    private final double HIGH_JUNCTION = 33.25; // Length slides must move for highest junction
    private final double SPOOL_DIAMETER = 1.75; // Diameter of spool for linear slides
    private final double CUP_BASE_HEIGHT = 0.75;
    BNO055IMU imu;
    private Orientation lastOrientation;
    private double currentAngle;

    private void power(double pow)
    {
        frontleft.setPower(pow);
        frontright.setPower(pow);
        backleft.setPower(pow);
        backright.setPower(pow + 0.007);
    }

    private void power(double fL, double fR, double bL, double bR)
    {
        frontleft.setPower(fL);
        frontright.setPower(fR);
        backleft.setPower(bL);
        backright.setPower(bR + 0.007);
    }

    private void stopDrive()
    {
        frontleft.setPower(0.0);
        frontright.setPower(0.0);
        backleft.setPower(0.0);
        backright.setPower(0.0);
    }

    private void forward(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        backleft.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        frontright.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        backright.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Accelerate
        /*double target = Math.abs(backleft.getTargetPosition());
        double progress = 0.0;
        while (progress < 0.25 * target)
        {
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(ROBOT_SPEED * Math.tanh(0.33 * progress));
            progress = Math.abs(backleft.getCurrentPosition());
        }*/
        double error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
        power(ROBOT_SPEED);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy() || error > 0.5)
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
            error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
            power(ROBOT_SPEED * Math.tanh(0.33*error)); // Decelerate using tanh function
        }

        // Stop Driving
        stopDrive();
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Update y position of robot
        yPos += distance;
    }

    private void backward(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));
        backleft.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));
        frontright.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));
        backright.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Accelerate
        /*double target = Math.abs(backleft.getTargetPosition());
        double progress = 0.0;
        while (progress < 0.25 * target)
        {
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(ROBOT_SPEED * Math.tanh(0.33 * progress));
            progress = Math.abs(backleft.getCurrentPosition());
        }*/
        double error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
        power(ROBOT_SPEED);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy() || error > 0.5)
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
            error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
            power(ROBOT_SPEED * Math.tanh(0.33 * error));
        }
        stopDrive();
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Update y position of robot
        yPos -= distance;
    }

    private void left(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));
        backleft.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        frontright.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        backright.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Accelerate
        /*double target = Math.abs(backleft.getTargetPosition());
        double progress = 0.0;
        while (progress < 0.25 * target)
        {
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(ROBOT_SPEED * Math.tanh(0.33 * progress));
            progress = Math.abs(backleft.getCurrentPosition());
        }*/
        double error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
        power(ROBOT_SPEED);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy() || error > 0.5)
        {
            telemetry.addData("Left", distance);
            telemetry.update();
            error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
            power(ROBOT_SPEED * Math.tanh(0.33 * error));
        }

        stopDrive();
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Update x position of robot
        xPos -= distance;
    }

    private void right(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        backleft.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));
        frontright.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));
        backright.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Accelerate
        /*double target = Math.abs(backleft.getTargetPosition());
        double progress = 0.0;
        while (progress < 0.25 * target)
        {
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(ROBOT_SPEED * Math.tanh(0.33 * progress));
            progress = Math.abs(backleft.getCurrentPosition());
        }*/
        double error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
        power(ROBOT_SPEED);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy() || error > 0.5)
        {
            telemetry.addData("Left", distance);
            telemetry.update();
            error = Math.abs(backleft.getTargetPosition() - backleft.getCurrentPosition());
            power(ROBOT_SPEED * Math.tanh(0.33 * error));
        }

        stopDrive();
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Update x position of robot
        xPos += distance;
    }

    private void identifyColor()
    {
        isGreen = isPurple = isOrange = false;
        if (ColorSensor_ColorSensor.blue() > ColorSensor_ColorSensor.green() && ColorSensor_ColorSensor.blue() > ColorSensor_ColorSensor.red())
        {
            telemetry.addData("Purple", ColorSensor_ColorSensor.blue());
            telemetry.update();
            isPurple = true;
        }
        else if (ColorSensor_ColorSensor.green() > ColorSensor_ColorSensor.blue() && ColorSensor_ColorSensor.green() > ColorSensor_ColorSensor.red())
        {
            telemetry.addData("Green", ColorSensor_ColorSensor.green());
            telemetry.update();
            isGreen = true;
        }
        else if (ColorSensor_ColorSensor.red() > ColorSensor_ColorSensor.blue() && ColorSensor_ColorSensor.red() > ColorSensor_ColorSensor.green())
        {
            telemetry.addData("Orange", ColorSensor_ColorSensor.red());
            telemetry.update();
            isOrange = true;
        }
        else
        {
            telemetry.addData("None Detected", ColorSensor_ColorSensor.red());
            telemetry.update();
            identifyColor();
        }
        telemetry.update();
    }

    private void resetAngle()
    {
        lastOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currentAngle = 0.0;
    }

    private double getAngle()
    {
        Orientation newOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double deltaTheta = newOrientation.firstAngle - lastOrientation.firstAngle;

        while (currentAngle >= 360)
        {
            currentAngle -= 360;
        }
        while (currentAngle <= -360)
        {
            currentAngle += 360;
        }
        if (deltaTheta > 180)
        {
            deltaTheta -= 360;
        }
        else if (deltaTheta <= -180)
        {
            deltaTheta += 360;
        }

        currentAngle += deltaTheta;
        lastOrientation = newOrientation;
        telemetry.addData("Gyro", newOrientation.firstAngle);
        return currentAngle;
    }

    private void counterclockwise(double degrees)
    {
        resetAngle();
        double error = filterAngle(degrees);
        while (opModeIsActive() && Math.abs(error) > 1)
        {
            double motorPower = ROBOT_SPEED * Math.tanh(0.33*error);
            power(-1*motorPower, motorPower, -1*motorPower, motorPower);
            error = Math.abs(filterAngle(degrees - getAngle()));
            telemetry.addData("error", error);
            telemetry.update();
        }
    }

    private void clockwise(double degrees)
    {
        resetAngle();
        double error = filterAngle(degrees);
        while (opModeIsActive() && Math.abs(error) > 1)
        {
            double motorPower = ROBOT_SPEED * Math.tanh(0.33 * error);
            power(motorPower, -1*motorPower, motorPower, -1*motorPower);
            error = Math.abs(filterAngle(degrees - getAngle()));
            telemetry.addData("error", error);
            telemetry.update();
        }
    }

    private void goToAngle(double degrees)
    {
        Orientation newOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double dTheta = degrees - newOrientation.firstAngle;

        if (dTheta > 180)
        {
            dTheta -= 360;
        }
        else if (dTheta <= -180)
        {
            dTheta += 360;
        }

        if (dTheta > 0)
        {
            counterclockwise(dTheta);
        }
        else
        {
            clockwise(dTheta);
        }
    }


    private double filterAngle(double angle)
    {
        if (angle > 180)
        {
            angle -= 360;
        }
        else if (angle <= -180)
        {
            angle += 360;
        }
        return angle;
    }

    private void slidesUp(int level)
    {
        // Reset encoder counts to 0
        slideExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        if (level == 0)
        {
            slidePos = (int)((GROUND_JUNCTION / (SPOOL_DIAMETER * Math.PI)) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }
        else if (level == 1)
        {
            slidePos = (int)((LOW_JUNCTION / (SPOOL_DIAMETER * Math.PI)) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }
        else if (level == 2)
        {
            slidePos = (int)((MEDIUM_JUNCTION / (SPOOL_DIAMETER * Math.PI)) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }
        else
        {
            slidePos = (int)((HIGH_JUNCTION / (SPOOL_DIAMETER * Math.PI)) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }

        // Set the motors to move to the specified positions
        slideExtender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideExtender.setPower(0.8); // set power for moving up
        while (slideExtender.isBusy())
        {
            telemetry.addData("Slides Up", level);
            telemetry.update();
            slideExtender.setPower(0.8); // continues moving up
        }
        slideExtender.setPower(0.12); // set power for staying in place
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideExtender.setPower(0.12); // set power for staying in place
    }

    private void slidesDown()
    {
        slideExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideExtender.setTargetPosition((int)(-0.8*slidePos));
        slideExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideExtender.setPower(0.3); // set power for moving down
        while (slideExtender.isBusy())
        {
            telemetry.addData("Slides", "Down");
            telemetry.update();
            slideExtender.setPower(0.3); // continues moving down
        }
        slideExtender.setPower(0.0); // stop moving
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void openClaw() // don't mess with this cause it works somehow
    {
        double clawSpeed = 0.0;
        timer.reset();
        while ((int)timer.milliseconds() < 800)
        {
            claw.setDirection(Servo.Direction.FORWARD);
            clawSpeed -= CLAW_SPEED;
            claw.setPosition(Range.clip(clawSpeed, CLAW_MIN_RANGE, CLAW_MAX_RANGE));
            telemetry.addData("claw", "open");
            telemetry.update();
        }
        /*timer.reset();
        while ((int)timer.milliseconds() < 800)
        {
            claw.setDirection(Servo.Direction.REVERSE);
            clawSpeed -= CLAW_SPEED;
            claw.setPosition(Range.clip(clawSpeed, CLAW_MIN_RANGE, CLAW_MAX_RANGE));
            telemetry.addData("claw", "close");
            telemetry.update();
        }
        claw.setPosition(0.0);*/
    }

    private void closeClaw()
    {
        double clawSpeed = 0.0;
        timer.reset();
        while ((int)timer.milliseconds() < 800)
        {
            claw.setDirection(Servo.Direction.REVERSE);
            clawSpeed -= CLAW_SPEED;
            claw.setPosition(Range.clip(clawSpeed, CLAW_MIN_RANGE, CLAW_MAX_RANGE));
            telemetry.addData("claw", "close");
            telemetry.update();
        }
        claw.setPosition(0.0);
    }

    private void grabCone(int level, double distance)
    {
        level -= 1;
        slideExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideExtender.setTargetPosition((int)((3 + level * CUP_BASE_HEIGHT / (SPOOL_DIAMETER * Math.PI)) * 537.7));
        slideExtender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideExtender.setPower(0.8);
        while (slideExtender.isBusy())
        {
            telemetry.addData("Grab Cone", level);
            telemetry.update();
            slideExtender.setPower(0.8);
        }
        slideExtender.setPower(0.0);
        slideExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideExtender.setPower(0.12);
        backwardPOV();
        backward(distance);
        openClaw();
        closeClaw();
        slideExtender.setPower(0.0);
        slidesUp(3);
    }

    private void scoreCone(double distance)
    {
        backwardPOV();
        backward(distance);
        openClaw();
        closeClaw();
        forward(distance);
        slidesDown();
    }

    private void goToPos(double x, double y)
    {
        double dx = x - xPos;
        double dy = y - yPos;
        if (dy > 0)
        {
            forward(Math.abs(dy));
        }
        else
        {
            backward(Math.abs(dy));
        }

        if (dx > 0)
        {
            right(Math.abs(dx));
        }
        else
        {
            left(Math.abs(dx));
        }
        xPos = x;
        yPos = y;
    }

    private void goToPosHor(double x, double y)
    {
        double dx = x - xPos;
        double dy = y - yPos;
        if (dx > 0)
        {
            right(Math.abs(dx));
        }
        else
        {
            left(Math.abs(dx));
        }
        if (dy > 0)
        {
            forward(Math.abs(dy));
        }
        else
        {
            backward(Math.abs(dy));
        }
        xPos = x;
        yPos = y;
    }


    // Don't worry about this for now
    // Switching the POVs messes with the coordinate system so for now the robot
    // is just gonna be in backwardPOV
    private void backwardPOV()
    {
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void forwardPOV()
    {
        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
    }


    @Override
    public void runOpMode()
    {
        ColorSensor_ColorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backwardPOV(); // Robot starts out facing backward because color sensor is on the back

        slideExtender = hardwareMap.get(DcMotorEx.class, "slide extender");
        slideExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideExtender.setDirection(DcMotorEx.Direction.REVERSE);

        // Servos are wack
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(CLAW_HOME);
        claw.setDirection(Servo.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        isPurple = false;
        isOrange = false;
        isGreen = false;

        // Coordinate system based on center point of claw (origin of grid in bottom left corner)
        // This autonomous assumes the robot is on the left side of the red alliance
        xPos = 34.5;
        yPos = 2.5;
        lastOrientation = new Orientation();
        currentAngle = 0.0;
        timer = new ElapsedTime();


        // Put initialization blocks here.
        telemetry.addData("Robot", "Ready");
        telemetry.update();
        waitForStart();

        // Go forward 20 inches before identifying color
        forward(24);//was 24 testing 26

        //sleeps to ensure proper detection
        sleep(100);


        // Identify color
        identifyColor();
        backwardPOV();
        goToPos(48, 77);
        /*slidesUp(3);
        scoreCone(2);
        slidesDown();*/

        backwardPOV();
        // Act on the identified color
        if (isOrange)
        {
            // Strafe right
            //forward(10.0);
            goToPosHor(60, 24);
            telemetry.addData("Color", "Orange");
            telemetry.update();
        }
        else if (isGreen)
        {
            // Strafe left
            //forward(10.0);
            goToPosHor(11, 24);

            telemetry.addData("Color", "Green");
            telemetry.update();

        }
        else if(isPurple)
        {
            //Stay in place
            goToPosHor(36, 24);
            telemetry.addData("Color", "Purple");
            telemetry.update();
        }


        // Cut power just in case
        stopDrive();
    }
}