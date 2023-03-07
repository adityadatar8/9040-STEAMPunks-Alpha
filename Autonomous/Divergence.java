package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Divergence extends LinearOpMode
{
    private ColorSensor colorSensor;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx slideExtender;
    private Servo claw;
    private Servo turner;

    private boolean isPurple;
    private boolean isOrange;
    private boolean isGreen;

    private double xPos;
    private double yPos;
    private int slidePos;
    private boolean turnerPos; // boolean variable to save on memory and increase efficiency

    private ElapsedTime timer;

    private final static double CLAW_HOME = 0.5;
    private final static double CLAW_MAX_RANGE = 0.15;

    private final double TURNER_HOME = 0.1;
    private final double TURNER_MAX_RANGE = 0.9;
    private final double GROUND_JUNCTION_HEIGHT = 2.0; // Length slides must move for ground junction in centimeters
    private final double LOW_JUNCTION_HEIGHT = 13.5; // Length slides must move for low junction in centimeters
    private final double MEDIUM_JUNCTION_HEIGHT = 23.5; // Length slides must move for medium junction in centimeters
    private final double HIGH_JUNCTION_HEIGHT = 35.00; // Length slides must move for highest junction in centimeters
    private final double DISTANCE_PER_ROTATION = 1.20; // Distance traveled per rotation of slide motor in centimeters
                                                      // Advertised to be 120mm on GoBilda website
    private final double CUP_BASE_HEIGHT = 0.75; // Height of the base of the cone

    private final double START_POSITION_X = 31.35; // inches from left wall to center point of claw
    private final double START_POSITION_Y = 2.5; // inches from back wall to center point of claw
    private final double SLIDES_IN_PLACE_POWER = 0.12;
    private final double SLIDES_UP_POWER = 0.8;
    private final double SLIDES_DOWN_POWER = 0.3;
    private final double SLIDES_SAFETY_INTERVAL = 3000; // Reference time for the slides' emergency stop in milliseconds


    @Override
    public void runOpMode()
    {
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        // May need to change this depending on how robot behaves
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);

        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");

        slideExtender = hardwareMap.get(DcMotorEx.class, "slide extender");
        slideExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideExtender.setDirection(DcMotorEx.Direction.REVERSE);

        // Servos are wack
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(CLAW_HOME);

        turner = hardwareMap.get(Servo.class, "turner");
        turner.setDirection(Servo.Direction.FORWARD);
        turner.setPosition(TURNER_HOME);

        isPurple = false;
        isOrange = false;
        isGreen = false;

        // Coordinate system based on center point of claw (origin of grid in bottom left corner)
        // This autonomous assumes the robot is on the left side
        xPos = START_POSITION_X;
        yPos = START_POSITION_Y;
        slidePos = 0;
        turnerPos = false; // starts out facing toward the drivers
        timer = new ElapsedTime();

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        goToX(34.5); // Robot starts out touching line, then centers itself in the tile

        forward(24); // Go forward 20 inches before identifying color

        sleep(100); //sleeps to ensure proper detection

        identifyColor(); // Identify color on signal cone

        goToY(55.65); // Go to Y coordinate of pole

        // Push cone out of the way
        forward(6);
        backward(6);

        // Go to pole and score cone
        goToX(78.25);
        scoreCone(3);

        // Act on the identified color
        if (isOrange)
        {
            // Right parking spot
            goToX(62);
            //goToY(40);
            telemetry.addData("Color", "Orange");
            telemetry.update();
        }
        else if (isGreen)
        {
            // Left parking spot
            goToX(11);
            //goToY(36);
            telemetry.addData("Color", "Green");
            telemetry.update();

        }
        else if(isPurple)
        {
            // Middle Parking Spot
            goToX(36);
            //goToY(36);
            telemetry.addData("Color", "Purple");
            telemetry.update();
        }

        // Cut power just in case
        power(0.0);
    }

    private void power(double pow)
    {
        frontleft.setPower(pow);
        frontright.setPower(pow);
        backleft.setPower(pow);
        backright.setPower(pow);
    }

    private void power(double fL, double fR, double bL, double bR)
    {
        frontleft.setPower(fL);
        frontright.setPower(fR);
        backleft.setPower(bL);
        backright.setPower(bR);
    }

    private void forward(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
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

        power(0.25); // Initial power applied
        double progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI); // number of inches traveled so far
        while (Math.abs((double)(backleft.getCurrentPosition()) / (double)(backleft.getTargetPosition())) < 0.5)
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        }

        // Cut off power
        power(0.0);
        sleep(100);
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

        power(0.25); // Initial power applied
        double progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI); // number of inches traveled so far
        while (Math.abs((double)(backleft.getCurrentPosition()) / (double)(backleft.getTargetPosition())) < 0.5)
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        }

        // Cut off power
        power(0.0);
        sleep(100);
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

        power(0.25); // Initial power applied
        double progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI); // number of inches traveled so far
        while (Math.abs((double)(backleft.getCurrentPosition()) / (double)(backleft.getTargetPosition())) < 0.5)
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        }

        // Cut off power
        power(0.0);
        sleep(100);
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

        power(0.25); // Initial power applied
        double progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI); // number of inches traveled so far
        while (Math.abs((double)(backleft.getCurrentPosition()) / (double)(backleft.getTargetPosition())) < 0.5)
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = Math.abs((backleft.getCurrentPosition() / 537.7) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(((backleft.getTargetPosition() - backleft.getCurrentPosition()) / 537.7) * Math.PI);
        }

        // Cut off power
        power(0.0);
        sleep(100);
        xPos += distance;
    }

    private void identifyColor()
    {
        isGreen = isPurple = isOrange = false;
        if (colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.red())
        {
            telemetry.addData("Purple", colorSensor.blue());
            telemetry.update();
            isPurple = true;
        }
        else if (colorSensor.green() > colorSensor.blue() && colorSensor.green() > colorSensor.red())
        {
            telemetry.addData("Green", colorSensor.green());
            telemetry.update();
            isGreen = true;
        }
        else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green())
        {
            telemetry.addData("Orange", colorSensor.red());
            telemetry.update();
            isOrange = true;
        }
        else
        {
            telemetry.addData("None Detected", colorSensor.red());
            telemetry.update();
            identifyColor(); // Recursive call, keeps checking until color is detected
        }
        telemetry.update();
    }

    private void slidesUp(int level)
    {
        // Reset encoder counts to 0
        slideExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        if (level == 0)
        {
            slidePos = (int)((GROUND_JUNCTION_HEIGHT / DISTANCE_PER_ROTATION) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }
        else if (level == 1)
        {
            slidePos = (int)((LOW_JUNCTION_HEIGHT / DISTANCE_PER_ROTATION) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }
        else if (level == 2)
        {
            slidePos = (int)((MEDIUM_JUNCTION_HEIGHT / DISTANCE_PER_ROTATION) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }
        else
        {
            slidePos = (int)((HIGH_JUNCTION_HEIGHT / DISTANCE_PER_ROTATION) * 537.7);
            slideExtender.setTargetPosition(slidePos);
        }

        // Set the motors to move to the specified positions
        slideExtender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        timer.reset();
        boolean emergencyStop = false;
        slideExtender.setPower(SLIDES_UP_POWER); // set power for moving up
        while (slideExtender.isBusy() && !(emergencyStop))
        {
            telemetry.addData("Slides Up", level);
            telemetry.update();
            slideExtender.setPower(SLIDES_UP_POWER); // continues moving up
            // Emergency stop after some amount of time has passed
            emergencyStop = (int)timer.milliseconds() > SLIDES_SAFETY_INTERVAL;
        }

        // Cut off power momentarily and switch modes
        power(0.0);
        slideExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slideExtender.setPower(SLIDES_IN_PLACE_POWER); // set power for staying in place

        if (emergencyStop) // Updates telemetry for testing purposes
        {
            telemetry.addData("Emergency Stop", "True");
            telemetry.update();
        }
    }

    private void slidesDown()
    {
        // reset encoder counts to 0
        slideExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set target position for encoder
        slideExtender.setTargetPosition((int)(-0.8*slidePos)); // experiment with this number if needed
        slideExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();
        boolean emergencyStop = false;
        slideExtender.setPower(SLIDES_DOWN_POWER); // set power for moving down
        while (slideExtender.isBusy() && !(emergencyStop))
        {
            telemetry.addData("Slides", "Down");
            telemetry.update();
            slideExtender.setPower(SLIDES_DOWN_POWER); // continues moving down
            // Emergency stop after some amount of time has passed
            emergencyStop = (int)timer.milliseconds() > SLIDES_SAFETY_INTERVAL;
        }

        // Cut off power
        slideExtender.setPower(0.0);
        slideExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidePos = 0; // update position of linear slides
        if (emergencyStop) // Updates telemetry for testing purposes
        {
            telemetry.addData("Emergency Stop", "True");
            telemetry.update();
        }
    }

    private void openClaw() {claw.setPosition(CLAW_MAX_RANGE);}

    private void closeClaw() {claw.setPosition(CLAW_HOME);}

    private void turnerTurn()
    {
        if (turnerPos) {turner.setPosition(TURNER_MAX_RANGE);}
        else {turner.setPosition(TURNER_HOME);}
    }

    private void scoreCone(double distance)
    {
        //backward(distance);
        slidesUp(3);
        sleep(1200);
        backward(distance -0.5);
        sleep(200);
        openClaw();
        sleep(350);
        closeClaw();
        forward(distance-0.35);
        slidesDown();
    }

    private void goToX(double x) // goes to the vertical line (x coordinate) specified
    {
        double dx = x - xPos;
        right(dx);
    }
    private void goToY(double y) // goes to the horizontal line (y coordinate) specified
    {
        double dy = y - yPos;
        forward(dy);
    }

    private double acceleratorTransform(double input) {return Math.tanh(0.35 + 0.05 * input);}
}
