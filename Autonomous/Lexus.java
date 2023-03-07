package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Lexus")
public class Lexus extends LinearOpMode
{
    private ColorSensor colorSensor;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx slideExtender;
    private Servo claw;
    private boolean isPurple;
    private boolean isOrange;
    private boolean isGreen;
    private double xPos;
    private double yPos;
    private int slidePos;
    private ElapsedTime timer;
    private final static double CLAW_HOME = -0.3;
    private final static double CLAW_MIN_RANGE = .65;
    private final static double CLAW_MAX_RANGE = .8;
    private  final double CLAW_SPEED = 0.1;
    private final double ROBOT_SPEED = 0.5; // Speed of robot
    private final double GROUND_JUNCTION = 2.0; // Length slides must move for ground junction
    private final double LOW_JUNCTION = 13.5; // Length slides must move for low junction
    private final double MEDIUM_JUNCTION = 23.5; // Length slides must move for medium junction
    private final double HIGH_JUNCTION = 35.00; // Length slides must move for highest junction
    private final double SPOOL_DIAMETER = 1.75; // Diameter of spool for linear slides
    private final double CUP_BASE_HEIGHT = 0.75;

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
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(CLAW_HOME);
        claw.setDirection(Servo.Direction.FORWARD);

        isPurple = false;
        isOrange = false;
        isGreen = false;

        // Coordinate system based on center point of claw (origin of grid in bottom left corner)
        // This autonomous assumes the robot is on the left side
        xPos = 31.35; // inches from left wall to center point of claw
        yPos = 2.5; // inches from back wall to center point of claw
        timer = new ElapsedTime();

        telemetry.addData("Roadster", "Ready");
        telemetry.update();
        waitForStart();

        goToX(34.5);
        // Go forward 20 inches before identifying color
        forward(24);

        //sleeps to ensure proper detection
        sleep(100);

        // Identify color
        identifyColor();

        // If something goes terribly wrong with the coordinate system,
        // just try hard coding the whole thing with forward, backward, left, and right
        // Coordinate system just makes it easier to code everything
        goToY(55.65);
        forward(6);
        backward(6);
        goToX(78.25);
        scoreCone(4);

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

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
        }*/
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
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


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

        // If the wonderful math stuff doesn't work
        /*
        // give power to motors
        power(ROBOT_SPEED);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
        }*/
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
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code

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

        // If the wonderful math stuff doesn't work
        /*
        // give power to motors
        power(ROBOT_SPEED);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
        }*/
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
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code

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

        // If the wonderful math stuff doesn't work
        /*
        // give power to motors
        power(ROBOT_SPEED);
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
        }*/
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
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code

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
        }

        // Cut off power momentarily and switch modes
        power(0.0);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slideExtender.setPower(0.12); // set power for staying in place
    }

    private void slidesDown()
    {
        // reset encoder counts to 0
        slideExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set target position for encoder
        slideExtender.setTargetPosition((int)(-0.5*slidePos)); // experiment with this number if needed
        slideExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideExtender.setPower(0.3); // set power for moving down
        while (slideExtender.isBusy())
        {
            telemetry.addData("Slides", "Down");
            telemetry.update();
            slideExtender.setPower(0.3); // continues moving down
        }

        // Cut off power
        slideExtender.setPower(0.0);
        slideExtender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidePos = 0; // update position of linear slides
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
    }

    private void closeClaw() // don't mess with this cause it works somehow
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
        //closeClaw();
    }

    // This is basically the goToPos() method split up into two parts
    // You can use the goToX and goToY methods in different orders depending
    // on how the robot has to move
    // For example, the robot may have to strafe left before going backward to avoid hitting a pole
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

    private double acceleratorTransform(double input)
    {
        return Math.tanh(0.35 + 0.05 * input);
        // Play around with these numbers until they work
        // 0.6 is where the maximum possible power for the robot should go
        // 0.25 is where the minimum possible power for the robot should go
        // 0.05 is how much the power should increase/decrease for every inch traveled
        // input should be the number of inches traveled or the number of inches away from the target
    }

}
