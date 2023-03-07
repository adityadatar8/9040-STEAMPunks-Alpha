package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Assigns the name for this code that will appear in the driver hub
@TeleOp(name = "thiscodeisbroken")

public class thiscodeisbroken extends LinearOpMode {

    //Declares all DcMotors and Servos
    private DcMotorEx frontleft;
    private DcMotorEx backright;
    private DcMotorEx backleft;
    private DcMotorEx frontright;
    private DcMotorEx slideExtender;
    private Servo claw;
    private Servo turner;

    //Declares variables for Claw
    private final static double CLAW_HOME = 0.5;
    private final static double CLAW_MIN_RANGE = 0.6;
    private final static double CLAW_MAX_RANGE = 0.15;
    private  final double CLAW_SPEED = 0.1;

    //Declares variables for Turner
    private final static double TURNER_HOME = 0.1;
    private final static double TURNER_MIN_RANGE = 0.1;
    private final static double TURNER_MAX_RANGE = 0.9;
    private final double TURNER_SPEED = 0.1;

    //Declares a timer;
    private ElapsedTime timer;

    //Method to set motor power for all drivetrain motors
    public void setMotorsPower(double FrontLeft,double BackLeft, double BackRight, double FrontRight){

        frontleft.setPower((-1*FrontLeft));
        backleft.setPower((-1*BackLeft));
        backright.setPower(BackRight);
        frontright.setPower(FrontRight);
    }

    //Essentially the main method for the class
    @Override
    public void runOpMode() {

        //Maps the DcMotors and Servos to their respective names stated in the driver hub
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backright = hardwareMap.get(DcMotorEx.class, "back right");
        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        slideExtender = hardwareMap.get(DcMotorEx.class, "slide extender");
        claw = hardwareMap.get(Servo.class, "claw");
        turner = hardwareMap.get(Servo.class, "turner");

        //Assigns the initial position and speed of the claw
        double clawPosition = CLAW_HOME;
        double clawSpeed = 0;

        //Assigns the initial position and speed of the claw
        double turnerPosition = TURNER_HOME;
        double turnerSpeed = 0;

        //creates a new ElapsedTime object for the timer
        timer = new ElapsedTime();

        waitForStart();
        boolean speedMode = true;
        double pow;
        turner.setPosition(TURNER_HOME);

        while (opModeIsActive()) {

            pow = 0.47;

            //Conditions for Controller1/Gamepad1 controls
            boolean forward = gamepad1.left_stick_y > 0.1;
            boolean backward = gamepad1.left_stick_y < -0.1;
            boolean left = gamepad1.right_stick_x > 0.1;
            boolean right = gamepad1.right_stick_x < -0.1;
            boolean strafeLeft = gamepad1.left_bumper;
            boolean strafeRight = gamepad1.right_bumper;
            boolean stop = gamepad1.a;
            boolean turnleft180 = gamepad1.dpad_left;
            boolean turnright180 = gamepad1.dpad_right;
            boolean decelerate = gamepad1.a;
            boolean slowMode = gamepad1.left_trigger > 0.25;

            //Conditions for Controller2/Gamepad2 controls
            boolean up = gamepad2.right_trigger>0.25;
            boolean down = gamepad2.dpad_down;
            boolean stay = gamepad2.left_trigger>0.25;
            boolean clawOpen = gamepad2.a;
            boolean turnForward = gamepad2.x;
            boolean turnBackward = gamepad2.b;


            //Condition for stopping the movement of Controller1
            boolean isZero = true;

            //Condition for stopping the movement of Controller2
            boolean slideStop = gamepad2.left_bumper;




            //Increased the speed if condition was met
            if(speedMode){pow = 0.47;}

            //Decreased the speed if condition was met and increased the speed otherwise
            if(slowMode){pow = 0.25;}
            else{pow = 0.47;}

            //closes claw
            claw.setDirection(Servo.Direction.FORWARD);

            //--------------------------------------StopBot
            if(stop){
                isZero = false;
                setMotorsPower(0,0,0,0);
                telemetry.update();
            }
            //--------------------------------------Forward
            else if(forward){
                isZero = false;
                setMotorsPower(pow,pow,pow,pow);
                telemetry.update();
            }
            //--------------------------------------Backward
            else if(backward){
                isZero = false;
                setMotorsPower(-pow, -pow, -pow, -pow);
                telemetry.update();
            }
            //--------------------------------------Left
            else if(left){
                isZero = false;
                setMotorsPower(-pow,-pow,pow,pow);
                telemetry.update();
            }
            //--------------------------------------Right
            else if(right){
                isZero = false;
                setMotorsPower(pow,pow,-pow, -pow);
                telemetry.update();
            }
            //--------------------------------------StrafeLeft
            else if(strafeLeft){
                isZero = false;
                setMotorsPower(pow,-pow,pow,-pow);
                telemetry.update();
            }
            //--------------------------------------StrafeRight
            else if(strafeRight){
                isZero = false;
                setMotorsPower(-pow,pow,-pow,pow);
                telemetry.update();
            }
            //--------------------------------------Decelerate
            else if (decelerate)
            {
                while (backleft.getVelocity() > 0.5)
                {
                    double state = Math.tanh(0.32 + 0.05 * backleft.getVelocity());
                    backleft.setVelocity(state);
                    backright.setVelocity(state);
                    frontleft.setVelocity(state);
                    frontright.setVelocity(state);
                    telemetry.update();
                }
                setMotorsPower(0, 0, 0, 0);
                telemetry.update();
            }
            //--------------------------------------TurnLeft180
            else if(turnleft180){
                isZero = false;
                timer.reset();
                while (timer.milliseconds() < 600)
                {
                    setMotorsPower(-1,-1,1,1);
                    telemetry.update();
                }
                setMotorsPower(0, 0, 0, 0);
                speedMode = true;
            }
            //--------------------------------------TurnRight180
            else if(turnright180){
                isZero = false;
                timer.reset();
                while (timer.milliseconds() < 600)
                {
                    setMotorsPower(1,1,-1,-1);
                    telemetry.update();
                }
                setMotorsPower(0, 0, 0, 0);
                speedMode = true;
            }

            //--------------------------------------StopController1Movements
            if(isZero){
                setMotorsPower(0,0,0,0);
                telemetry.update();
            }


            //--------------------------------------LinearSlidesUp
            if(up){
                isZero = false;
                slideExtender.setPower(-0.8);
                telemetry.update();
            }
            //--------------------------------------LinearSlidesStay
            else if(stay){
                isZero = false;
                slideExtender.setPower(-0.15);
                telemetry.update();
            }
            //--------------------------------------LinearSlidesDown
            else if(down){
                isZero = false;
                slideExtender.setPower(.5);
                telemetry.update();
            }
            //--------------------------------------StopSlides
            else if (slideStop)
            {
                slideExtender.setPower(0);
                clawSpeed = 0;
                telemetry.update();
            }

            //--------------------------------------ClawOpen
            if(clawOpen){
                isZero = false;
                //claw.setDirection(Servo.Direction.REVERSE);
                //clawSpeed -= CLAW_SPEED;
                claw.setPosition(CLAW_MAX_RANGE);
                telemetry.update();
            }
            //--------------------------------------ClawClose
            else {
                isZero = false;
                //claw.setDirection(Servo.Direction.FORWARD);
                //clawSpeed -= CLAW_SPEED;
                claw.setPosition(CLAW_HOME);
                telemetry.update();
            }


            //--------------------------------------TurnerTurn
            if(turnForward){
                isZero = false;
                //turner.setDirection(Servo.Direction.REVERSE);
                //turnerSpeed -= TURNER_SPEED;
                turner.setPosition(TURNER_MAX_RANGE);
                telemetry.update();
            }
            else if(turnBackward){
                isZero = false;
                //turner.setDirection(Servo.Direction.FORWARD);
                //turnerSpeed -= TURNER_SPEED;
                turner.setPosition(TURNER_HOME);
                telemetry.update();
            }

            //--------------------------------------StopsController2Movements
            if(isZero){
                slideExtender.setPower(0);
                clawSpeed = 0;
                telemetry.update();
            }


            //clawPosition = Range.clip(clawSpeed, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
            //claw.setPosition(clawPosition);
            //turnerPosition = Range.clip(turnerSpeed, TURNER_MIN_RANGE, TURNER_MAX_RANGE);
            //turner.setPosition(turnerPosition);
            telemetry.update();
        }
    }
}

