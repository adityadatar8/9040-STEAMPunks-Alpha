package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


  

@Autonomous(name = "BlitzReal")
public class BlitzReal extends LinearOpMode {
  
 
  private final static double CLAW_HOME = .3;
  private final static double CLAW_MIN_RANGE = .65;
  private final static double CLAW_MAX_RANGE = .8;
  private final static double CLAW_SPEED = 0.1;
   
  
  double clawSpeed = 0;
    
  private DcMotor frontleft;
  private DcMotor frontright;
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor slideExtender;
  private Servo claw;

 
  @Override
  public void runOpMode() {

    //initialize all motors
    frontleft = hardwareMap.get(DcMotor.class, "front left");
    frontright = hardwareMap.get(DcMotor.class, "front right");
    backleft = hardwareMap.get(DcMotor.class, "back left");
    backright = hardwareMap.get(DcMotor.class, "back right");
    slideExtender = hardwareMap.get(DcMotor.class, "slide extender");

    //initialize all servos
    claw = hardwareMap.get(Servo.class, "claw");
    

    //close claw 
    claw.setPosition(.6);
    clawSpeed -=CLAW_SPEED;
    sleep(1000);

    //extends the linear slides to it's full height
    slideExtender.setPower(-1.5);

    //strafe right
    frontright.setPower(0.5);
    backright.setPower(-0.5);
    backleft.setPower(-0.5);
    frontleft.setPower(0.5);
    sleep(1919);
    
    //move forward
    frontright.setPower(-0.2);
    backright.setPower(-0.2);
    backleft.setPower(0.2);
    frontleft.setPower(0.2);
    sleep(45);
    
    //retracts the linear slides to it's minimum height
    slideExtender.setPower(1.5);
    sleep(100);
    
    //open claw
    claw.setDirection(Servo.Direction.REVERSE);
    clawSpeed -= CLAW_SPEED;
    sleep(50);

    
  }
}
