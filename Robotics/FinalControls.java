 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp

public class Fianlcontrols extends LinearOpMode {

  private DcMotor frontl, frontr, leftb, rightb, l1, l2, flin, slin;
  private CRServo wrist;
  private Servo sclaw, fclaw, b1, b2;
  
  double maxSpeed = 1.0;

  @Override
  public void runOpMode() {

    // This creates variables for each motor based on the configuration file
    frontl = hardwareMap.get(DcMotor.class, "fl");
    frontr = hardwareMap.get(DcMotor.class, "fr");
    leftb = hardwareMap.get(DcMotor.class, "bl");
    rightb = hardwareMap.get(DcMotor.class, "br");
    l2 = hardwareMap.get(DcMotor.class, "l2");
    l1 = hardwareMap.get(DcMotor.class, "l1");
    slin = hardwareMap.get(DcMotor.class, "flin");
    flin = hardwareMap.get(DcMotor.class, "slin");
    
    sclaw = hardwareMap.get(Servo.class, "sclaw");
    fclaw = hardwareMap.get(Servo.class, "fclaw");
    b1 = hardwareMap.get(Servo.class, "b1");
    b2 = hardwareMap.get(Servo.class, "b2");
    wrist = hardwareMap.get(CRServo.class, "wrist");
   

    frontl.setDirection(DcMotor.Direction.REVERSE);
    leftb.setDirection(DcMotor.Direction.REVERSE);
    
    l1.setDirection(DcMotor.Direction.REVERSE);
    
    
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    
    

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    

    
    double openPosition = 0.572;   // Full open (adjust this value as needed)
    double closedPosition = .2;
    double openPosition2 = 0;   // Full open (adjust this value as needed)
    double closedPosition2 = .15;

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

    //linear slide controls
    if(this.gamepad2.left_bumper == true && this.gamepad2.a == true)
     {
       l1.setPower(.9);
       l2.setPower(.9);
     
     }
     else if(this.gamepad2.right_bumper == true)
     {
        l1.setPower(-.8);
        l2.setPower(-.8);

      
     }
     
     else if(this.gamepad2.left_bumper == true)
     {
       l1.setPower(.6);
       l2.setPower(.6);
     
     }
     else
     {
         l1.setPower(-.1);
         l2.setPower(-.1);
     }
        
     //other slide control
     if(this.gamepad1.right_bumper == true)
     {
        slin.setPower(.7);
        
     }
     else if(this.gamepad1.left_bumper == true)
     {
       slin.setPower(-.5);
     
     }
     else
     {
       slin.setPower(.07);
      
     }

     //claw with regular servo position code
     if (gamepad1.x) {
            sclaw.setPosition(closedPosition);  // Close the fclaw
            telemetry.addData("fclaw State", "Closing");
        }

        // Check if the B button is pressed to open the fclaw
        if (gamepad1.b) {
            sclaw.setPosition(openPosition);   // Open the fclaw
            telemetry.addData("fclaw State", "Opening");
        }

     if (gamepad1.dpad_left) {
         fclaw.setPosition(closedPosition2); 
        }
      
     if (gamepad1.dpad_right) {
         fclaw.setPosition(openPosition2); 
      }

      //servo basket controls
      if (gamepad1.dpad_down)
      {
        //b1.setPosition(1.2);
        b2.setPosition(.45);
        
      }
      else if (gamepad1.dpad_up)
      {
        //b1.setPosition(.2);
        b2.setPosition(.9);
        
      }
    
    //following code is using joysticks on controller to alter the power of the motors individually depending
    double rightJoystickY = gamepad1.right_stick_y; // Negative to match forwards being positive

    // Set the power of the flin motor based on the joystick input
    flin.setPower(rightJoystickY/2);
    
    double rightJoystickY2 = -gamepad2.right_stick_y;
    
    
        double leftTrigger = gamepad1.left_trigger / 2;  // Left trigger goes from 0 to 1
        double rightTrigger = gamepad1.right_trigger / 2;  // Right trigger goes from 0 to 1

        // If left trigger is pressed, rotate counterclockwise (negative power)
        // If right trigger is pressed, rotate clockwise (positive power)
        double wristPower = rightTrigger - leftTrigger;  // Right trigger is positive, left trigger is negative

        // Set the wrist motor power based on trigger input
        wrist.setPower(wristPower);

        // Telemetry for debugging
        telemetry.addData("Left Trigger", leftTrigger);
        telemetry.addData("Right Trigger", rightTrigger);
        telemetry.addData("Wrist Power", wristPower);
        telemetry.update();
     
     // Get joystick inputs
            double leftStickX = gamepad1.left_stick_x;  // Left stick X-axis (strafe)
            double leftStickY = -gamepad1.left_stick_y; // Left stick Y-axis (forward/backward)
            double rightStickX = gamepad1.right_stick_x; // Right stick X-axis (turning)

            // Calculate motor powers for mecanum drive
            double frontLeftPower = leftStickY + leftStickX + rightStickX;
            double frontRightPower = leftStickY - leftStickX - rightStickX;
            double backLeftPower = leftStickY - leftStickX + rightStickX;
            double backRightPower = leftStickY + leftStickX - rightStickX;

            // Apply speed scaling (this adjusts the maximum speed)
            frontLeftPower = Range.clip(frontLeftPower * maxSpeed, -.6, .6);
            frontRightPower = Range.clip(frontRightPower * maxSpeed, -.6, .6);
            backLeftPower = Range.clip(backLeftPower * maxSpeed, -.6, .6);
            backRightPower = Range.clip(backRightPower * maxSpeed, -.6, .6);

            // Set power to motors
            frontl.setPower(frontLeftPower);
            frontr.setPower(frontRightPower);
            leftb.setPower(backLeftPower);
            rightb.setPower(backRightPower);

            // Optionally, add telemetry to display joystick inputs or motor power
            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Left Stick Y", leftStickY);
            telemetry.addData("Right Stick X", rightStickX);
            
            
            //telemetry.addData("Servo 1 position", servo1.getPosition());
            //telemetry.addData("Servo 2 position", servo2.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
    }
    
    
  }
  
  
}




