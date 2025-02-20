package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;



import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Basket Auto", group="Autonomous")
public class EncoderAutoBasket extends LinearOpMode {

    // Motors
    private DcMotor frontl, frontr, leftb, rightb, l1, l2, flin, slin;
    private CRServo wrist;
    private Servo sclaw, fclaw, b2;
    private BHI260IMU imu; // IMU sensor for heading

    // Movement constants
    private static final double WHEEL_DIAMETER = 4.0; // 
    //inches
    private static final double TICKS_PER_REVOLUTION = 1440;
    private static final double INCHES_PER_TICK = (Math.PI * WHEEL_DIAMETER) / TICKS_PER_REVOLUTION;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
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
        wrist = hardwareMap.get(CRServo.class, "wrist");
        b2 = hardwareMap.get(Servo.class, "b2");

        // Initialize IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Reverse motors
        frontl.setDirection(DcMotor.Direction.REVERSE);
        leftb.setDirection(DcMotor.Direction.REVERSE);
        frontr.setDirection(DcMotor.Direction.FORWARD);
        rightb.setDirection(DcMotor.Direction.FORWARD);
        l1.setDirection(DcMotor.Direction.REVERSE);

        // Reset motor encoders
        frontl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();
        
        frontl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double openPosition = 0.2;   // Full open (adjust this value as needed)
        double closedPosition = .572;
        double closedPosition2 = 0;   // Full open (adjust this value as needed)
        double openPosition2 = .15;
        
    
        waitForStart();

        // Set motors to use encoders


        // Example movements
        //driveForward(4);
        //turnToHeading(90);
        //driveBackward(2);
        //strafeRight(3);


        //start of auto running
        fclaw.setPosition(openPosition2); 
        strafeLeft(3.6);
        driveBackward(2.2);
        //basket up
        slideUp(38.9);
        b2.setPosition(.43);
        sleep(2000);
        b2.setPosition(.9);
        slideDown(38.75);
        //change here to fix block pickup and 2nd part below
        driveForward(2);
        strafeLeft(2.5);
        turnToAngle(42, -.5);
        wrist.setPower(.06);
        //side arm reach forward
        flinUp(12.2);
        wrist.setPower(.4);
        sleep(650);
        wrist.setPower(0);
        sleep(300);
        fclaw.setPosition(closedPosition2); 
        sleep(800);
        wrist.setPower(-.4);
        sleep(625);
        wrist.setPower(.055);
        //side arm reach back
        flinDown(12.86);
        wrist.setPower(-.2);
        fclaw.setPosition(openPosition2);
        sleep(1000);
        wrist.setPower(.06);
        sleep(1000);
        //2nd half change here to fix block pickup
        turnToAngle(0, -.5);
        strafeLeft(-2.5);
        driveForward(-2);
        slideUp(40);
        b2.setPosition(.43);
        sleep(2000);
        b2.setPosition(.9);
        slideDown(38.7);

        
        

        
    }

    //sets targetPosition for motors
    private void driveForward(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(targetTicks, targetTicks, targetTicks, targetTicks);
        moveToTarget(0.6);
    }

    //same as above
    private void driveBackward(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(-targetTicks, -targetTicks, -targetTicks, -targetTicks);
        moveToTarget(0.6);
    }

    //same as above
    private void strafeRight(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(targetTicks, -targetTicks, -targetTicks, targetTicks); // Correct motor directions
        moveToTarget(0.6);
    }

    //same as above
    private void strafeLeft(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(-targetTicks, targetTicks, targetTicks, -targetTicks); // Correct motor directions
        moveToTarget(0.6);
    }

    //same as above
    private void armUp(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionArm(targetTicks);
        moveToTargetSlin(.8);
    
    }

    //same as above
    private void armDown(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionArm(-targetTicks);
        moveToTargetSlin(.8);
    
    }

    //same as above
    private void setTargetPositionArm(int slinT) {
        slin.setTargetPosition(slin.getCurrentPosition() + slinT);
        slin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //changes motors to run to the position previously set
    private void setTargetPosition(int flTicks, int frTicks, int lbTicks, int rbTicks) {
        frontl.setTargetPosition(frontl.getCurrentPosition() + flTicks);
        frontr.setTargetPosition(frontr.getCurrentPosition() + frTicks);
        leftb.setTargetPosition(leftb.getCurrentPosition() + lbTicks);
        rightb.setTargetPosition(rightb.getCurrentPosition() + rbTicks);
        frontl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //sets the power for slide to move to position
    private void moveToTargetSlin(double power)
    {
        slin.setPower(power);
        while (opModeIsActive() && slin.isBusy())
        {
            telemetry.addData("Moving", "Slide running...");
            telemetry.update();
        }
        slin.setPower(0.05);
    }

    //sets the power for motors to move to position previously set
    private void moveToTarget(double power) {
        frontl.setPower(power);
        frontr.setPower(power);
        leftb.setPower(power);
        rightb.setPower(power);
        while (opModeIsActive() && frontl.isBusy() && frontr.isBusy() && leftb.isBusy() && rightb.isBusy()) {
            telemetry.addData("Moving", "Motors running...");
            telemetry.update();
        }
        stopMotors();
    }

    //I dont know
    private double normalizeAngle(double angleDifference) {
        while (angleDifference > 180) angleDifference -= 360;
        while (angleDifference < -180) angleDifference += 360;
            return angleDifference;
    }

    //sets bot to turn to angle of IMU position, depends on control hub orientation
    //power portion is messed up and bot randomly chooses turning direction based on both negative or positive 
    private void turnToAngle(double targetAngle, double power) {
        double currentAngle = getHeading();
        double angleDifference = targetAngle - currentAngle;
    
        angleDifference = normalizeAngle(angleDifference);
        
        double turnPower = Math.signum(angleDifference) * (power);

        frontl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Continue turning until the robot is within 1 degree of the target angle
        while (opModeIsActive() && Math.abs(angleDifference) > 1) {
            currentAngle = getHeading();
            angleDifference = targetAngle - currentAngle;

            angleDifference = normalizeAngle(angleDifference);

                frontl.setPower(turnPower);
                frontr.setPower(-turnPower);
                leftb.setPower(turnPower);
                rightb.setPower(-turnPower);

            // Debugging telemetry
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Angle Difference", angleDifference);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }

        // Stop the motors once the target angle is reached
        frontl.setPower(-turnPower);
        frontr.setPower(turnPower);
        leftb.setPower(-turnPower);
        rightb.setPower(turnPower);
        sleep(50);
        
        frontl.setPower(0);
        frontr.setPower(0);
        leftb.setPower(0);
        rightb.setPower(0);
        
        frontl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    private void flinUp(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionFlin(-targetTicks);
        moveToTargetFlin(.6);
    
    }
    
    private void flinDown(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionFlin(targetTicks);
        moveToTargetFlin(.6);
    }
    
     private void setTargetPositionFlin(int flinT) {
        flin.setTargetPosition(flin.getCurrentPosition() + flinT);
        flin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    private void moveToTargetFlin(double power)
    {
        flin.setPower(power);
        while (opModeIsActive() && flin.isBusy())
        {
            telemetry.addData("Moving", "Slide running...");
            telemetry.update();
        }
        flin.setPower(0);
    }
    
    private void slideUp(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionSlide(-targetTicks);
        moveToTargetSlide(.6);
    
    }
    
    private void slideDown(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionSlide(targetTicks);
        moveToTargetSlide(.6);
    
    }
    
     private void setTargetPositionSlide(int slideT) {
        l1.setTargetPosition(l1.getCurrentPosition() + slideT);
        l2.setTargetPosition(l2.getCurrentPosition() + slideT);
        l1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        l2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    private void moveToTargetSlide(double power)
    {
        l1.setPower(power);
        l2.setPower(power);
        while (opModeIsActive() && l1.isBusy() && l2.isBusy())
        {
            telemetry.addData("Moving", "Slide running...");
            telemetry.update();
        }
        l1.setPower(0.05);
        l2.setPower(0.05);
    }

    //stops all motors
    private void stopMotors() {
        frontl.setPower(0);
        frontr.setPower(0);
        leftb.setPower(0);
        rightb.setPower(0);
    }
}
