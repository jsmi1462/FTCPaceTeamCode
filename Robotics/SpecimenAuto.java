//most comments for encoders and IMU in Basket Auto code
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

@Autonomous(name="Specimen Auto", group="Autonomous")
public class EncoderAutonomousOpMode extends LinearOpMode {

    // Motors
    private DcMotor frontl, frontr, leftb, rightb, l1, l2, flin, slin;
    private CRServo wrist;
    private Servo sclaw, fclaw;
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

        // Initialize IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Reverse left side motors
        frontl.setDirection(DcMotor.Direction.REVERSE);
        leftb.setDirection(DcMotor.Direction.REVERSE);
        frontr.setDirection(DcMotor.Direction.FORWARD);
        rightb.setDirection(DcMotor.Direction.FORWARD);

        // Reset motor encoders
        frontl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();
        
        frontl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double openPosition = 0.2;   // Full open (adjust this value as needed)
        double closedPosition = .572;
    
        waitForStart();

        // Set motors to use encoders


        // Example movements
        //driveForward(4);
        //turnToHeading(90);
        //driveBackward(2);
        //strafeRight(3);
        
        sclaw.setPosition(closedPosition);
        armUp(14.9);
        strafeRight(10.5);
        strafeRight(3);
        armDown(4.9);
        sclaw.setPosition(openPosition);
        strafeLeft(3);
        turnToAngle(150, -.62);
        turnToAngle(178, -.2);
        armDown(10.32);
        driveForward(7.4);
        sleep(50);
        strafeLeft(14);
        sleep(50);
        driveForward(3.9);
        sleep(50);
        strafeRight(21.28);//changed from 21.38
        sclaw.setPosition(closedPosition);
        sleep(700);
        armUp(5);
        strafeLeft(5);
        turnToAngle(-1, -.5);
        driveForward(14);
        // turnToAngle(-20, -.6);
        // turnToAngle(-5, -.2);
        armUp(14.2);
        strafeRight(4.95);//second speci
        armDown(8); // changed from 7.9
        sclaw.setPosition(openPosition);
        strafeLeft(4.75);
        armDown(10);
        turnToAngle(-.2, -.55);
        driveBackward(14.5);
        strafeRight(18);
        driveBackward(5.65);
        strafeLeft(23);
        
        
    }

    private void driveForward(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(targetTicks, targetTicks, targetTicks, targetTicks);
        moveToTarget(0.6);
    }

    private void driveBackward(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(-targetTicks, -targetTicks, -targetTicks, -targetTicks);
        moveToTarget(0.6);
    }

    private void strafeRight(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(targetTicks, -targetTicks, -targetTicks, targetTicks); // Correct motor directions
        moveToTarget(0.6);
    }

    private void strafeLeft(double distanceInches) {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPosition(-targetTicks, targetTicks, targetTicks, -targetTicks); // Correct motor directions
        moveToTarget(0.6);
    }
    
    private void armUp(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionArm(targetTicks);
        moveToTargetSlin(.8);
    
    }
    
    private void armDown(double distanceInches)
    {
        int targetTicks = (int) (distanceInches / INCHES_PER_TICK);
        setTargetPositionArm(-targetTicks);
        moveToTargetSlin(.8);
    
    }
    
     private void setTargetPositionArm(int slinT) {
        slin.setTargetPosition(slin.getCurrentPosition() + slinT);
        slin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


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
    private double normalizeAngle(double angleDifference) {
        while (angleDifference > 180) angleDifference -= 360;
        while (angleDifference < -180) angleDifference += 360;
            return angleDifference;
    }

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
    
    private void stopMotors() {
        frontl.setPower(0);
        frontr.setPower(0);
        leftb.setPower(0);
        rightb.setPower(0);
    }
}
