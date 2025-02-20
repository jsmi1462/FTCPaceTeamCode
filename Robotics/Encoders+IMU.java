package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Encoders2", group = "Linear Opmode")
public class Encoders2 extends LinearOpMode {

    IMU imu;
    private DcMotor frontLeft, frontRight, backLeft, backRight, l1, l2, arm;
    private CRServo clawL, clawR;
    private Orientation angles;

    static final double COUNTS_PER_MOTOR_REV = 1440; // Adjust based on your motor's specifications
    static final double DRIVE_GEAR_REDUCTION = 1.0;  // Adjust if you have a gear reduction
    static final double WHEEL_DIAMETER_INCHES = 6.5; // Adjust to match your wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                         (WHEEL_DIAMETER_INCHES * Math.PI);

    private final int ANGLE_0 = 50;        // Encoder ticks for 0°
    private final int ANGLE_30 = 205;    // Adjust based on your motor and gearing
    private final int ANGLE_70 = 375;  // Adjust based on your motor and gearing
    private final int ANGLE_110 = 545;  // Adjust based on your motor and gearing

    private final int TOLERANCE = 10;

    @Override
    public void runOpMode() {
        // Initialize motors
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        l2 = hardwareMap.get(DcMotor.class, "l2");
        l1 = hardwareMap.get(DcMotor.class, "l1");
        arm = hardwareMap.get(DcMotor.class, "arm");

        clawL = hardwareMap.get(CRServo.class, "clawL");
        clawR = hardwareMap.get(CRServo.class, "clawR");
        
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to use encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
                )
            );
            
        imu.resetYaw(); // This resets the yaw to 0
        telemetry.addData("IMU", "Yaw reset to 0");
        telemetry.update();

        waitForStart();

        // Example: Move forward 21 inches to place specimen
        clawL.setPower(-.15);
        clawR.setPower(.15);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(300);
        
        l1.setPower(.5);
        l2.setPower(-.5);
        encoderDrive(0.65, 11, 11, 11, 11, 5.0);
        sleep(1000);
            
        moveToAngle(ANGLE_70);
        l1.setPower(.07);
        l2.setPower(-.07);
        sleep(1000);
        
        l1.setPower(-.3);
        l2.setPower(.3);
        sleep(1000);
            
        clawL.setPower(.25);
        clawR.setPower(-.25);
        l1.setPower(.1);
        l2.setPower(-.1);
        sleep(600);
        
        l1.setPower(-.38);
        l2.setPower(.38);
        moveToAngle(ANGLE_0);
        sleep(800);
        
        arm.setPower(0);
        clawL.setPower(.05);
        clawR.setPower(-.05);
        l1.setPower(-.1);
        l2.setPower(.1);
        encoderDrive(0.6, -3, -3, -3, -3, 5.0);
        
        turnToAngle(-84, -.45);
        
        encoderDrive(0.65, 19, 19, 19, 19, 5.0);
        
        turnToAngle(-170, -.45);
        sleep(200);
        
        encoderDrive(0.6, .9, .9, .9, .9, 5.0);
        moveToAngle(ANGLE_70);
        
        sleep(1500);
        
        clawL.setPower(-.15);
        clawR.setPower(.15);
        sleep(500);
        
        moveToAngle(ANGLE_0);
        sleep(600);
        
        turnToAngle(95, -.45);
        
        encoderDrive(0.65, 25, 25, 25, 25, 5.0);
        
        turnToAngle(5, -.45);
        
        l1.setPower(.5);
        l2.setPower(-.5);
        encoderDrive(0.6, 3.7, 3.7, 3.7, 3.7, 5.0);
        sleep(1000);
        
        moveToAngle(ANGLE_70);
        l1.setPower(.07);
        l2.setPower(-.07);
        sleep(1000);
        
        l1.setPower(-.3);
        l2.setPower(.3);
        sleep(1000);
            
        clawL.setPower(.25);
        clawR.setPower(-.25);
        l1.setPower(.1);
        l2.setPower(-.1);
        sleep(600);
        
        l1.setPower(-.38);
        l2.setPower(.38);
        moveToAngle(ANGLE_0);
        sleep(800);
    
    }

    public void encoderDrive(double speed, double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
                // Optionally add telemetry to display the progress
                telemetry.addData("Path", "Driving to %7d :%7d :%7d :%7d",
                                  newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Current Position", "Running at %7d :%7d :%7d :%7d",
                                  frontLeft.getCurrentPosition(),
                                  frontRight.getCurrentPosition(),
                                  backLeft.getCurrentPosition(),
                                  backRight.getCurrentPosition());
                telemetry.update();
            }

            frontLeft.setPower(-.1);
            frontRight.setPower(-.1);
            backLeft.setPower(-.1);
            backRight.setPower(-.1);
            
            sleep(50);
            
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    private void turnToAngle(double targetAngle, double power) {
        double currentAngle = getHeading();
        double angleDifference = targetAngle - currentAngle;
    
        angleDifference = normalizeAngle(angleDifference);
        
        double turnPower = Math.signum(angleDifference) * (power);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Continue turning until the robot is within 1 degree of the target angle
        while (opModeIsActive() && Math.abs(angleDifference) > 1) {
            currentAngle = getHeading();
            angleDifference = targetAngle - currentAngle;

            angleDifference = normalizeAngle(angleDifference);

                frontLeft.setPower(turnPower);
                frontRight.setPower(-turnPower);
                backLeft.setPower(turnPower);
                backRight.setPower(-turnPower);

            // Debugging telemetry
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Angle Difference", angleDifference);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }

        // Stop the motors once the target angle is reached
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    
    private double normalizeAngle(double angleDifference) {
        while (angleDifference > 180) angleDifference -= 360;
        while (angleDifference < -180) angleDifference += 360;
            return angleDifference;
    }
    
    private void moveToAngle(int targetPosition) 
    {
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to move the motor
        arm.setPower(0.3);

            // Stop the motor when it reaches the target
        if (Math.abs(arm.getCurrentPosition() - targetPosition) <= TOLERANCE) 
        {
            arm.setPower(0);
        }
    }

}




