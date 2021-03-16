package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Class for controlling the drivetrain of an FTC robot.
 */
public class DriveTrain {

    // Class variables
    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    DcMotor leftEncoder, rightEncoder;
    BNO055IMU gyro;
    Telemetry telemetry;
    int leftZero, rightZero;
    double gyroZero = 0.0;
    Boolean highSpeed = true;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry   the robot instance of the telemetry object
     */
    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        leftFrontMotor = hardwareMap.get(DcMotor.class, RobotMap.LEFT_FRONT_MOTOR);
        rightFrontMotor = hardwareMap.get(DcMotor.class, RobotMap.RIGHT_FRONT_MOTOR);
        leftBackMotor = hardwareMap.get(DcMotor.class, RobotMap.LEFT_BACK_MOTOR);
        rightBackMotor = hardwareMap.get(DcMotor.class, RobotMap.RIGHT_BACK_MOTOR);

        // Set the motor directions
        leftFrontMotor.setDirection(RobotMap.LEFT_DRIVE_DIRECTION);
        rightFrontMotor.setDirection(RobotMap.RIGHT_DRIVE_DIRECTION);
        leftBackMotor.setDirection(RobotMap.LEFT_DRIVE_DIRECTION);
        rightBackMotor.setDirection(RobotMap.RIGHT_DRIVE_DIRECTION);

        leftEncoder = leftBackMotor;
        rightEncoder = rightBackMotor;

        //Set the encoder starting positions
        rightZero = getEncoderRight();
        leftZero = getEncoderLeft();

        //Set up gyroscope
        //gyro = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public int getEncoderRight() {
        return RobotMap.REVERSE_DRIVETRAIN_ENCODER_VALUE * (rightEncoder.getCurrentPosition());
    }

    public int getEncoderLeft() {
        return RobotMap.REVERSE_DRIVETRAIN_ENCODER_VALUE * (leftEncoder.getCurrentPosition());
    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(parameters);
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 10);

        gyroZero = gyro.getAngularOrientation().firstAngle;
    }


    /**
     * Set the drivetrain motor power for both left and right motors using the joystick values
     *
     * @param gamepad The gamepad from which to read joystick values
     */


    public void mecanumDrive(Gamepad gamepad) {


        double leftPower = gamepad.left_stick_y;
        double rightPower = gamepad.right_stick_y;


        // Reverse joystick values if requested
        if (RobotMap.REVERSE_JOYSTICKS) {
            leftPower *= -1.0;
            rightPower *= -1.0;
        }

        double rightTrigger = -gamepad.right_trigger;
        double leftTrigger = -gamepad.left_trigger;

        double strafeValue = rightTrigger - leftTrigger;

        if (gamepad.left_bumper) {
            highSpeed = false;

        } else if (gamepad.right_bumper) {
            highSpeed = true;
        }

        mecanumDrive(leftPower, rightPower, strafeValue);

        // Output Encoder Values
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Left Encoder", getEncoderLeft());
            telemetry.addData("Right Encoder", getEncoderRight());
            // telemetry.addData("Gyroscope", gyro.getAngularOrientation().firstAngle);
        }
    }

    public void mecanumDrive(double leftPower, double rightPower, double strafeValue){


        double leftFrontPower = leftPower + strafeValue;
        double rightFrontPower = rightPower - strafeValue;
        double leftBackPower = leftPower - strafeValue;
        double rightBackPower = rightPower + strafeValue;

        double maxPower = Math.abs(leftFrontPower);
        maxPower = Math.max(Math.abs(rightFrontPower), maxPower);
        maxPower = Math.max(Math.abs(leftBackPower), maxPower);
        maxPower = Math.max(Math.abs(rightBackPower), maxPower);

        if(maxPower > 1){

            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;

        }


        double speedLimit;
        if (highSpeed) {
            speedLimit = RobotMap.HIGHSPEED;
        }
        else {
            speedLimit = RobotMap.LOWSPEED;
        }

        leftFrontPower *= speedLimit;
        rightFrontPower *= speedLimit;
        leftBackPower *= speedLimit;
        rightBackPower *= speedLimit;

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);


    }

    public void arcadeDrive(Gamepad gamepad) {
        arcadeDrive(gamepad.right_stick_x, gamepad.left_stick_y, gamepad.left_stick_x, gamepad.left_bumper, gamepad.right_bumper);
    }



    public void arcadeDrive(double rStickX, double lStickY, double lStickX, Boolean leftB, Boolean rightB){
        double rotation = rStickX;
        double leftPower = lStickY - rotation;
        double rightPower = lStickY + rotation;
        double strafeValue = -lStickX;


        // Reverse joystick values if requested
        if (RobotMap.REVERSE_JOYSTICKS) {
            leftPower *= -1.0;
            rightPower *= -1.0;
        }

        if (leftB) {
            highSpeed = false;

        }
        else if (rightB){
            highSpeed = true;
        }

        mecanumDrive(leftPower, rightPower, strafeValue);

        // Output Encoder Values
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
            // telemetry.addData("Gyroscope", gyro.getAngularOrientation().firstAngle);
        }


    }


    public void tankDrive(Gamepad gamepad) {

        if (gamepad.left_bumper) {
            highSpeed = false;

        }
        else if (gamepad.right_bumper){
            highSpeed = true;
        }


        double speedLimit;
        if (highSpeed) {
            speedLimit = RobotMap.HIGHSPEED;
        }
         else {
            speedLimit = RobotMap.LOWSPEED;
        }

        // Get joystick values from gamepad
        double leftPower = gamepad.left_stick_y;
        double rightPower = gamepad.right_stick_y;

        // Limit speed of drivetrain
        leftPower *= speedLimit;
        rightPower *= speedLimit;


        // Reverse joystick values if requested
        if (RobotMap.REVERSE_JOYSTICKS) {
            leftPower *= -1.0;
            rightPower *= -1.0;
        }

        setPower(leftPower, rightPower);


    }

    private void setPower(double leftPower, double rightPower) {
        // Make sure power levels are within expected range
        leftPower = safetyCheck(leftPower);
        rightPower = safetyCheck(rightPower);

        // Send calculated power to motors
        leftFrontMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        leftBackMotor.setPower(leftPower);
        rightBackMotor.setPower(rightPower);

        //Display power levels for operator
        if (RobotMap.DISPLAY_MOTOR_VALUES) {
            telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                    leftPower, rightPower);
        }
    }

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(-1.0, out);
        out = Math.min(1.0, out);
        return out;
    }




    //Autonomous Here Down


    /*
    private double leftEncoderInches() {
        return (leftMotor.getCurrentPosition() - leftZero) / RobotMap.MOTOR_SCALE;
    }

    private double rightEncoderInches() {
       return (rightMotor.getCurrentPosition() - rightZero) / RobotMap.MOTOR_SCALE;
    }
    */
    /*
    private void move(double distance, int directionDegrees){
        double directionRatio = 20;
        for(int i = 0; i < directionDegrees * directionRatio; i++){
            if(directionDegrees < 0){
                mecanumDrive(-1, 1, 0);
            }
            else{
                mecanumDrive(1, -1, 0);
            }
        }

        double distRatio = 20;
        for(int i = 0; i < distance * distRatio; ++i){
            mecanumDrive(1, 1, 0);
        }


    }
*/

}

