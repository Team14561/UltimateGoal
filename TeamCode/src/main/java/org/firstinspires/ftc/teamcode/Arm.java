package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class Arm {

    // Class variables
    DcMotor leftMotor, rightMotor, encoderMotor;
    AnalogInput pot;
    Telemetry telemetry;
    double encoderGoal, previousEncoder;
    int encoderZero;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        leftMotor = hardwareMap.get(DcMotor.class, RobotMap.LEFT_ARM_MOTOR);
        rightMotor = hardwareMap.get(DcMotor.class, RobotMap.RIGHT_ARM_MOTOR);

        // Set the motor directions
        leftMotor.setDirection(RobotMap.LEFT_ARM_DIRECTION);
        rightMotor.setDirection(RobotMap.RIGHT_ARM_DIRECTION);

        //Create the pot
        pot = hardwareMap.get(AnalogInput.class, RobotMap.POT_NAME);

        //Set the encoder starting position
        encoderMotor = rightMotor;
        initEncoder();
        encoderGoal = getEncoder();
        previousEncoder = encoderGoal;
    }

    /**
     * Set the arm motor power for both left and right motors
     *
     * @param gamepad The gamepad from which to read joystick values
     */


    public void test(Gamepad gamepad) {

        double encoderValue = getEncoder();

        // Get joystick values from gamepad
        double power  = gamepad.left_stick_y * RobotMap.REVERSE_ARM_DIRECTION;

        double speedLimit = RobotMap.ARM_SPEED_UP;
        if (power > 0) speedLimit = RobotMap.ARM_SPEED_DOWN;

        // Limit speed of arm
        power *= speedLimit;

        setPower(power);

        //output the encoder value//
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Arm Encoder", encoderValue);
            telemetry.addData("Arm Goal", encoderGoal);
        }

    }

    public void manual(Gamepad gamepad) {

        // Get joystick values from gamepad
        double power  = gamepad.left_stick_y * RobotMap.REVERSE_ARM_DIRECTION;

        manual(power, gamepad.y);
    }

    public void manual(double power, boolean yPressed) {
        double encoderValue = getEncoder();
        double deltaEncoder = previousEncoder - encoderValue;
        previousEncoder = encoderValue;

        double speedLimit = RobotMap.ARM_SPEED_UP;
        if (power < 0) speedLimit = RobotMap.ARM_SPEED_DOWN;

        if(yPressed){
            encoderGoal = RobotMap.SHOOTING_POSITION;
        }

        if (Math.abs(power) < RobotMap.DEADZONE) {
            double error = encoderGoal - encoderValue;
            power = RobotMap.kP * error;
        }
        else {
            encoderGoal = getEncoder();
            double gravityCorrection = 0.0;

            if ((power < 0) && (encoderValue < RobotMap.ARM_UP)) {

                //D term for PID loop
                //gravityCorrection = RobotMap.kD * deltaEncoder;

                //Linear gravity correction
                //gravityCorrection = RobotMap.GRAVITY_AMPLITUDE * (RobotMap.ARM_UP - encoderValue) /
                //        (RobotMap.ARM_UP - RobotMap.ARM_DOWN);

                //Sinusoidal gravity correction
                gravityCorrection = RobotMap.GRAVITY_AMPLITUDE * Math.sin( (Math.PI / 2) *
                        (RobotMap.ARM_UP - encoderValue) /
                        (RobotMap.ARM_UP - RobotMap.ARM_DOWN) );
            }
            else if (power > 0 && (encoderValue > RobotMap.ARM_UP)) {
                gravityCorrection = -RobotMap.GRAVITY_AMPLITUDE * Math.sin( (Math.PI / 2) *
                        (RobotMap.ARM_UP - encoderValue) / RobotMap.ARM_UP);
            }
            power += gravityCorrection;
        }

        // Limit speed of arm
        power = safetyCheck(power);
        power *= speedLimit;

        setPower(power);

        //output the encoder value//
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Arm Encoder", encoderValue);
            telemetry.addData("Arm Goal", encoderGoal);
        }

    }

    private void setPower(double power  ){
        // Make sure power levels are within expected range
        power = safetyCheck(power);

        // Send calculated power to motors
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(-1.0, out);
        out = Math.min(1.0, out);
        return out;
    }

    public void initEncoder() {
        encoderZero = 0;
        encoderZero = getEncoder();
    }

    public int getEncoder () {
        telemetry.addData("Potentiometer", pot.getVoltage());
        return RobotMap.REVERSE_ARM_ENCODER_VALUE * encoderMotor.getCurrentPosition() - encoderZero;
   }




   //Autonomus Here Down

    public void autonMaintain (){
        double error = getEncoder () - encoderGoal;
        double power = RobotMap.kP * error;
        power = safetyCheck(power);
        power *= RobotMap.AUTO_ARM_SPEED;
        setPower(power);
    }

    public void setAutonGoal (int newGoal){
        encoderGoal = newGoal;
    }

    public boolean atGoal() {
        return Math.abs(getEncoder() - encoderGoal) <= RobotMap.ENCODER_TOLERANCE;
    }


}

