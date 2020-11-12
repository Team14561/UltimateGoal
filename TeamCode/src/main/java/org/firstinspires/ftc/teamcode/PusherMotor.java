package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class PusherMotor {

    // Class variables
    DcMotor motor;
    Telemetry telemetry;
    double encoderStart;


    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public PusherMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        motor = hardwareMap.get(DcMotor.class, RobotMap.PUSHER_MOTOR);

        // Set the motor directions
        motor.setDirection(RobotMap.PUSHER_DIRECTION);

        //Set the encoder starting position
        encoderStart = 0.0;

    }

    boolean buttonPressed = false;
    double encoderGoal;
    /**
     * Set the arm motor power for both left and right motors
     *
     * @param gamepad The gamepad from which to read joystick values
     */
    public void manual(Gamepad gamepad) {
        double speedLimit = RobotMap.PUSHER_SPEED;

        double power = gamepad.right_trigger - gamepad.left_trigger;

        if(gamepad.left_bumper) {
            buttonPressed = true;
            encoderGoal = encoderStart;
        }
        else if(Math.abs(power) > RobotMap.DEADZONE){
            buttonPressed = false;
        }

        if(buttonPressed){
            double error = encoderGoal - getEncoder();
            power = RobotMap.PUSHER_KP * error;
            power = safetyCheck(power);
            if(Math.abs(error) < RobotMap.ENCODER_TOLERANCE){
                buttonPressed = false;
            }

        }

        // Limit speed of arm
        power *= speedLimit;

        setPower(power);

        //output the encoder value//
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Pusher Encoder", getEncoder());
        }

    }

    private void setPower(double power){
        // Make sure power levels are within expected range
        power = safetyCheck(power);

        // Send calculated power to motors
        motor.setPower(power);
    }

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(-1.0, out);
        out = Math.min(1.0, out);
        return out;
    }

    public int getEncoder () {
        return RobotMap.REVERSE_PUSHER_ENCODER_VALUE * (motor.getCurrentPosition());
    }


}