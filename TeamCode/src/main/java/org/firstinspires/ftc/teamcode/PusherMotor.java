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

    }

    /**
     * Set the arm motor power for both left and right motors
     *
     * @param gamepad The gamepad from which to read joystick values
     */


    public void manual(Gamepad gamepad) {
        double speedLimit = RobotMap.PUSHER_SPEED;

        // Get joystick values from gamepad
        //double power  = gamepad.right_stick_y;

        double power = gamepad.right_trigger - gamepad.left_trigger;


        // Limit speed of arm
        power *= speedLimit;

        setPower(power);

        //output the encoder value//
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Elevator Encoder", getEncoder());
        }

    }

    private void setPower(double power  ){
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
        return motor.getCurrentPosition();
    }


}