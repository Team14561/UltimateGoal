package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class FlyWheel {
    public static enum FlywheelState {SHOOT, INTAKE, STOP;}

    // Class variables
    DcMotor motor;
    Telemetry telemetry;
    FlywheelState state;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public FlyWheel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects
        motor = hardwareMap.get(DcMotor.class, RobotMap.FLYWHEEL_MOTOR);

        // Set the motor directions
        motor.setDirection(RobotMap.FLYWHEEL_DIRECTION);

        //Set FlyWheel initial state
        state = FlywheelState.STOP;

    }

    /**
     * Set the arm motor power for both left and right motors
     *
     * @param gamepad The gamepad from which to read joystick values
     */


    public void manual(Gamepad gamepad) {

        if (gamepad.left_bumper) {
            state = FlywheelState.INTAKE;
        }
        else if (gamepad.right_bumper) {
            state = FlywheelState.SHOOT;
        }
        else if (gamepad.b) {
            state = FlywheelState.STOP;
        }

        double power = 0.0;

        if (state == FlywheelState.SHOOT) {
            power = RobotMap.FLYWHEEL_SPEED_OUT;
        }
        else if (state == FlywheelState.INTAKE) {
            power = RobotMap.FLYWHEEL_SPEED_IN;
        }

        setPower(power);


        //output the encoder value//
        if (RobotMap.DISPLAY_ENCODER_VALUES) {
            telemetry.addData("Flywheel Encoder", getEncoder());
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