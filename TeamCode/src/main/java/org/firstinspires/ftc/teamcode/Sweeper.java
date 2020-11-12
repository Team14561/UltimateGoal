package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class Sweeper {

    // Class variables
    Servo sweeperServo;
    Telemetry telemetry;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects

        sweeperServo = hardwareMap.get(Servo.class, RobotMap.SWEEPER_SERVO);
        sweeperServo.setPosition(RobotMap.SERVO_ANGLE_DEFAULT);

    }

    /**
     * Move Servo
     *
     * @param gamepad The gamepad from which to read joystick values
     */
    private boolean clawOpen = false;
    private boolean aReleased = true;
    private boolean xEnabled = false;
    private double servoAngle;

    public void buttonServo(Gamepad gamepad) {
        if (gamepad.a) {
            if (aReleased) clawOpen = !clawOpen;
            aReleased = false;
            xEnabled = false;

        }
        else{
            aReleased = true;
        }

        if(!xEnabled){
            servoAngle = (clawOpen) ? RobotMap.SERVO_MID : RobotMap.SERVO_CLOSED;
        }



        if (gamepad.right_bumper){
            servoAngle = RobotMap.SERVO_OPEN;
            xEnabled = true;
        }

        servoAngle = safetyCheck(servoAngle);
        sweeperServo.setPosition(servoAngle);

   }6

    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(RobotMap.MINIMUM_SERVO_POSITION, out);
        out = Math.min(RobotMap.MAXIMUM_SERVO_POSITION, out);
        return out;
    }


}
