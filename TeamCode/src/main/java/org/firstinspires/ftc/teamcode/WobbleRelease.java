package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class WobbleRelease {

    // Class variables
    Servo wobbleServo;
    Telemetry telemetry;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public WobbleRelease(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects

        wobbleServo = hardwareMap.get(Servo.class, RobotMap.WOBBLE_SERVO);
        wobbleServo.setPosition(RobotMap.WOBBLE_HOLD);

    }



    public void wobbleRelease(){
        wobbleServo.setPosition(RobotMap.WOBBLE_RELEASE);
    }


    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(RobotMap.MINIMUM_SERVO_POSITION, out);
        out = Math.min(RobotMap.MAXIMUM_SERVO_POSITION, out);
        return out;
    }


}
