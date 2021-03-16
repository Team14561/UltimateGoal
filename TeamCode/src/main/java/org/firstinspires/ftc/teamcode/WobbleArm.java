package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling the Arm of an FTC robot.
 */
public class WobbleArm {

    // Class variables
    Servo wobbleServo1;
    Servo wobbleServo2;
    Telemetry telemetry;
    private Boolean armOpen = false;
    private Boolean bReleased = true;

    /**
     * Constructor for the drivetrain
     *
     * @param hardwareMap the robot instance of the hardware map
     * @param telemetry the robot instance of the telemetry object
     */
    public WobbleArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Assign hardware objects

        wobbleServo1 = hardwareMap.get(Servo.class, RobotMap.WOBBLE_ARM_1);
        wobbleServo1.setPosition(RobotMap.WARM1_CLOSE);
        wobbleServo2 = hardwareMap.get(Servo.class, RobotMap.WOBBLE_ARM_2);
        wobbleServo2.setPosition(RobotMap.WARM2_CLOSE);

    }



    public void wobbleOpen(){
        wobbleServo1.setPosition(RobotMap.WARM1_OPEN);
        wobbleServo2.setPosition(RobotMap.WARM2_OPEN);
    }

    public void wobbleclose(){
        wobbleServo1.setPosition(RobotMap.WARM1_GRAB);
        wobbleServo2.setPosition(RobotMap.WARM2_GRAB);
    }


    private double safetyCheck(double inp) {
        double out = inp;
        out = Math.max(RobotMap.MINIMUM_SERVO_POSITION, out);
        out = Math.min(RobotMap.MAXIMUM_SERVO_POSITION, out);
        return out;
    }

    public void manual(Gamepad gamepad){
        if(gamepad.b){
            if(bReleased){
                if(armOpen){
                    armOpen = false;
                    wobbleclose();
                } else {
                    armOpen = true;
                    wobbleOpen();
                }
            }
            bReleased = false;
        }else{
            bReleased = true;
        }
    }
}
