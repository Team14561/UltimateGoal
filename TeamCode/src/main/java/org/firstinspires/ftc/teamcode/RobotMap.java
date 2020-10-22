package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotMap {

    // Robot Parameters
    public static final Boolean DISPLAY_TIME = true;

    // Drivetrain Parameters
    public static final String LEFT_FRONT_MOTOR = "left_front_drive";
    public static final String RIGHT_FRONT_MOTOR = "right_front_drive";
    public static final String LEFT_BACK_MOTOR = "left_back_drive";
    public static final String RIGHT_BACK_MOTOR = "right_back_drive";
    public static final DcMotor.Direction LEFT_DRIVE_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_DRIVE_DIRECTION = DcMotor.Direction.REVERSE;

    //TankDrive Parameters
    public static final Boolean DISPLAY_MOTOR_VALUES = true;
    public static final Boolean REVERSE_JOYSTICKS = false;
    public static final double HIGHSPEED = 0.9;
    public static final double LOWSPEED = 0.3;

    //Encoder Parameters
    public static final Boolean DISPLAY_ENCODER_VALUES = true;
    public static final int ENCODER_TOLERANCE = 10;

    //Arm Parameters
    public static final String LEFT_ARM_MOTOR = "arm_left";
    public static final String RIGHT_ARM_MOTOR = "arm_right";
    public static final DcMotor.Direction LEFT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double ARM_SPEED_UP = 0.2;
    public static final double ARM_SPEED_DOWN = 0.2;
    public static final double kP = 0.05;
    public static final double DEADZONE = 0.05;
    public static final double AUTO_ARM_SPEED = 0.17;

}
