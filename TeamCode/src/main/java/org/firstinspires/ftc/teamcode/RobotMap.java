package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotMap {

    // Robot Parameters
    public static final Boolean DISPLAY_TIME = true;
    public static final double DEADZONE = 0.05;

    // Drivetrain Parameters
    public static final String LEFT_FRONT_MOTOR = "left_front_drive";
    public static final String RIGHT_FRONT_MOTOR = "right_front_drive";
    public static final String LEFT_BACK_MOTOR = "left_back_drive";
    public static final String RIGHT_BACK_MOTOR = "right_back_drive";
    public static final DcMotor.Direction LEFT_DRIVE_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_DRIVE_DIRECTION = DcMotor.Direction.REVERSE;
    public static final int REVERSE_DRIVETRAIN_ENCODER_VALUE = -1;

    //TankDrive Parameters
    public static final Boolean DISPLAY_MOTOR_VALUES = true;
    public static final Boolean REVERSE_JOYSTICKS = false;
    public static final double HIGHSPEED = 0.9;
    public static final double LOWSPEED = 0.3;

    //Encoder Parameters
    public static final Boolean DISPLAY_ENCODER_VALUES = true;
    public static final int ENCODER_TOLERANCE = 20;

    //Arm Parameters
    public static final String LEFT_ARM_MOTOR = "arm_lifter";
    public static final String RIGHT_ARM_MOTOR = "arm_lifter_encoder";
    public static final DcMotor.Direction LEFT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double ARM_SPEED_UP = 0.35;
    public static final double ARM_SPEED_DOWN = 0.3;
    public static final double AUTO_ARM_SPEED = 0.3;
    public static final int REVERSE_ARM_ENCODER_VALUE = -1;
    public static final int REVERSE_ARM_DIRECTION  = -1;
    public static final double kP = 0.06;
    public static final double kD = 0.05;
    public static final double ARM_UP = -300;
    public static final double ARM_DOWN = -1000;
    public static final double GRAVITY_AMPLITUDE = 1.18;
    public static final double SHOOTING_POSITION = -670.0;

    //FlyWheel Parameters
    public static final String FLYWHEEL_MOTOR = "flywheel_motor";
    public static final DcMotor.Direction FLYWHEEL_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double FLYWHEEL_SPEED_IN = 0.24;
    public static final double FLYWHEEL_SPEED_OUT = -0.48;
    public static final int REVERSE_FLYWHEEL_ENCODER_VALUE = -1;

    //Sweeper Parameters
    public static final String SWEEPER_SERVO = "sweeper_servo";
    public static final double SERVO_OPEN = 0.0;
    public static final double SERVO_CLOSED = 0.7;
    public static final double SERVO_MID = 0.4;
    public static final double MINIMUM_SERVO_POSITION = 0.0;
    public static final double MAXIMUM_SERVO_POSITION = 1.0;
    public static final double SERVO_ANGLE_DEFAULT = SERVO_CLOSED;

    //Pusher Parameters
    public static final String PUSHER_MOTOR = "pusher_arm";
    public static final DcMotor.Direction PUSHER_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double PUSHER_SPEED = 0.25;
    public static final int REVERSE_PUSHER_ENCODER_VALUE = -1;
    public static final double PUSHER_KP = 0.01;

    //Touch Sensor Parameters
    public static final String HEIGHT_SENSOR = "height_sensor";

}
