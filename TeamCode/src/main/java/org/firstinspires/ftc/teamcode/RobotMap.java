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
    public static final double HIGHSPEED = 1.0;
    public static final double LOWSPEED = 0.3;

    //Encoder Parameters
    public static final Boolean DISPLAY_ENCODER_VALUES = true;
    public static final int ENCODER_TOLERANCE = 20;

    //Arm Parameter
    public static final String LEFT_ARM_MOTOR = "arm_lifter";
    public static final String RIGHT_ARM_MOTOR = "arm_lifter_encoder";
    public static final String POT_NAME = "pot";
    public static final DcMotor.Direction LEFT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_ARM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double ARM_SPEED_UP = 0.45; // 0.4, 0.45
    public static final double ARM_SPEED_DOWN = 0.43; // 0.3, 0.43
    public static final double AUTO_ARM_SPEED = 0.3;
    public static final int REVERSE_ARM_ENCODER_VALUE = -1;
    public static final int REVERSE_ARM_DIRECTION  = -1;
    public static final double kP = 0.06;
    public static final double kP_POT = 50.0; // 20.0, 25.0
    public static final double ARM_UP = -300;
    public static final double ARM_DOWN = -1200; //-1000, -1200
    public static final double GRAVITY_AMPLITUDE = 1.18;
    public static final double SHOOTING_POSITION = -700.0;
    public static final int FOUR_RING_HEIGHT = -900;
    public static final int ONE_RING_HEIGHT = -950;
    public static final double POT_ARM_UP = 1.03;
    public static final double POT_ARM_DOWN = 2.24;
    public static final double POT_SHOOTING_POSITION = 1.73;

    //FlyWheel Parameters
    public static final String FLYWHEEL_MOTOR = "flywheel_motor";
    public static final DcMotor.Direction FLYWHEEL_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double FLYWHEEL_SPEED_IN = 0.24;
    public static final double FLYWHEEL_SPEED_OUT = -0.48;
    public static final int REVERSE_FLYWHEEL_ENCODER_VALUE = -1;

    //Sweeper Parameters
    public static final String SWEEPER_SERVO = "sweeper_servo";
    public static final double SERVO_OPEN = 0.0;
    public static final double SERVO_CLOSED = 0.8;
    public static final double SERVO_MID = 0.25;
    public static final double MINIMUM_SERVO_POSITION = 0.0;
    public static final double MAXIMUM_SERVO_POSITION = 1.0;
    public static final double SERVO_ANGLE_DEFAULT = SERVO_CLOSED;

    //Pusher Parameters
    public static final String PUSHER_MOTOR = "pusher_arm";
    public static final DcMotor.Direction PUSHER_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double PUSHER_SPEED = 0.25;
    public static final int REVERSE_PUSHER_ENCODER_VALUE = -1;
    public static final double PUSHER_KP = 0.01;
    //one ring shot value: 295
    //two ring shot value: 510
    //three ring shot (empty) value: 680

    //Touch Sensor Parameters
    public static final String HEIGHT_SENSOR = "height_sensor";
    public static final String COLOR_SENSOR = "color_sensor";

    //Wobble Release Parameters
    public static final String WOBBLE_SERVO = "wobble_release";
    public static final double WOBBLE_HOLD = 1.0;
    public static final double WOBBLE_RELEASE = 0.3;

}
