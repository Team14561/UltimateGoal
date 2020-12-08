package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RingSenseAuton {

    private int stage = 0;
    private double expirationTime;
    private ElapsedTime runtime;
    private HeightSensor heightSensor;
    private Arm arm;
    private FlyWheel flyWheel;
    private PusherMotor pusherMotor;
    private Sweeper sweeper;
    private DriveTrain driveTrain;
    private double armEncoderStart;
    private double armEncoderEnd = 1317.0;
    private double oldHeight;
    private double newHeight;
    public int ringNumber;
    private double sensorWait;
    private double driveTrainEncoder = driveTrain.rightEncoder.getCurrentPosition();
    private double driveTrainGoal;

    // Object variables mimicing gamepad control
    double armPower = 0.0;
    boolean armGotoShoot = false;

    public RingSenseAuton (HardwareMap hardwareMap,
                            Telemetry telemetry, ElapsedTime runtime){
        this.runtime = runtime;
        heightSensor = new HeightSensor(hardwareMap,telemetry);
        arm = new Arm(hardwareMap, telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        flyWheel = new FlyWheel(hardwareMap, telemetry);
        pusherMotor = new PusherMotor(hardwareMap, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
    }

    public void mainStages() {
        arm.manual(armPower, armGotoShoot);

        /*
            1. Move the arm down out of starting position
            2. Stop the arm when the encoder gets to scan distance
            3. Set the arm goal to position for 3-ring
            4. Wait for arm to get into position
            5. Check distance sensor, if rings detected number equals three, goto stage 10
            6. Set the arm goal to position for 2-ring
            7. Wait for arm to get into position
            8. Check distance sensor, if rings detected number equals two, goto stage 10
            9. Else number equals 1-ring
            10. Continue
        */


        if (stage == 0) {
            armEncoderStart = arm.getEncoder();
            armPower = 0.5;
            expirationTime = runtime.time() + 5.0;
            stage = 1;
        }
        //get values to look for jump
        else if (stage == 1) {
            oldHeight = heightSensor.getHeight();
            sensorWait = runtime.time() + 0.1;
            stage = 2;
        } else if (stage == 2) {
            if (runtime.time() > sensorWait) {
                newHeight = heightSensor.getHeight();
                stage = 3;
            }
        }
        //if jump or time runs out stop arm and move on
        else if (stage == 3) {
            if (Math.abs(oldHeight - newHeight) > 50 || runtime.time() > expirationTime) {
                armPower = 0.0;
                armEncoderEnd = arm.getEncoder();
                stage = 4;
            } else {
                stage = 1;
            }
        }
        //figure out how far arm went and determine number of rings
        else if (stage == 4) {
            if (armEncoderStart - armEncoderEnd < 300) {
                ringNumber = 3;
            } else if (armEncoderStart - armEncoderEnd > 300) {
                ringNumber = 1;
            } else {
                ringNumber = 2;
            }
            //no end value set (timed out)
            if (armEncoderEnd == 1317.0) {
                ringNumber = 0;
            }
            stage = 5;
        }
        //intake rings
        else if(stage == 5){
            double time1 = 0;
            if (armEncoderStart - armEncoderEnd > -300) {
                arm.manual(-5, false);
                expirationTime = runtime.time() + 5.0;
                time1 = expirationTime - 5.0;
            }
            else if(time1 < expirationTime){
                flyWheel.manual(true, false, false);
            } else{
                stage = 6;
            }
        }
        //move to wobble goal
        else if(stage == 6){
            driveTrainGoal = 100;
            if(driveTrainEncoder </*will have to change sign*/ driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 1, false, false);
            } else {
                stage = 7;
            }

        }
        //pick up wobble goal
        else if(stage == 7){
            armPower = -0.2;
            expirationTime = runtime.time() + 5.0;
            stage = 8;
        }
        //move to target zone
        else if(stage == 8){
            driveTrainGoal = 1000;

            if(driveTrainEncoder </*will have to change sign*/ driveTrainGoal) {
                driveTrain.arcadeDrive(0, 1, 0, false, false);
            } else {
                stage = 9;
            }
        }
        //move to target zone specified
        else if(stage == 9){
            switch(ringNumber){
                case 1:
                    driveTrainGoal = 100;
                    if(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(0, 0, 1, false, false);
                    } else {
                        stage = 10;
                        break;
                    }
                case 2:
                    driveTrainGoal = 100;
                    if(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(0, 2, -1, false, false);
                    } else {
                        stage = 10;
                        break;
                    }
                case 3:
                    driveTrainGoal = 100;
                    if(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(0, 3, 1, false, false);
                    } else {
                        stage = 10;
                        break;
                    }
            }
        }
        //drop wobble goal
        else if(stage == 10){
            armPower = -.03;
            expirationTime = runtime.time() + 5.0;

            driveTrainGoal = 100;
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, -1, 0, false, false);
            }
            stage = 10;
        }
        //go to shooting position
        else if(stage == 11){
            double stickY = 0;
            double stickX = 0;
            switch(ringNumber){
                case 1:
                    stickY = -2;
                    stickX = -1;
                    driveTrainGoal = 100;
                    break;
                case 2:
                    stickY = -2;
                    stickX = 0;
                    driveTrainGoal = 100;
                    break;
                case 3:
                    stickY = -3;
                    stickX = -1;
                    driveTrainGoal = 100;
                    break;
            }

            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, stickY, stickX, false, false);
            }else{
                stage = 12;
            }

        }
        //shoot rings
        else if(stage == 12){
            double pusherGoal = 100;
            sweeper.buttonServo(false, true);
            flyWheel.manual(false, true, false);
            if(pusherGoal > pusherMotor.getEncoder()){
                pusherMotor.manual(.5, 0, false);
            }else{
                stage = 13;
            }

        }
        //move to end zone
        else if(stage == 13)
        {
            driveTrainGoal = 100;
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, 1, 0, false, false);
            }

        }
    }
}


