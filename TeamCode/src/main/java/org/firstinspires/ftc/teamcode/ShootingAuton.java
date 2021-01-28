package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShootingAuton {

    private AnalogInput potentiometer;
    private int stage = 0;
    private double expirationTime;
    private ElapsedTime runtime;
    private Arm arm;
    private FlyWheel flyWheel;
    private PusherMotor pusherMotor;
    private Sweeper sweeper;
    private DriveTrain driveTrain;
    private double driveTrainEncoder;
    Telemetry telemetry;
    int pusherGoal;
    private double driveTrainGoal;


    // Object variables mimicing gamepad control
    double armPower = 0.0;
    boolean armGotoShoot = false;

    public ShootingAuton(HardwareMap hardwareMap,
                         Telemetry telemetry, ElapsedTime runtime){
        this.runtime = runtime;
        this.telemetry = telemetry;
        arm = new Arm(hardwareMap, telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        flyWheel = new FlyWheel(hardwareMap, telemetry);
        pusherMotor = new PusherMotor(hardwareMap, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);

    }

    public void mainStages() {
        driveTrainEncoder = driveTrain.rightEncoder.getCurrentPosition();

        arm.manual(armPower, armGotoShoot, false, RobotMap.POT_SHOOTING_POSITION + 0.01);


        /*
            1. Move the arm down out of starting position
            2. Stop the arm when the encoder gets to scan distance
            3. Set the arm goal to position for 3-ring
            4. Wait for arm to get into position?
            5. Check distance sensor, if rings detected number equals three, goto stage 10
            6. Set the arm goal to position for 2-ring
            7. Wait for arm to get into position
            8. Check distance sensor, if rings detected number equals two, goto stage 10
            9. Else number equals 1-ring
            10. Continue
        */


        if (stage == 0) {
            armPower = -0.9;
            expirationTime = runtime.time() + 7.0;
            stage = 10;
        }
        else if (stage == 10) {
           if ( (runtime.time() > expirationTime) || (arm.getEncoder() < -800) ) {
               armPower = 0.0;
               stage = 20;
           }
        }
        else if (stage == 20) {
            armGotoShoot = true;

            stage = 30;
        }
        else if(stage == 30){
            expirationTime = runtime.time() + 3.0;
            sweeper.buttonServo(false, true);
            flyWheel.manual(false, true, false);
            stage = 35;
        }
        else if (stage == 35){
            if ( (runtime.time() > expirationTime)) {
                stage = 40;
            }
        }
        else if(stage == 40){
            driveTrainGoal = driveTrainEncoder - 6100;
            driveTrain.arcadeDrive(0, -1, 0, false, false);
            stage = 50;
        }
        else if(stage == 50){
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 60;
            }
        }
        else if(stage == 60){
            pusherGoal = pusherMotor.getEncoder() + 680;
            pusherMotor.manual(1, 0.0, false);
            stage = 70;
        }
        else if(stage == 70){
            if(pusherMotor.getEncoder() > pusherGoal){
                pusherMotor.manual(0.0, 0.0, false);
                stage = 80;
            }
        }
        else if(stage == 80){
            flyWheel.manual(false, false, true);
            pusherMotor.manual(0, 0, true);
            stage = 90;
        }
        else if(stage == 90){
            driveTrainGoal = driveTrainEncoder - 3000;
            driveTrain.arcadeDrive(0, -1, 0, false, false);
            stage = 100;
        }
        else if(stage == 100){
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 110;
            }
        }




        /*
        else if(stage == 40){
            pusherGoal = pusherMotor.getEncoder() + 280;
            pusherMotor.manual(1.0, 0.0, false);
            stage = 50;
        }
        else if(stage == 50){
            if(pusherMotor.getEncoder() > pusherGoal){
                pusherMotor.manual(0.0, 0.0, false);
                stage = 60;
            }
        }
        else if(stage == 60){
            driveTrainGoal = driveTrainEncoder - 470;
            driveTrain.arcadeDrive(0, 0, .5, false, false);
            stage = 70;
        }
        else if(stage == 70){
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 80;
            }
        }
        else if(stage == 80){
            pusherGoal = pusherMotor.getEncoder() + 200;
            pusherMotor.manual(1.0, 0.0, false);
            stage = 90;
        }
        else if(stage == 90){
            if(pusherMotor.getEncoder() > pusherGoal){
                pusherMotor.manual(0.0, 0.0, false);
                stage = 100;
            }
        }
        else if(stage == 100){
            driveTrainGoal = driveTrainEncoder -470;
            driveTrain.arcadeDrive(0, 0, .5, false, false);
            stage = 110;
        }
        else if(stage == 110){
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 120;
            }
        }
        else if(stage == 120){
            pusherGoal = pusherMotor.getEncoder() + 200;
            pusherMotor.manual(1, 0, false);
            stage = 130;
        }
        else if(stage == 130){
            if(pusherMotor.getEncoder() > pusherGoal){
                pusherMotor.manual(0, 0, false);
                stage = 140;
            }
        }
        */
    }
}

