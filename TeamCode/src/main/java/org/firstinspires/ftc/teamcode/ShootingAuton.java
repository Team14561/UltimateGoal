package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.FlyWheel;
import org.firstinspires.ftc.teamcode.PusherMotor;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Sweeper;
import org.firstinspires.ftc.teamcode.WobbleRelease;

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
    private WobbleRelease wobbleRelease;
    private RingSensor colorSensor;
    boolean countRings = true;
    int ringCount = 0;
    double firstRingShown;
    double encoderDiff;


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
        wobbleRelease = new WobbleRelease(hardwareMap, telemetry);
        colorSensor = new RingSensor(hardwareMap, telemetry);

    }

    public void mainStages() {
        driveTrainEncoder = driveTrain.rightEncoder.getCurrentPosition();

        arm.manual(armPower, armGotoShoot, false, RobotMap.POT_SHOOTING_POSITION);
        colorSensor.broadcastColor();

        if(countRings){
            if(colorSensor.isOrange()){

                if(ringCount == 0) {
                    firstRingShown = driveTrainEncoder;
                    ringCount = 1;
                }
                else if(ringCount == 1){
                    encoderDiff = driveTrainEncoder - firstRingShown;

                    if(encoderDiff < -650){
                        ringCount = 4;
                        countRings = false;
                    }
                }
            }
        }
        telemetry.addData("encoder difference:", encoderDiff);
        telemetry.addData("Ring Number: ", ringCount);
        telemetry.addData("arm power: ", armPower);

        if (stage == 0) {
            armPower = -1.0;
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
            driveTrainGoal = driveTrainEncoder - 3900; //60:1 gearbox value was 6100 * (2/3) = 4067
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
            driveTrainGoal = driveTrainEncoder - 2800;
            driveTrain.arcadeDrive(0, -1, 0, false, false);
            stage = 100;
        }
        else if(stage == 100){
            if(driveTrainEncoder < (driveTrainGoal + 1000)){
                countRings = false;
            }
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 110;
            }
        }
        else if (stage == 110) {
            armGotoShoot = false;
            armPower = 1.0;
            pusherMotor.manual(0.0, 0.0, true);
            stage = 120;
        }
        else if (stage == 120) {
            if (arm.getEncoder() > -60) {
                armPower = 0.0;
                pusherMotor.manual(0.0,0.0,false);

                switch (ringCount){
                    case 0:
                        stage = 150;
                        break;
                    case 1:
                        stage = 135;
                        break;
                    case 4:
                        stage = 125;
                        break;
                }
            }
        }
        else if(stage == 125){ //if 4 rings
            driveTrainGoal = driveTrainEncoder - 4250;
            driveTrain.arcadeDrive(0, -1, 0, false, false);
            stage = 140;
        }
        else if(stage == 135){
            driveTrainGoal = driveTrainEncoder - 4000;
            driveTrain.arcadeDrive(0, -1, 1, false, false);
            stage = 140;
        }
        else if(stage == 140){
            if(driveTrainEncoder < driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 150; //going to drop goal
            }
        }
        else if(stage == 150) {
            wobbleRelease.wobbleRelease();
            expirationTime = runtime.time() + 2.0;
            stage = 160;
        }
        else if(stage == 160){
            if ( (runtime.time() > expirationTime)) {
                switch (ringCount){
                    case 0:
                        break; //has no need to move back
                    case 1:
                        stage = 180;
                        break;
                    case 4:
                        stage = 170;
                        break;
                }
            }
        }
        else if(stage == 170){ // 1 ring
            driveTrainGoal = driveTrainEncoder + 4000;
            driveTrain.arcadeDrive(0, 1, 0, false, false);
            stage = 190;
        }
        else if(stage == 180){ // 4 rings
            driveTrainGoal = driveTrainEncoder + 2000;
            driveTrain.arcadeDrive(0, 1, 0, false, false);
            stage = 190;
        }
        else if(stage == 190){ //stopping
            if(driveTrainEncoder > driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 200;
            }
        }






        /* Power shot

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


