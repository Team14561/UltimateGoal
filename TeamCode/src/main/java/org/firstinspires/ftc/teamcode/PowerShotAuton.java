package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PowerShotAuton {

    private WobbleArm wobbleArm;
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

    public PowerShotAuton(Arm arm,
                          DriveTrain driveTrain,
                          FlyWheel flyWheel,
                          PusherMotor pusherMotor,
                          Sweeper sweeper,
                          WobbleArm wobbleArm,
                          Telemetry telemetry,
                          ElapsedTime runtime) {
        this.runtime = runtime;
        this.telemetry = telemetry;
        this.arm = arm;
        this.driveTrain = driveTrain;
        this.flyWheel = flyWheel;
        this.pusherMotor = pusherMotor;
        this.sweeper = sweeper;


    }

    public PowerShotAuton(HardwareMap hardwareMap,
                          Telemetry telemetry, ElapsedTime runtime) {
        this.runtime = runtime;
        this.telemetry = telemetry;
        arm = new Arm(hardwareMap, telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        flyWheel = new FlyWheel(hardwareMap, telemetry);
        pusherMotor = new PusherMotor(hardwareMap, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        wobbleArm = new WobbleArm(hardwareMap, telemetry);
        wobbleRelease = new WobbleRelease(hardwareMap, telemetry);
        colorSensor = new RingSensor(hardwareMap, telemetry);

    }

    public void mainStages() {
        driveTrainEncoder = driveTrain.rightEncoder.getCurrentPosition();

        arm.manual(armPower, armGotoShoot, false, RobotMap.POT_POWER_POSITION);
        colorSensor.broadcastColor();

        if (countRings) {
            if (colorSensor.isOrange()) {

                if (ringCount == 0) {
                    firstRingShown = driveTrainEncoder;
                    ringCount = 1;
                } else if (ringCount == 1) {
                    encoderDiff = driveTrainEncoder - firstRingShown;

                    if (encoderDiff < -1200) {
                        ringCount = 4;
                        countRings = false;
                    }
                }
            }
        }
        telemetry.addData("stage:", stage);
        telemetry.addData("Ring Number: ", ringCount);
        telemetry.addData("arm power: ", armPower);
        if(stage == 0){
            driveTrainGoal = driveTrainEncoder - 30; //60:1 gearbox value was 6100 * (2/3) = 4067 //4175
            driveTrain.arcadeDrive(0, 0, .7, false, false);
            stage = 1;
        }
        else if(stage == 1){
            if (driveTrainEncoder < driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 8;
            }
        } else if (stage == 8) {
            armPower = -1.0;
            expirationTime = runtime.time() + 1.0;
            stage = 10;
        } else if (stage == 10) {
            if ((runtime.time() > expirationTime) || (arm.getEncoder() < -850)) {
                armPower = 0.0;
                stage = 20;
            }
        } else if (stage == 20) {
            armGotoShoot = true;

            stage = 30;
        } else if (stage == 30) {
            expirationTime = runtime.time() + 3.0;
            sweeper.buttonServo(false, true);
            stage = 35;
        } else if (stage == 35) {
            if ((runtime.time() > expirationTime)) {
                stage = 40;
            }
        } else if (stage == 40) {
            flyWheel.manual(false, true, false);
            driveTrainGoal = driveTrainEncoder - 4100; //60:1 gearbox value was 6100 * (2/3) = 4067 //4175
            driveTrain.arcadeDrive(0, -.7, 0, false, false);
            stage = 50;
        } else if (stage == 50) {
            expirationTime = runtime.time() + 1.0;
            if (driveTrainEncoder < driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 55;
            }
        } else if (stage == 55) {
            if ((runtime.time() > expirationTime)) {
                stage = 57;
            }
        }
        else if(stage == 57){
            driveTrainGoal = driveTrainEncoder + 30; //60:1 gearbox value was 6100 * (2/3) = 4067 //4175
            driveTrain.arcadeDrive(0, 0, -.7, false, false);
            stage = 58;
        }
        else if(stage == 58){
            if (driveTrainEncoder > driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 60;
            }
        }
        else if (stage >= 60 && stage < 90) {
            stage = powerShoot(stage);

        } else if (stage == 90) {
            driveTrainGoal = driveTrainEncoder - 2800;
            driveTrain.arcadeDrive(0, -1, 0, false, false);
            stage = 100;
        } else if (stage == 100) {
            if (driveTrainEncoder < (driveTrainGoal + 1000)) {
                countRings = false;
            }
            if (driveTrainEncoder < driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 103;
            }
        } else if (stage == 103) {
            driveTrainGoal = driveTrainEncoder + 400;
            driveTrain.arcadeDrive(0, 0, -.7, false, false);
            stage = 107;
        } else if (stage == 107) {
            if (driveTrainEncoder > driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 110;
            }
        } else if (stage == 110) {
            armGotoShoot = false;
            armPower = 1.0;
            pusherMotor.manual(0.0, 0.0, true);
            stage = 120;
        } else if (stage == 120) {
            if (arm.getEncoder() > -100) {
                armPower = 0.0;
                pusherMotor.manual(0.0, 0.0, false);

                switch (ringCount) {
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
        } else if (stage == 125) { //if 4 rings
            driveTrainGoal = driveTrainEncoder - 4250;
            driveTrain.arcadeDrive(0, -1, 0, false, false);
            stage = 140;
        } else if (stage == 135) {
            driveTrainGoal = driveTrainEncoder - 4000;
            driveTrain.arcadeDrive(0, -1, 1, false, false);
            stage = 140;
        } else if (stage == 140) {
            if (driveTrainEncoder < driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 150; //going to drop goal
            }
        } else if (stage == 150) {
            wobbleRelease.wobbleRelease();
            expirationTime = runtime.time() + 2.0;
            stage = 160;
        } else if (stage == 160) {
            if ((runtime.time() > expirationTime)) {
                switch (ringCount) {
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
        } else if (stage == 170) { // 1 ring
            driveTrainGoal = driveTrainEncoder + 4000;
            driveTrain.arcadeDrive(0, 1, 0, false, false);
            stage = 190;
        } else if (stage == 180) { // 4 rings
            driveTrainGoal = driveTrainEncoder + 2000;
            driveTrain.arcadeDrive(0, 1, 0, false, false);
            stage = 190;
        } else if (stage == 190) { //stopping
            if (driveTrainEncoder > driveTrainGoal) {
                driveTrain.arcadeDrive(0, 0, 0, false, false);
                stage = 200;
            }
        }

    }


    private int powerShoot(int level) {
        return powerShoot(level, driveTrainEncoder);
    }


    public int powerShoot(int level, double driveEncoder){
        driveTrainEncoder = driveEncoder;
        double turnSpeed = .25;

        if(level == 60) {
            driveTrainGoal = driveTrainEncoder + 340; //TURNING 1
            driveTrain.arcadeDrive(turnSpeed, 0, 0, false, true);
            level = 65;
        }
        else if(level == 65){
            if(driveTrainEncoder > driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, true);
                level = 70;
            }
        }
        else if(level == 70){
            pusherGoal = pusherMotor.getEncoder() + 228; ////   SHOOTING 1
            pusherMotor.manual(1, 0.0, false);
            level = 71;
        }
        else if(level == 71){
            if(pusherMotor.getEncoder() > pusherGoal){
                pusherMotor.manual(0.0, 0.0, false);
                level = 72;
            }
        }
        else if(level == 72){
            driveTrainGoal = driveTrainEncoder + 100;//// TURNING 2
            driveTrain.arcadeDrive(turnSpeed, 0, 0, false, true);
            level = 73;
        }
        else if(level == 73){
            if(driveTrainEncoder > driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, true);
                level = 74;
            }
        }
        else if(level == 74){
            pusherGoal = pusherMotor.getEncoder() + 228; ////   SHOOTING 2
            pusherMotor.manual(1, 0.0, false);
            level = 75;
        }
        else if(level == 75){
            if(pusherMotor.getEncoder() > pusherGoal){
                pusherMotor.manual(0.0, 0.0, false);
                level = 77;
            }
        }
        else if(level == 77){
            driveTrainGoal = driveTrainEncoder + 115;//// TURNING 3
            driveTrain.arcadeDrive(turnSpeed, 0, 0, false, true);
            level = 78;
        }
        else if(level == 78){
            if(driveTrainEncoder > driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, true);
                level = 79;
            }
        }
        else if(level == 79){
            pusherGoal = pusherMotor.getEncoder() + 200; ////   SHOOTING 3
            pusherMotor.manual(1, 0.0, false);
            level = 80;
        }
        else if(level == 80){
            if(pusherMotor.getEncoder() > pusherGoal){
                pusherMotor.manual(0.0, 0.0, false);
                level = 81;
            }
        }
        else if(level == 81){
            flyWheel.manual(false, false, true);
            pusherMotor.manual(0, 0, true);
            level = 83;
        }
        else if(level == 83){
            driveTrainGoal = driveTrainEncoder - 550;//// TURNING BACK
            driveTrain.arcadeDrive(-.3, 0, 0, false, true);
            level = 85;
        }
        else if(level == 85){
            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(0, 0, 0, false, true);
                level = 90; //ihebnibge
            }
        }
        return level;
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


