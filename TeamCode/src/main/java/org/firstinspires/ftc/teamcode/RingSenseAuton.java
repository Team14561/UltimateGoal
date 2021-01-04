package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RingSenseAuton {

    private AnalogInput potentiometer;
    private int stage = 0;
    private double expirationTime;
    private ElapsedTime runtime;
    private HeightSensor heightSensor;
    private Arm arm;
    private FlyWheel flyWheel;
    private PusherMotor pusherMotor;
    private Sweeper sweeper;
    private DriveTrain driveTrain;
    private double height;
    public int ringNumber;
    private double driveTrainEncoder;
    private double driveTrainGoal;
    Telemetry telemetry;


    // Object variables mimicing gamepad control
    double armPower = 0.0;
    boolean armGotoShoot = false;

    public RingSenseAuton (HardwareMap hardwareMap,
                            Telemetry telemetry, ElapsedTime runtime){
        this.runtime = runtime;
        this.telemetry = telemetry;
        heightSensor = new HeightSensor(hardwareMap,telemetry);
        arm = new Arm(hardwareMap, telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        flyWheel = new FlyWheel(hardwareMap, telemetry);
        pusherMotor = new PusherMotor(hardwareMap, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);

        //Get initial encoder value
        driveTrainEncoder = driveTrain.rightEncoder.getCurrentPosition();
    }

    public void mainStages() {
        driveTrainEncoder = driveTrain.rightEncoder.getCurrentPosition();

        arm.manual(armPower, armGotoShoot);

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
            armPower = -0.8;
            expirationTime = runtime.time() + 7.0;
            stage = 1;
        }
        else if (stage == 1) {
           if ( (runtime.time() > expirationTime) || (arm.getEncoder() < -800) ) {
               armPower = 0.0;
               stage = 2;
           }
        }
        else if (stage == 2) {
            arm.setEncoderGoal(RobotMap.FOUR_RING_HEIGHT);
            expirationTime = runtime.time() + 3.0;
            stage = 3;
        }
        else if (stage == 3) {
            if (runtime.time() > expirationTime) {
                stage = 4;
            }
        }
        else if (stage == 4) {
            height = heightSensor.getHeight();
            if (height < 100) {
                ringNumber = 4;
                stage = 8;
            }
            else {
                stage = 5;
            }
        }
        else if (stage == 5) {
            arm.setEncoderGoal(RobotMap.ONE_RING_HEIGHT);
            expirationTime = runtime.time() + 3.0;
            stage = 6;
        }
        else if (stage == 6) {
            if (runtime.time() > expirationTime) {
                stage = 7;
            }
        }
        else if (stage == 7) {
            height = heightSensor.getHeight();
            if (height < 9) {
                ringNumber = 1;
                stage = 8;
            }
            else {
                ringNumber = 0;
                stage = 8;
            }
        }
        else if (stage == 8) {
            telemetry.addData("Ring Number", ringNumber);
            stage = 8;
        }

        //move left out of starting area
        else if(stage == 9){
            driveTrainGoal = 100;

            if(driveTrainEncoder < driveTrainGoal){
                driveTrain.arcadeDrive(-1, 0, 0, false, false);
            } else {
                stage = 10;
            }
        }

        //move forward to shooting line
        else if(stage == 10)
        {
            driveTrainGoal = 100;

            if(driveTrainGoal < driveTrainGoal){
                driveTrain.arcadeDrive(0, 1, 0, false, false);
            } else {
                stage = 11;
            }
        }

        //strafe right to line up with shooting targets
        else if(stage == 11)
        {
            driveTrainGoal = 100;

            if(driveTrainGoal < driveTrainGoal){
                driveTrain.arcadeDrive(1, 0, 0, false, false);
            } else {
                stage = 12;
            }
        }

        //shoot rings
        else if(stage == 12){

            stage = 13;
        }

        //move to target zone specified
        else if(stage == 13){
            wobbleGoalMove(ringNumber);
            stage = 14;
        }

        else if(stage == 14){
            //drop wobble goal
            stage = 15;
        }

        else if(stage == 15){
            moveToEnd(ringNumber);
            stage = 16;
        }




        } // ending bracket for mainStages();

        private void wobbleGoalMove(int number) {
            switch(number){
                case 0:
                    //zone A
                    driveTrainGoal = 100;

                    while(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(1, 0, 0, false, false);
                    }

                    break;
                case 1:
                    //zone B
                    driveTrainGoal = 100;

                    while(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(0, 1, 0, false, false);
                    }

                    break;
                case 4:
                    //zone C
                    driveTrainGoal = 100;

                    while(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(1, 2, 0, false, false);
                    }


                    break;
            }
        }

        private void moveToEnd(int number){
            switch(number){
                case 0:
                    //back from zone A
                    driveTrainGoal = 100;

                    while(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(-1, 0, 0, false, false);
                    }

                    break;
                case 1:
                    //back from zone B
                    driveTrainGoal = 100;

                    while(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(0, -1, 0, false, false);
                    }

                    break;
                case 4:
                    //back from zone C
                    driveTrainGoal = 100;

                    while(driveTrainEncoder < driveTrainGoal){
                        driveTrain.arcadeDrive(-1, -2, 0, false, false);
                    }


                    break;

            }
        }


    }



