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
    private double armEncoderStart;
    private double armEncoderEnd = 1317.0;
    private double height;
    public int ringNumber;

    // Object variables mimicing gamepad control
    double armPower = 0.0;
    boolean armGotoShoot = false;

    public RingSenseAuton (HardwareMap hardwareMap,
                            Telemetry telemetry, ElapsedTime runtime){
        this.runtime = runtime;
        heightSensor = new HeightSensor(hardwareMap,telemetry);
        arm = new Arm(hardwareMap, telemetry);
    }

    public void mainStages() {
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
            armEncoderStart = arm.getEncoder();
            armPower = 0.5;
            expirationTime = runtime.time() + 5.0;
            stage = 1;
        }
        if else (stage == 1) {
           if (runtime.time > expirationTime) {
               armPower = 0.0;
               stage = 2;
           }
        }
        if else (stage == 2) {
            encoderGoal = encoderValue + 100;
            //???
            expirationTime = runtime.time() + 2.0;
            stage = 3;
        }
        //???
        if else (stage == 3) {
            if (runtime.time > expirationTime) {
                armPower = 0.0;
                stage = 4;
            }
        }
        if else (stage == 4) {
            height = HeightSensor.getHeight();
            if (Height < 100) {
                ringNumber = 3;
                stage = 8
            }
            else {
                stage = 5;
            }
        }
        if else (stage == 5) {
            encoderGoal = encoderValue + 10;
            //???
            expirationTime = runtime.time() + 1.0;
            stage = 6;
        }
        //???
        if else (stage == 6) {
            if (runtime.time > expirationTime) {
                armPower = 0.0;
                stage = 7;
            }
        }
        if else (stage == 7) {
            height = HeightSensor.getHeight();
            if (Height < 100) {
                ringNumber = 1;
                stage = 8;
            }
            else {
                ringNumber = 0;
                stage = 8;
            }
        }
        if else (stage == 8) {

        }
    }
}


