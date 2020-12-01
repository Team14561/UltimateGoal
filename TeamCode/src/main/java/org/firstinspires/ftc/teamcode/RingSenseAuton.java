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
    private double oldHeight;
    private double newHeight;
    public int ringNumber;
    private double sensorWait;

    public RingSenseAuton (HardwareMap hardwareMap,
                            Telemetry telemetry, ElapsedTime runtime){
        this.runtime = runtime;
        heightSensor = new HeightSensor(hardwareMap,telemetry);
        arm = new Arm(hardwareMap, telemetry);
    }

    public void mainStages() {
        arm.autonMaintain();

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
            arm.manual(0.5, false);
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
                arm.manual(0.0, false);
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
    }
}


