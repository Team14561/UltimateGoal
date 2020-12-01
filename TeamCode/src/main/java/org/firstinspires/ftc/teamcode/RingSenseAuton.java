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

    public int mainStages(){
        arm.autonMaintain();


        if (stage == 0){
            armEncoderStart = arm.getEncoder;
            arm.setPower(0.2);
            expirationTime = runtime.time() + 5.0;
            stage = 1;
        }
        //get values to look for jump
        else if (stage == 1){
            oldHeight = HeightSensor.getHeight;
            sensorWait = runtime.time() + 0.1;
            stage = 2:
        }
        else if (stage == 2) {
            if (runtime.time() > sensorWait) {
                newHeight = HeightSensor.getHeight;
                stage = 3;
            }
        }
        //if jump or time runs out stop arm and move on
        else if (stage == 3) {
            if ( oldHeight - newHeight > 50) || runtime.time() > expirationTime){
                arm.setpower(0.0);
                armEncoderEnd = arm.getEncoder;
                stage = 4;
            }
            else {
                stage = 1;
            }
        }
        //figure out how far arm went and determine number of rings
        else if (stage == 4) {
            if (armEncoderStart - armEncoderEnd < 300) {
                ringNumber = 3;
            }
            if (armEncoderStart - armEncoderEnd  > 300) {
                ringNumber = 1;
            }
            //no end value set (timed out)
            if (armEncoderEnd == 1317.0 ) {
                ringNumber = 0;
            }
            return ringNumber;
            stage = 5;
        }

    }
}


