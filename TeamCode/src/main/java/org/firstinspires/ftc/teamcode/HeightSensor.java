package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HeightSensor {
    DistanceSensor heightSensor;
    Telemetry telemetry;

    public HeightSensor(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        heightSensor = hardwareMap.get(DistanceSensor.class, RobotMap.HEIGHT_SENSOR);

    }

    public double getHeight(){
        return heightSensor.getDistance(DistanceUnit.INCH);
    }

    public void broadcastHeight(){
        telemetry.addData("Distance Sensor", getHeight());
    }

}
