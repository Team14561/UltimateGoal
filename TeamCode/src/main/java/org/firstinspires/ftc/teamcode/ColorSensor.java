package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorSensor {
    ColorSensor colorSensor;
    Telemetry telemetry;

    public colorSensor(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        colorsensor = hardwareMap.get(ColorSensor.class, RobotMap.COLOR_SENSOR);

    }

    public double getColor(){
        return (colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }

    public void broadcastColor(){
        telemetry.addData("Color Sensor", getColor());
    }

}
