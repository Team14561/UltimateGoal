package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RingSensor {
    ColorSensor colorSensor;
    Telemetry telemetry;

    public RingSensor(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, RobotMap.COLOR_SENSOR);

    }

    public int isOrange(){
        boolean orange;
        if(colorSensor.red() > 500) {
            orange = true;
        } else {
            orange = false;
        }
        return(colorSensor.red());

    }

    public void broadcastColor(){
        telemetry.addData("Red Value: ", isOrange());
    }

}
