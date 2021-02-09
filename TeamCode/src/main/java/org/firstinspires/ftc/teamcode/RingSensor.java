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

    public boolean isOrange(){
        boolean orange;
        if(colorSensor.red() > 35) {
            orange = true;
        } else {
            orange = false;
        }
        return orange;

    }

    public void broadcastColor(){
        telemetry.addData("Ring Detected: ", isOrange());
        //telemetry.addData("Red Value: ", colorSensor.red());
        //telemetry.addData("Blue Value: ", colorSensor.blue());
        //telemetry.addData("Green Value: ", colorSensor.green());
    }

}
