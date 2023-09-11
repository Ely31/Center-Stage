package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

public class RevColorSensor {
    RevColorSensorV3Ex colorSensor;
    String name;

    int alpha;
    int red;
    int green;
    int blue;
    Scalar scalar;

    public RevColorSensor(HardwareMap hardwareMap, String name){
        this.name = name;
        colorSensor = hardwareMap.get(RevColorSensorV3Ex.class, name);
    }

    public double getAlpha(){return alpha;}
    public double getRed(){return red;}
    public double getGreen(){return green;}
    public double getBlue(){return blue;}
    public Scalar getScalar(){return scalar;}

    // You MUST call update every loop
    public void update(){
        alpha = colorSensor.alpha();
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        scalar = new Scalar(red, green, blue);
    }

    public void displayDebugInfo(Telemetry telemetry){
        telemetry.addData("alpha", getAlpha());
        telemetry.addData("scalar", getScalar());
    }
}
