package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

public class Imu {
    BNO055IMU imu;
    String name;

    double rawAngle;
    double angle;
    double offset;

    public Imu(HardwareMap hardwareMap, String name){
        this.name = name;
        imu = hardwareMap.get(BNO055IMU.class, name);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
    public double getAngle(){return angle;}
    public void zeroHeading(){offset = -rawAngle;}
    // I had stuff here for the second and third angles but my os got wiped and I'm too lazy to rewrite it because we'll probably never use it

    // example axes remap: remapAxes(AxesOrder.YXZ, AxesSigns.NPN);
    public void remapAxes(AxesOrder order, AxesSigns signs){
        BNO055IMUUtil.remapAxes(imu, order, signs);
    }

    public void update(){
        rawAngle = imu.getAngularOrientation().firstAngle;
        angle = rawAngle + offset;
    }
}
