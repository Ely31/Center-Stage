package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DroneLauncher {
    Servo launcher;
    public static double releasePos = 0.8;
    public static double holdPos = 0.5;

    public DroneLauncher(HardwareMap hwmap){
        launcher = hwmap.get(Servo.class, "launcher");
        hold();
    }

    public void hold(){
        launcher.setPosition(holdPos);
    }
    public void release(){
        launcher.setPosition(releasePos);
    }
}
