package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TapeMeasure {
    DcMotor tapeMeasureMotor;
    Servo tapeMeasureRelease;
    public static double holdTM = 0;
    public static double releaseTM = 1;

    enum ShootyBoi{
        YEET,
        HOLD
    }

    ShootyBoi shootyBoi = ShootyBoi.YEET;

    public TapeMeasure(HardwareMap hwmap){
       tapeMeasureMotor = hwmap.get(DcMotor.class, "TapeMeasureMotor");
       tapeMeasureMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       tapeMeasureRelease = hwmap.get(Servo.class, "tapeMeasureRelease");
       zeroTapeMeasureMotor();
    }

    public void zeroTapeMeasureMotor(){ tapeMeasureMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

    public void releaseTM(){tapeMeasureRelease.setPosition(releaseTM);}

    public int getPos(){return tapeMeasureMotor.getCurrentPosition();}

    public void executeBPM(){
        switch(shootyBoi){
            case YEET:
                tapeMeasureMotor.setTargetPosition(100);
                tapeMeasureMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tapeMeasureMotor.setPower(1);
                if(getPos() == 100){
                    shootyBoi = shootyBoi.HOLD;
                }

            case HOLD:
                tapeMeasureRelease.setPosition(holdTM);
        }
    }
}
