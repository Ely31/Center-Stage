package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TapeMeasure {
    DcMotor tapeMeasureMotor;
    Servo tapeMeasureRelease;
    public static double holdTM = 0;
    public static double releaseTM = 1;

    //1150rpm
    //4in diameter wheel
    //5.2:1
    enum ShootyBoi{
        YEET,
        HOLD
    }

    ShootyBoi shootyBoi = ShootyBoi.YEET;

    public TapeMeasure(HardwareMap hwmap){
       tapeMeasureMotor = hwmap.get(DcMotor.class, "tapeMotor");
       tapeMeasureMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       tapeMeasureRelease = hwmap.get(Servo.class, "tapeServo");
       tapeMeasureMotor.setPower(0);
    }

    public void zeroTapeMeasureMotor(){ tapeMeasureMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void zeroPower(){tapeMeasureMotor.setPower(0);}

    public void releaseTM(){tapeMeasureRelease.setPosition(releaseTM);}

    public int getPos(){return tapeMeasureMotor.getCurrentPosition();}

    public void fuckinYeeeet(){tapeMeasureMotor.setPower(-1);}
    public void noYeet(){tapeMeasureMotor.setPower(1);}

    public void yeetKids(){
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
