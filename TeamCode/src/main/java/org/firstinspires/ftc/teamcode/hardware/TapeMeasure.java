package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;

public class TapeMeasure {
    LinearActuator TMActuator;
    DcMotor tapeMeasureMotor;

    //1150rpm
    //4in diameter wheel
    //5.2:1
    public TapeMeasure(HardwareMap hwmap){
       tapeMeasureMotor = hwmap.get(DcMotor.class, "tapeMeasure");
       tapeMeasureMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void zeroPower(){tapeMeasureMotor.setPower(0);}
    public int getPos(){return tapeMeasureMotor.getCurrentPosition();}

    public void Yeeeeeet(){tapeMeasureMotor.setPower(1);}
    public void noYeet(){tapeMeasureMotor.setPower(-1);}

}
