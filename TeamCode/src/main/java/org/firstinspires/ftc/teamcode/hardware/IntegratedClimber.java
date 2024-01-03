package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntegratedClimber {
    DcMotor climberMotor;
    public IntegratedClimber(HardwareMap hwmap){
        climberMotor = hwmap.get(DcMotor.class, "climber");
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //zero();
    }

    public void zero(){
        climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPower(double power){
        climberMotor.setPower(power);
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("CLIMBER");
        telemetry.addData("Power", climberMotor.getPower());
    }
}
