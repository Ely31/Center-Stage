package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Climber {

    DcMotor climberMotor;
    public static double targetLiftHeight = 17;
    public static double hangingHeight = 17;

    public Climber(HardwareMap hwmap){
        climberMotor = hwmap.get(DcMotor.class, "climber");
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zero();
    }

    public void zero(){
        climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPower(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climberMotor.setPower(power);
    }
    public void setTargetPos(int pos){
        climberMotor.setTargetPosition(pos);
    }
    public void goToTargetPos(){
        climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(1);
    }
    public int getPos(){
        return climberMotor.getCurrentPosition();
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("CLIMBER");
        telemetry.addData("Power", climberMotor.getPower());
        telemetry.addData("Target pos", climberMotor.getTargetPosition());
        telemetry.addData("Lift target height", targetLiftHeight);
    }
}
