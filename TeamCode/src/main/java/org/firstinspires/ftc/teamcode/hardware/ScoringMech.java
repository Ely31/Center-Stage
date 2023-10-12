package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ScoringMech {
    SingleMotorLift lift;
    Arm arm;
    // Constructor
    public ScoringMech(HardwareMap hwmap){
        lift = new SingleMotorLift(hwmap);
        arm = new Arm(hwmap);
    }

    // Functions from the arm class
    public void openClaw(){
        arm.openClaw();
    }
    public void closeClaw(){
        arm.closeClaw();
    }
    public void setClawState(boolean state){
        arm.setClawState(state);
    }
    public boolean getClawState(){
        return arm.getClawState();
    }
    public double getPivotPos(){
        return arm.getPivotPos();
    }
    public boolean getConeStatus(){
        return arm.coneIsInClaw();
    }

    // Functions from the lift class
    public void editExtendedPos(double step){
        lift.editExtendedPos(step);
    }
    public double getLiftHeight(){
        return lift.getHeight();
    }

    public void score(){
        lift.extend();
        // More stuff to do
    }

    public void retract(){
        lift.retract();
        arm.grabPassthrough();
    }
    public void retract(double v4bTimerMs){
        v4bToGrabbingPos();
        if (v4bTimerMs > Arm.pivotActuationTime){
            retractLift();
        }
    }
    public void zeroLift(){
        lift.zero();
    }

    public void retractLift(){
        lift.retract();
    }
    public void v4bToGrabbingPos(){
        arm.grabPassthrough();
    }

    // ESSENTIAL to call this function every loop
    public void update(){
        lift.update();
    }

    // DANGEROUS!
    public void setRawLiftPowerDangerous(double power){
        lift.setRawPowerDangerous(power);
    }

    // Stuff the ds with telemetry if we want
    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("SCORING MECH");
        telemetry.addData("claw state", getClawState());
        arm.displayDebug(telemetry);
        lift.disalayDebug(telemetry);
    }
}
