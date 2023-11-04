package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ScoringMech {
    Lift lift;
    Arm arm;
    // Constructor
    public ScoringMech(HardwareMap hwmap){
        lift = new Lift(hwmap);
        arm = new Arm(hwmap);
    }
    public ScoringMech (){}

    // Functions from the arm class
    public void setBottomState(boolean state){
        arm.setBottomGripperState(state);
    }
    public boolean getBottomState(){
        return arm.getBottomGripperState();
    }
    public double getPivotPos(){
        return arm.getPivotPos();
    }
    public boolean getConeStatus(){
        return arm.pixelIsInBottom();
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
        arm.pivotScore();
        // More stuff to do
    }

    public void retract(){
        lift.retract();
        arm.pivotGoToIntake();
    }
    public void retract(double armTimerMs){
        armToIntakePos();
        if (armTimerMs > Arm.pivotActuationTime){
            retractLift();
        }
    }
    public void zeroLift(){lift.zero();}
    public void retractLift(){lift.retract();}
    public void armToIntakePos(){arm.pivotGoToIntake();}
    public void preMoveArm(){arm.preMove();}

    // ESSENTIAL to call this function every loop
    public void update(){lift.update();}

    // DANGEROUS!
    public void setRawLiftPowerDangerous(double power){lift.setRawPowerDangerous(power);}

    // Stuff the ds with telemetry if we want
    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("SCORING MECH");
        telemetry.addData("bottom state", getBottomState());
        arm.displayDebug(telemetry);
        lift.disalayDebug(telemetry);
    }
}
