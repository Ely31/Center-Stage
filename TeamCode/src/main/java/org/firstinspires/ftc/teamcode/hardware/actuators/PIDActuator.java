package org.firstinspires.ftc.teamcode.hardware.actuators;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

import java.util.Objects;

public class PIDActuator {
    DcMotorEx motor;
    private final MotorConstants motorConstants = new MotorConstants();
    String name;
    public double GEARBOX_RATIO;
    public double EXTERNAL_GEAR_RATIO = 1.0 / 1.0;
    public double TICKS_PER_REV;
    public double TICKS_PER_DEGREE;
    double maxAngle;
    double minAngle;

    double targetAngle = 0;

    // Variables to save read data to so we only have to read from the motor once a loop
    double currentPosition; // In ticks
    double currentPower;
    double currentCurrent; // current current, funny
    DcMotorSimple.Direction currentDirection;
    DcMotor.RunMode currentMode;

    public PIDCoefficients angleCoeffs;
    public PIDFController angleController;
    // Must call setCoefficients to use any pid features
    public void setAngleCoefficients(PIDCoefficients coefficients) {
        angleCoeffs = coefficients;
        angleController = new PIDFController(angleCoeffs);
    }

    public void zero(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Constructors
    public PIDActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio) {
        this.name = name;
        EXTERNAL_GEAR_RATIO = externalGearRatio;
        setTicks(gearboxRatio);

        motor = hardwareMap.get(DcMotorEx.class, name);
        zero();
    }
    public PIDActuator(HardwareMap hardwareMap, String name, double gearboxRatio) {
        this.name = name;
        EXTERNAL_GEAR_RATIO = 1;
        setTicks(gearboxRatio);

        motor = hardwareMap.get(DcMotorEx.class, name);
        zero();
    }

    // you MUST call update EVERY loop for this stuff to work
    // it's done this way so that we only read from the motor once a loop, saving that data to variables,
    // so we don't hurt loop times by grabbing it multiple times.
    public void update(){
        currentPosition = motor.getCurrentPosition();
        currentCurrent = motor.getCurrent(CurrentUnit.AMPS);
        currentPower = motor.getPower();
        currentDirection = motor.getDirection();
        currentMode = motor.getMode();

        angleController.setTargetPosition(targetAngle);
        motor.setPower(angleController.update(getCurrentAngle()));
    }

    // Position things
    public void setAngularLimits(double min, double max){
        minAngle = min;
        maxAngle = max;
    }
    public double getMaxAngle() {return maxAngle;}
    public  double getMinAngle() {return minAngle;}
    public double getTargetAngle() {return targetAngle;}
    public double getCurrentAngle() {return currentPosition / TICKS_PER_DEGREE;}


    // PID positon methods
    public void setAnglePID(double angle) { // Make sure to use .setLimits before using this
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = Utility.clipValue(minAngle, maxAngle, angle);
    }


    private void setTicks(double gearboxRatio){
        GEARBOX_RATIO = gearboxRatio;
        if (GEARBOX_RATIO == 1) TICKS_PER_REV = motorConstants.BARE * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 3.7) TICKS_PER_REV = motorConstants.TICKS_37 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 5.2) TICKS_PER_REV = motorConstants.TICKS_52 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 13.7) TICKS_PER_REV = motorConstants.TICKS_137 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 19.2) TICKS_PER_REV = motorConstants.TICKS_192 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 26.9) TICKS_PER_REV = motorConstants.TICKS_269 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 50.9) TICKS_PER_REV = motorConstants.TICKS_509 * EXTERNAL_GEAR_RATIO;
        if (Objects.isNull(TICKS_PER_REV)) {
            throw new IllegalArgumentException("Invalid gearbox ratio");
        }
        TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    }
    public void displayDebugInfo(Telemetry telemetry) {
        // Fill up telemetry with all the info you could ever want
        // All the "%.3f" bits make things look a lot nicer by limiting the digits to 3 after the decimal point
        telemetry.addData("Current angle", "%.3f",getCurrentAngle());
        telemetry.addData("Target angle", "%.3f", getTargetAngle());
        telemetry.addData("Angle error","%.3f",angleController.getLastError());
        telemetry.addData("Min", minAngle);
        telemetry.addData("Max", maxAngle);
        telemetry.addData("Ticks per degree", "%.3f",TICKS_PER_DEGREE);
        telemetry.addData("Ticks per rev", "%.3f",TICKS_PER_REV);
        telemetry.addData("Gearbox ratio", "%.1f",GEARBOX_RATIO);
    }
}
