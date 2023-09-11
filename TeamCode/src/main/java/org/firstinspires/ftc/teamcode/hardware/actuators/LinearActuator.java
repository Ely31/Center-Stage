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

public class LinearActuator {
    DcMotorEx motor;
    private final MotorConstants motorConstants = new MotorConstants();
    String name;
    public double GEARBOX_RATIO;
    public double EXTERNAL_GEAR_RATIO = 1.0 / 1.0;
    public double CIRCUMFERENCE; // In inches
    public double TICKS_PER_REV;
    public double TICKS_PER_INCH;
    double maxDistance;
    double minDistance;

    double targetDistance = 0;

    // Variables to save read data to so we only have to read from the motor once a loop
    double currentPosition; // In ticks
    double currentPower;
    double currentCurrent; // current current, funny
    DcMotorSimple.Direction currentDirection;
    DcMotor.RunMode currentMode;

    public PIDCoefficients Coeffs;
    double fCoefficient;
    public PIDFController Controller;
    // Must call setCoefficients to use any pid features
    public void setCoefficients(PIDCoefficients coefficients) {
        Coeffs = coefficients;
        Controller = new PIDFController(Coeffs);
    }
    public void setfCoefficient(double f){
        fCoefficient = f;
    }

    public void zero(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setReversed(){
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Constructors
    public LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio, double circumference) {
        this.name = name;
        EXTERNAL_GEAR_RATIO = externalGearRatio;
        CIRCUMFERENCE = circumference;
        setTicks(gearboxRatio);

        motor = hardwareMap.get(DcMotorEx.class, name);
        zero();
    }
    public LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double circumference) {
        this.name = name;
        EXTERNAL_GEAR_RATIO = 1;
        CIRCUMFERENCE = circumference;
        setTicks(gearboxRatio);

        motor = hardwareMap.get(DcMotorEx.class, name);
        zero();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
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

        Controller.setTargetPosition(targetDistance);
        motor.setPower(Controller.update(getCurrentDistance()) + fCoefficient);
    }

    // DANGEROUS!
    public void setRawPowerDangerous(double power){
        motor.setPower(power);
    }

    // Position things
    public void setLimits(double min, double max){
        minDistance = min;
        maxDistance = max;
    }
    public double getMaxDistance() {return maxDistance;}
    public  double getMinDistance() {return minDistance;}
    public double getTargetDistance() {return targetDistance;}
    public double getCurrentDistance() {return currentPosition / TICKS_PER_INCH;}


    // PID positon methods
    public void setDistance(double distance) { // Make sure to use .setLimits before using this
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetDistance = Utility.clipValue(minDistance, maxDistance, distance);
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
        TICKS_PER_INCH = TICKS_PER_REV / (CIRCUMFERENCE);
    }
    public void displayDebugInfo(Telemetry telemetry) {
        // Fill up telemetry with all the info you could ever want
        // All the "%.3f" bits make things look a lot nicer by limiting the digits to 3 after the decimal point
        telemetry.addData("Current distance", "%.3f", getCurrentDistance());
        telemetry.addData("Target distance", "%.3f", getTargetDistance());
        telemetry.addData("distance error","%.3f", Controller.getLastError());
        telemetry.addData("Min", minDistance);
        telemetry.addData("Max", maxDistance);
        telemetry.addData("Ticks per inch", "%.3f",TICKS_PER_INCH);
        telemetry.addData("Ticks per rev", "%.3f",TICKS_PER_REV);
        telemetry.addData("Gearbox ratio", "%.1f",GEARBOX_RATIO);
        telemetry.addData("Lef", motor.getPower());
    }
}
