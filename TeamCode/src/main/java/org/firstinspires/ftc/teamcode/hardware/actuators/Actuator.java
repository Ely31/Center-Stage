package org.firstinspires.ftc.teamcode.hardware.actuators;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

import java.util.Objects;

public class Actuator {
    DcMotorEx motor;
    private final MotorConstants motorConstants = new MotorConstants();
    String name;
    public double GEARBOX_RATIO;
    public double EXTERNAL_GEAR_RATIO = 1.0 / 1.0;
    public double TICKS_PER_REV;
    public double TICKS_PER_DEGREE;
    // For linear motion with a rack and pinion or spool and string, in centimeters
    private double EFFECTIVE_DIAMETER;
    private double EFFECTIVE_CIRCUMFERENCE;
    private double TICKS_PER_CM;
    double maxAngle;
    double minAngle;
    double maxDistance;
    double minDistance;
    double MAX_POWER = 1;

    double targetAngle = 0;
    double targetDistance = 0;

    // Variables to save read data to so we only have to read from the motor once a loop
    double currentPosition; // In ticks
    double currentVelo;
    double currentPower;
    double currentCurrent; // current current, funny
    DcMotorSimple.Direction currentDirection;
    DcMotor.RunMode currentMode;

    // Pid stuff
    enum PidMode{
        ANGULAR,
        LINEAR,
        DISABLED
    }
    PidMode pidMode = PidMode.DISABLED;
    public PidMode getPidMode(){return pidMode;}

    public PIDCoefficients angleCoeffs;
    public PIDCoefficients distanceCoeffs;
    public PIDFController angleController;
    public PIDFController distanceController;
    // Must call setCoefficients to use any pid features
    public void setAngleCoefficients(PIDCoefficients coefficients) {
        angleCoeffs = coefficients;
        angleController = new PIDFController(angleCoeffs);
    }
    public void setLinearCoefficients(PIDCoefficients coefficients) {
        distanceCoeffs = coefficients;
        distanceController = new PIDFController(angleCoeffs);

    }
    // On-hub velo
    public void setVelocityCoefficients(PIDFCoefficients coefficients) {motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);}

    // Constructors
    public Actuator(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio) {
        this.name = name;
        EXTERNAL_GEAR_RATIO = externalGearRatio;
        setTicks(gearboxRatio);

        motor = hardwareMap.get(DcMotorEx.class, name);
        zero();
    }
    public Actuator(HardwareMap hardwareMap, String name, double gearboxRatio) {
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
        currentVelo = motor.getVelocity();
        currentCurrent = motor.getCurrent(CurrentUnit.AMPS);
        currentPower = motor.getPower();
        currentDirection = motor.getDirection();
        currentMode = motor.getMode();
        switch (pidMode){
            case ANGULAR:
                angleController.setTargetPosition(targetAngle);
                motor.setPower(angleController.update(getCurrentAngle()));
                break;

            case LINEAR:
                distanceController.setTargetPosition(targetDistance);
                motor.setPower(distanceController.update(getCurrentDistance()));
                break;

            case DISABLED:
                break;
        }
    }

    // Power and other primative things
    public void setPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }
    public void setMaxPower(double maxPower){
        MAX_POWER = maxPower;
    }

    public void stop() {
        motor.setPower(0);
    }
    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setDirection(DcMotorSimple.Direction direction) {motor.setDirection(direction);}
    public DcMotorSimple.Direction getDirection(){return currentDirection;}
    public DcMotor.RunMode getRunMode(){return currentMode;}

    // Position things
    public void setAngularLimits(double min, double max){
        minAngle = min;
        maxAngle = max;
    }
    public double getMaxAngle() {return maxAngle;}
    public  double getMinAngle() {return minAngle;}
    public double getTargetAngle() {return targetAngle;}
    public double getCurrentAngle() {return currentPosition / TICKS_PER_DEGREE;}

    // RTP position methods
    // Make sure to use .setLimits before using this
    public void runToAngleRTP(double angle) {runToAngleRTP(angle,1);}
    public void runToAngleRTP(double angle, double power) {
        angle = Utility.clipValue(minAngle, maxAngle, angle);
        runToAngleRTP_Free(angle, power);
    }
    // "Free" means no limits on rotation apply. Use for mechanisms that can rotate continuously.
    public void runToAngleRTP_Free(double angle) {runToAngleRTP_Free(angle, 1);}
    public void runToAngleRTP_Free(double angle, double power) {
        pidMode = PidMode.DISABLED;
        targetAngle = angle;
        motor.setTargetPosition((int) (targetAngle * TICKS_PER_DEGREE));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power*MAX_POWER);
    }
    // Velocity things
    public void setVelocity(double velocity) { // In rotations per second
        pidMode = PidMode.DISABLED;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(velocity * TICKS_PER_REV);
    }
    public double getVelocityInRotations() {
        return currentVelo / TICKS_PER_REV;
    }
    public double getVelocityInDegrees() {
        return currentVelo / TICKS_PER_DEGREE;
    }

    // PID positon methods
    public void setAnglePID(double angle) { // Make sure to use .setLimits before using this
        pidMode = PidMode.ANGULAR;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = Utility.clipValue(minAngle, maxAngle, angle);
    }

    // Linear motion methods
    public void setDiameter(double diameter){
        // Convert diameter into ticks per cm
        EFFECTIVE_DIAMETER = diameter;
        EFFECTIVE_CIRCUMFERENCE = EFFECTIVE_DIAMETER * Math.PI;
        TICKS_PER_CM = TICKS_PER_REV / EFFECTIVE_CIRCUMFERENCE;
    }
    public double getCurrentDistance(){return currentPosition / TICKS_PER_CM;}
    public double getTargetDistance(){return targetDistance;}
    public void setDistanceLimits(double min, double max){
        minDistance = min;
        maxDistance = max;
    }

    public void setDistance(double distance) { // Make sure to use .setLimits before using this
        pidMode = PidMode.LINEAR;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetDistance = Utility.clipValue(minDistance, maxDistance, distance);
    }

    // Miscellaneous methods
    public double getCurrent() {return currentCurrent;}
    public double getPower() {return currentPower;}

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
        telemetry.addData("Current velo (rotations per second)", "%.3f",getVelocityInRotations());
        telemetry.addData("Target angle", "%.3f", getTargetAngle());
        telemetry.addData("Angle error", "%.3f",angleController.getLastError());
        telemetry.addData("Target distance", getTargetDistance());
        telemetry.addData("Distance error", distanceController.getLastError());
        telemetry.addData("PID mode", getPidMode());
        telemetry.addData("Min", minAngle);
        telemetry.addData("Max", maxAngle);
        telemetry.addData("Runmode", getRunMode());
        telemetry.addData("Direction", getDirection());
        telemetry.addData("Power", "%.3f",getPower());
        telemetry.addData("Current", "%.3f",getCurrent());
        telemetry.addData("Ticks per degree", "%.3f",TICKS_PER_DEGREE);
        telemetry.addData("Ticks per rev", "%.3f",TICKS_PER_REV);
        telemetry.addData("Ticks per cm", "%.3f",TICKS_PER_CM);
        telemetry.addData("Gearbox ratio", "%.1f",GEARBOX_RATIO);
    }
}
