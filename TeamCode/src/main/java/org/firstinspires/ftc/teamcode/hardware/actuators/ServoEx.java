package org.firstinspires.ftc.teamcode.hardware.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoEx {
    private ServoImplEx servo;
    private double travel;
    private double offset = 0; // For when the horn can't be put in the perfect spot

    public double getOffset() {
        return offset;
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }

    // Constructors
    public ServoEx(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(ServoImplEx.class, name);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Extend range to get the most angle out of it
        travel = 180;
    }
    public ServoEx(HardwareMap hardwareMap, String name, double travel) {
        servo = hardwareMap.get(ServoImplEx.class, name);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Extend range to get the most angle out of it
        this.travel = travel;
    }

    // Methods
    public void setRange(double min, double max) {
        servo.setPwmRange(new PwmControl.PwmRange(min, max));
    }
    public PwmControl.PwmRange getRange() {
        return servo.getPwmRange();
    }

    // Set position and angle things
    public void setPosition(double position) { // Normal control, 0 to 1 input
        servo.setPosition(position);
    }
    public void setAngle(double angle) {
        servo.setPosition((angle / travel) + offset);
    }
    public double getAngle() {
        return servo.getPosition() * travel;
    }

    // Direction things
    public void setDirection(boolean isInverted) {
        servo.setDirection(isInverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }
    public Servo.Direction getDirection() {
        return servo.getDirection();
    }

    // Telemetry stuff
    public void displayDebugInfo(Telemetry telemetry) {
        if (telemetry != null) {
            telemetry.addData("Position", servo.getPosition());
            telemetry.addData("Angle", getAngle());
            telemetry.addData("Direction", servo.getDirection());
            telemetry.addData("Range", getRange());
            telemetry.addData("Travel", travel);
            telemetry.addData("Offset", offset);
        }
    }
}
