package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RevTouchSensor {
    com.qualcomm.robotcore.hardware.TouchSensor touchsensor;
    String name;
    boolean currentState;
    boolean lastState;
    boolean justPressed;
    boolean justReleased;

    public RevTouchSensor(HardwareMap hardwareMap, String name){
        this.name = name;
        touchsensor = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class, this.name);
    }

    public boolean isPressed(){return currentState;}
    public boolean wasJustPressed() {return justPressed;}
    public boolean wasJustReleased() {return justReleased;}

    // you MUST call update every loop
    public void update(){
        lastState = currentState;
        currentState = touchsensor.isPressed();

        // Rising edge detector
        justPressed = (!lastState && currentState);
        // Falling edge detector
        justReleased = (lastState && !currentState);
    }

    public void displayDebugInfo(Telemetry telemetry){
        telemetry.addData("isPressed", isPressed());
        telemetry.addData("was just pressed", wasJustPressed());
        telemetry.addData("was just released", wasJustReleased());
    }
}
