package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivingInstructions {
    public static void printDrivingInstructions(Telemetry telemetry){
        telemetry.addLine();
        telemetry.addLine("DRIVING INSTRUCTIONS");
        telemetry.addLine("These instructions are likely to be out of date, I don't update them often");
        telemetry.addLine();
        telemetry.addLine("Gamepad 1 controls:");
        telemetry.addLine("Driving: Left stick is translation, right stick x is rotation. Use the right trigger to slow down."
                + "\ncalibrate feild-centric with the share button after you point the bot with the intake facing... err... I don't know where");
        telemetry.addLine();
        telemetry.addLine("Lift and arm: extend and retract with the left bumper, drop both pixels with the right."
                + "\nDrop the bottom and top ones with square and triangle respectively.");
        telemetry.addLine();
        telemetry.addLine("Intake: Toggle on and off with cross, reverse with circle.");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("Gamepad 2 controls:");
        telemetry.addLine("Make adjustments to the lift extended height with dpad up and down. This works even when the lift is retracted.");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("There's more details to the exact behavior that you'll learn as you drive, but I don't want to write them down");
    }
}
