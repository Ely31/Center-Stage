package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivingInstructions {
    public static void printDrivingInstructions(Telemetry telemetry){
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("Gamepad 1 controls:");
        telemetry.addLine("Driving: Left stick is translation, right stick x is rotation. Use the right trigger to slow down.");
        telemetry.addLine("calibrate feild-centric with the share button after you point the bot with the claw facing towards you");
        telemetry.addLine();
        telemetry.addLine("Lift and arm: toggle the claw open and closed with the left bumper, and toggle extend/retract with the left trigger");
        telemetry.addLine("Switch levels with buttons cross, square, triangle, and circle.");
        telemetry.addLine("Ground level is cross, levels get higher as you travel clockwise along the four buttons");
        telemetry.addLine();
        telemetry.addLine("Gamepad 2:");
        telemetry.addLine("make fine adjustments to the current height with dpad up and down.");
        telemetry.addLine("The height tweaks made are saved and will persist until the opmode is stopped.");
        telemetry.addLine("Change the stach index (which preset grabbing height the robot retracts to) with cross and triangle");
        telemetry.addLine("This allows you to grab off cone stacks");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("There's more details to the exact behavior of the code that you'll learn as you drive, but I don't want to write them down");
        telemetry.addLine();
        telemetry.addLine("Note: these instructions may be out of date, I don't update them often");
    }
}
