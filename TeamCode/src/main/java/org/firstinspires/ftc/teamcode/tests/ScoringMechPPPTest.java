package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ScoringMech3;

@TeleOp(name="",group="test")
public class ScoringMechPPPTest extends LinearOpMode {
    // Pre-init
    ScoringMech3 scoringMech;
    @Override
    public void runOpMode() {
        // Init
        scoringMech = new ScoringMech3(hardwareMap);

        waitForStart();
        // Pre-run
        while (opModeIsActive()) {
            // TeleOp loop
            scoringMech.setPPPState(!gamepad1.a);

            scoringMech.displayDebug(telemetry);
            telemetry.update();
        }
    }
}
