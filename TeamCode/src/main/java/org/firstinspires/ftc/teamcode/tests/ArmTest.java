package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name="",group="test")
public class ArmTest extends LinearOpMode {
    // Pre-init
    Arm arm;
    @Override
    public void runOpMode() {
        // Init
    arm = new Arm(hardwareMap);

        waitForStart();
        // Pre-run
        while (opModeIsActive()) {
            // TeleOp loop

            if (gamepad1.x) {
                arm.closeClaw();
            } else {
                arm.openClaw();
            }

            if (gamepad1.dpad_left) arm.grabPassthrough();
            else if (gamepad1.dpad_up) arm.scorePassthrough();
           /*
            else {
                arm.setPivotPos(gamepad1.right_trigger+.05);
                arm.setEndPos(gamepad1.left_trigger);
            }*/

            telemetry.addData("Pivot Pos", arm.getPivotPos());
            telemetry.addData("End Pos", arm.getEndPos());
            telemetry.update();
        }
    }
}
