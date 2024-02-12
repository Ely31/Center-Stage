package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "test")
public class GetCurrentTest extends LinearOpMode {
    DcMotorEx intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotorImplEx.class, "intake");

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Current", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
