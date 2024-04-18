package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "test")
public class HeadingCorrectionTest extends LinearOpMode {
    double in;
    double out;

    double constant = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()){
            in += gamepad1.left_stick_y*constant;
            in += gamepad1.right_stick_y*constant*0.1;

            out = convert();

            telemetry.addData("IN", in);
            telemetry.addData("OUT", out);
            telemetry.addData("modulused", in % (2*Math.PI));
            telemetry.update();
        }
    }
    public double convert(){
        boolean ver = false;
        if (ver) {

            // Try to fix steering deposit calibration being wrong
            double chunk;
            double correctedHeading = in;
            if (Math.abs(in) > (2 * Math.PI)) {
                chunk = in % (2 * Math.PI);
                if (in < 0) {
                    correctedHeading = chunk;
                } else {
                    correctedHeading = -chunk;
                }
            }
            return correctedHeading;
        } else {
        // Try to fix steering deposit calibration being wrong
        double in = this.in;
        double correctedHeading = Math.acos(Math.cos(in));

        return correctedHeading;
        }
    }
}
