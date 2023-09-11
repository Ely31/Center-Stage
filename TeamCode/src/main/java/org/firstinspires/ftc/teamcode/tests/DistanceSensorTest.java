package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.sensors.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Disabled
@TeleOp(name="",group="test")
public class DistanceSensorTest extends LinearOpMode {
    // Pre-init
    RevDistanceSensor sensor;
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Init
        sensor = new RevDistanceSensor(hardwareMap, "distance");
        telemetry.addLine("initialized");
        waitForStart();
    
        // Pre-run
        timer.reset();
        while (opModeIsActive()) {
            // Autonomous instructions
            timeUtil.update(timer.milliseconds());
            sensor.update();
            telemetry.update();

            sensor.displayDebugInfo(telemetry);
            telemetry.addData("loop time", timeUtil.getAverageLoopTime());
        }
    }
}
