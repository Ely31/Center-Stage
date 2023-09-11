package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.Actuator;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
@TeleOp(group = "test")
public class ActuatorTest extends LinearOpMode {
    // Pre init
    Actuator actuator;
    TimeUtil timeUtil = new TimeUtil();
    // Fsm setup stuff
    enum Mode{
        BASIC,
        RTP,
        VELOCITY,
        PID,
        LINEAR
    }
    Mode mode = Mode.BASIC;

    public static double min = 0;
    public static double max = 360;
    public static double pos1 = 0;
    public static double pos2 = 360;
    public static double dist1 = 0;
    public static double dist2 = 300;
    private double velo = 0;
    public static double maxPower = 1;
    public static double gearboxRatio = 3.7;
    public static double externalRatio = 1.0/1.0;
    public static double diameter = 4;
    public static String name = "carousel";
    public static com.acmerobotics.roadrunner.control.PIDCoefficients angleCoeffs = new PIDCoefficients(0.001,0,0);
    public static com.acmerobotics.roadrunner.control.PIDCoefficients linearCoeffs = new PIDCoefficients(0,0,0);

    Gamepad prevGamepad = new Gamepad();
    Gamepad currentGamepad = new Gamepad();
    @Override
    public void runOpMode(){
        // Init
        PhotonCore.enable();
        actuator = new Actuator(hardwareMap, name, gearboxRatio);
        actuator.setDiameter(diameter);
        actuator.setAngularLimits(min,max);
        //actuator.setAngleCoefficients(angleCoeffs);
        //actuator.setLinearCoefficients(linearCoeffs);

        // Make telemetry good
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();
        timeUtil.resetTimer();

        while (opModeIsActive()){
            // Teleop code
            // Gamepad shenannigans
            try{
                prevGamepad.copy(currentGamepad);
                currentGamepad.copy(gamepad1);
            } catch (RobotCoreException ignored){}

            // Update everything
            actuator.update();
            timeUtil.update();

            actuator.setAngularLimits(min, max);
            actuator.setAngleCoefficients(angleCoeffs);
            actuator.setLinearCoefficients(linearCoeffs);

            // Fsm to switch modes
            switch (mode){
                case BASIC:
                    // Toggle direction with start
                    if (currentGamepad.start && !prevGamepad.start) {
                       if (actuator.getDirection() == DcMotorSimple.Direction.FORWARD) actuator.setDirection(DcMotorSimple.Direction.REVERSE);
                       else actuator.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    // Control power with sticks, right stick is half power
                    actuator.setPower((-currentGamepad.left_stick_y) + (-currentGamepad.right_stick_y *0.5));

                    if (shareJustPressed()) {
                        gamepad1.rumble(200);
                        actuator.stop();
                        mode = Mode.PID;
                    }
                    break;

                case RTP:
                    if (currentGamepad.dpad_left) actuator.runToAngleRTP(pos1, maxPower);
                    if (currentGamepad.dpad_right) actuator.runToAngleRTP(pos2, maxPower);

                    if (shareJustPressed()){
                        gamepad1.rumble(200);
                        actuator.stop();
                        mode = Mode.VELOCITY;
                    }
                    break;
                case VELOCITY:
                    if (currentGamepad.dpad_right) velo += 0.02;
                    if (currentGamepad.dpad_left) velo -= 0.02;

                    actuator.setVelocity(velo);
                    
                    if (shareJustPressed()){
                        gamepad1.rumble(200);
                        actuator.setVelocity(0);
                        velo = 0;
                        actuator.stop();
                        mode = Mode.PID;
                    }
                    break;

                case PID:
                    if (currentGamepad.dpad_left) actuator.setAnglePID(pos1);
                    if (currentGamepad.dpad_right) actuator.setAnglePID(pos2);

                    if (shareJustPressed()){
                        gamepad1.rumble(200);
                        actuator.stop();
                        mode = Mode.LINEAR;
                    }
                    break;

                case LINEAR:
                    if (currentGamepad.dpad_left) actuator.setDistance(dist1);
                    if (currentGamepad.dpad_right) actuator.setDistance(dist2);

                    if (shareJustPressed()){
                        gamepad1.rumble(200);
                        actuator.stop();
                        mode = Mode.BASIC;
                    }
                    break;
            }


            telemetry.addLine("<font color=#19cce8>switch modes by pressing back/share</font>");
            telemetry.addData("<font color=Magenta>mode</font>", mode);

            actuator.displayDebugInfo(telemetry);
            telemetry.addData("average loop time", "%.1f",timeUtil.getAverageLoopTime());

            telemetry.update();
        }
    }
    // So I don't have to write this every state switch
    boolean shareJustPressed(){return currentGamepad.share && !prevGamepad.share;}
}