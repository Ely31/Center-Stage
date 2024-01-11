package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.AutoToTele;

@Config
public class TeleMecDrive2 {
    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;
    private IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection;
    RevHubOrientationOnRobot orientationOnRobot;

    private double heading;
    public double getHeading(){
        return heading;
    }
    // See https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
    public double getNormalizedHeading(){
        // reduce the angle
        double angle =  heading % 2*Math.PI;
        // force it to be the positive remainder, so that 0 <= angle < 360
        angle = (angle + 2*Math.PI) % 2*Math.PI;
        // force into the minimum absolute value residue class, so that -180 < angle <= 180
        if (angle > Math.PI) angle -= 2*Math.PI;

        return angle;
    }
    private double headingOffset = 0;

    private double rotX;
    private double rotY;

    private double slowFactor;

    private double targetHeading;
    public static PIDCoefficients headingCoeffs;
    private PIDFController headingController;

    public void setMotorMode(DcMotor.RunMode mode){
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        lf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
    }

    // Constructor
    public TeleMecDrive2(HardwareMap hardwareMap, double slowFactor) {
        lf = hardwareMap.get(DcMotorEx.class,"lf");
        lb = hardwareMap.get(DcMotorEx.class,"lb");
        rf = hardwareMap.get(DcMotorEx.class,"rf");
        rb = hardwareMap.get(DcMotorEx.class,"rb");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        // Configure motor behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Use bulk reads
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetIMU();

        this.slowFactor = slowFactor;

        headingCoeffs = new PIDCoefficients(0.05,0,0);
        headingController = new PIDFController(headingCoeffs);
    }

    // Driving methods
    public void driveFieldCentric(double x, double y, double turn, double slowInput){

        slowInput = ((-1 + slowFactor) * slowInput)+1;

        heading = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffset);

        targetHeading += turn * 0.01;
        headingController.setTargetPosition(targetHeading);
        double turnInput = headingController.update(heading);

        // Matrix math I don't understand to rotate the joystick input by the heading
        rotX = x * Math.cos(-heading) - -y * Math.sin(-heading);
        rotY = x * Math.sin(-heading) + -y * Math.cos(-heading);

        double lfPower = rotY + rotX + turnInput;
        double lbPower = rotY - rotX + turnInput;
        double rfPower = rotY - rotX - turnInput;
        double rbPower = rotY + rotX - turnInput;

        lf.setPower(lfPower*slowInput);
        lb.setPower(lbPower*slowInput);
        rf.setPower(rfPower*slowInput);
        rb.setPower(rbPower*slowInput);
    }

    public void resetIMU(){
        imu.resetYaw();
    }
    public void setHeadingOffset(double headingOffset){
        this.headingOffset = headingOffset;
    }
    public void resetHeadingOffset(){
        headingOffset = 0;
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("DRIVE");
        telemetry.addData("Heading in radians", getHeading());
        telemetry.addData("Heading in degrees", Math.toDegrees(getHeading()));
        telemetry.addData("Normalized heading in radians", getNormalizedHeading());
        telemetry.addData("End of auto heading", AutoToTele.endOfAutoHeading);
        telemetry.addData("End of auto heading in degrees", Math.toDegrees(AutoToTele.endOfAutoHeading));
        telemetry.addData("End of auto side", AutoToTele.allianceSide);
        telemetry.addData("Target Heading", targetHeading);
    }
}
