package org.firstinspires.ftc.teamcode.drive;

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

// This has way more functions than you really need
public class  TeleMecDrive {
    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;

    public DcMotor.RunMode runMode;
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior;

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

    public void setMotorMode(DcMotor.RunMode mode){
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);

        runMode = mode;
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        lf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);

        zeroPowerBehavior = behavior;
    }

    // Constructor
    public TeleMecDrive(HardwareMap hardwareMap, double slowFactor, boolean isProtobot) {
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
        //Set up the rev hub orientation
        if (isProtobot){
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        } else{
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        }
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetIMU();

        this.slowFactor = slowFactor;
    }

    // Driving methods
    public void driveFieldCentric(double x, double y, double turn, double slowInput){

        slowInput = ((-1 + slowFactor) * slowInput)+1;

        heading = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffset);

        // Matrix math I don't understand to rotate the joystick input by the heading
        rotX = x * Math.cos(-heading) - -y * Math.sin(-heading);
        rotY = x * Math.sin(-heading) + -y * Math.cos(-heading);

        double lfPower = rotY + rotX + turn;
        double lbPower = rotY - rotX + turn;
        double rfPower = rotY - rotX - turn;
        double rbPower = rotY + rotX - turn;

        lf.setPower(lfPower*slowInput);
        lb.setPower(lbPower*slowInput);
        rf.setPower(rfPower*slowInput);
        rb.setPower(rbPower*slowInput);
    }

    public void driveRobotCentric(double x, double y, double turn, double slowInput){
        slowInput = ((-1 + slowFactor) * slowInput)+1;

        double lfPower = y + x + turn;
        double lbPower = y - x + turn;
        double rfPower = y - x - turn;
        double rbPower = y + x - turn;

        lf.setPower(lfPower*slowInput);
        lb.setPower(lbPower*slowInput);
        rf.setPower(rfPower*slowInput);
        rb.setPower(rbPower*slowInput);
    }

    public void driveBoardLocked(double x, double y, double turn, double slowInput){
        // Only difference between this and driveRobotCentric is that the slow button only effects strafing
        // So that the control loops on heading and board distance don't get affected
        slowInput = ((-1 + slowFactor) * slowInput)+1;

        double lfPower = y + x*slowInput + turn*slowInput;
        double lbPower = y - x*slowInput + turn*slowInput;
        double rfPower = y - x*slowInput - turn*slowInput;
        double rbPower = y + x*slowInput - turn*slowInput;

        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rf.setPower(rfPower);
        rb.setPower(rbPower);
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
    }
}
