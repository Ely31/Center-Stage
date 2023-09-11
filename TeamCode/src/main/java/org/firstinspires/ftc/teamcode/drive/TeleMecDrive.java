package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.AutoToTele;

// This has way more functions than you really need
public class TeleMecDrive {
    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;
    private PIDFCoefficients coefficients = new PIDFCoefficients(20,0,8,11.9); // To be used in RUE mode, tuned in roadrunner

    public double lfMaxRPMFraction;
    public DcMotor.RunMode runMode;
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior;

    private BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;
    private double heading;
    public double getHeading(){
        return heading;
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
        if (mode == DcMotor.RunMode.RUN_USING_ENCODER) setCoefficients(coefficients);

        runMode = mode;
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        lf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);

        zeroPowerBehavior = behavior;
    }
    public void setCoefficients(PIDFCoefficients coefficients){
        lf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        lb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        rf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        rb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }
    public void setMaxRPMFraction(double maxRPMFraction){
        // Aahhh why do I have to type this so many times
        MotorConfigurationType lfConfig = lf.getMotorType().clone();
        lfConfig.setAchieveableMaxRPMFraction(maxRPMFraction);
        lf.setMotorType(lfConfig);

        lfMaxRPMFraction = lfConfig.getAchieveableMaxRPMFraction(); // Just to see if this works

        MotorConfigurationType lbConfig = lb.getMotorType().clone();
        lbConfig.setAchieveableMaxRPMFraction(maxRPMFraction);
        lb.setMotorType(lbConfig);

        MotorConfigurationType rfConfig = rf.getMotorType().clone();
        rfConfig.setAchieveableMaxRPMFraction(maxRPMFraction);
        rf.setMotorType(rfConfig);

        MotorConfigurationType rbConfig = rb.getMotorType().clone();
        rbConfig.setAchieveableMaxRPMFraction(maxRPMFraction);
        rb.setMotorType(rbConfig);
    }


    // Constructor
    public TeleMecDrive(HardwareMap hardwareMap, double slowFactor) {
        lf = hardwareMap.get(DcMotorEx.class,"lf");
        lb = hardwareMap.get(DcMotorEx.class,"lb");
        rf = hardwareMap.get(DcMotorEx.class,"rf");
        rb = hardwareMap.get(DcMotorEx.class,"rb");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set the max speed to %95 when using RUE
        setMaxRPMFraction(0.95);
        // Configure motor behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Use bulk reads
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        //initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        this.slowFactor = slowFactor;
    }

    // Driving methods
    public void driveFieldCentric(double x, double y, double turn, double slowInput){

        slowInput = ((-1 + slowFactor) * slowInput)+1;

        heading = (imu.getAngularOrientation().firstAngle + (AutoToTele.endOfAutoHeading + Math.toRadians(-90)) + headingOffset);

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


    public void setWheelPowers(double LF, double LB, double RF, double RB){
        lf.setPower(LF);
        lb.setPower(LB);
        rf.setPower(RF);
        rb.setPower(RB);
    }
    public void resetHeading(){
        AutoToTele.endOfAutoHeading = (Math.PI/2); // Unit circle coming in handy
        headingOffset = -(imu.getAngularOrientation().firstAngle + (AutoToTele.endOfAutoHeading + Math.toRadians(90 * AutoToTele.allianceSide)));
    }
}
