package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;

import java.util.Random;

public class AutoConstants {
    SampleMecanumDrive drive;
    // Constructor
    public AutoConstants(SampleMecanumDrive drive){
        this.drive = drive;
        randomMessageIndex = new Random().nextInt(messageList.length);
    }

    // 1 is red or left, -1 is blue or right
    int side = 1;
    public int getSide() {return side;}
    public void setSide(int side) {this.side = side;}

    public boolean sideToBool(){
        return side != 1;
    }
    public String sideToString(){
        if (side == 1) return "Left, red terminal";
        else return "Right, blue terminal";
    }

    int numCycles = 4;
    public int getNumCycles() {return numCycles;}
    public void setNumCycles(int numCycles) {this.numCycles = numCycles;}

    int parkZone = 2;
    public int getParkZone() {
        return parkZone;
    }

    public void updateParkZoneFromVisionResult(int visionResult){
        switch(visionResult){
            case 1:
                // Switch 1 and 3 if we're on blue terminal
                if (getSide() == 1){
                    parkZone = 1;
                } else {
                    parkZone = 3;
                }
                break;
            case 2:
                // We don't have to change the middle positon however
                parkZone = 2;
                break;
            case 3:
                // Switch 3 and 1 if we're on blue terminal
                if (getSide() == 1) {
                    parkZone = 3;
                } else {
                    parkZone = 1;
                }
                break;
        }
    }

    public double grabApproachVelo = 25;

    // Pose2d's
    public Pose2d startPos = new Pose2d(-35.8, -63*side, Math.toRadians(-90*side));

    // Set parkPos to a default to avoid null issues
    public Pose2d parkPos = new Pose2d(-57, -12*side, Math.toRadians(180*side));

    public Pose2d[] parkPositions;

    public void updateParkPos(int posIndex){
        parkPositions = new Pose2d[]{
                // Pos 1
                new Pose2d(-61, -11 * side, Math.toRadians(180 * side)),
                // Pos 2
                new Pose2d(-37, -11 * side, Math.toRadians(180 * side)),
                // Pos 3
                new Pose2d(-13, -11 * side, Math.toRadians(180 * side))
        };
        // Grab the correct pos from the array and set parkPos to it
        parkPos = parkPositions[posIndex-1];
    }

    public TrajectorySequence driveToPreloadPos;
    public TrajectorySequence toStackFromPreload;
    public TrajectorySequence toStack;
    public TrajectorySequence toJunctionPressing;
    public TrajectorySequence park;
    public TrajectorySequence toStackImproved;
    public TrajectorySequence toStackFromPreloadImproved;


    public void updateTrajectories() {

        startPos = new Pose2d(-35.8, -63*side, Math.toRadians(-90*side));

        driveToPreloadPos = drive.trajectorySequenceBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-35.8, -9.7*side, Math.toRadians(130*side)))
                .build();

        toStackFromPreload = drive.trajectorySequenceBuilder(driveToPreloadPos.end())
                .setTangent(Math.toRadians(-130 * side))
                .splineToSplineHeading(new Pose2d(-58, -12.2 * side, Math.toRadians(180 * side)), Math.toRadians(180 * side))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
                .lineTo(new Vector2d(-64.8, -12.2 * side))
                .build();

        toJunctionPressing = drive.trajectorySequenceBuilder(toStackFromPreload.end())
                .lineToSplineHeading(new Pose2d(-39, -12.5 * side, Math.toRadians(-135 * side)))
                .build();

        toStack = drive.trajectorySequenceBuilder(toJunctionPressing.end())
                .setTangent(Math.toRadians(180 * side))
                .splineToSplineHeading(new Pose2d(-58, -12.1 * side, Math.toRadians(180 * side)), Math.toRadians(180 * side))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
                .lineTo(new Vector2d(-64.7, -12.1 * side))
                .build();

        park = drive.trajectorySequenceBuilder(toJunctionPressing.end())
                .lineToSplineHeading(parkPos)
                .build();

        toStackImproved = drive.trajectorySequenceBuilder(toJunctionPressing.end())
                .lineToSplineHeading(new Pose2d(-47, -11.4 * side, Math.toRadians(180 * side)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
                .splineToSplineHeading(new Pose2d(-64.7, -13.1 * side, Math.toRadians(180*side)), Math.toRadians(180*side))
                .build();

        toStackFromPreloadImproved = drive.trajectorySequenceBuilder(toJunctionPressing.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, 13.2))
                .lineToSplineHeading(new Pose2d(-50, -11.75 * side, Math.toRadians(180 * side)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
                .splineToSplineHeading(new Pose2d(-64.7, -13.1 * side, Math.toRadians(180*side)), Math.toRadians(180*side))
                .build();
    }

    public void saveAutoPose(){
        AutoToTele.endOfAutoPose = drive.getPoseEstimate();
        AutoToTele.endOfAutoHeading = drive.getPoseEstimate().getHeading();
    }


    // Telemetry stuff
    public void addTelemetry(Telemetry telemetry){
        telemetry.addLine(sideToString());
        telemetry.addData("Alliance side as integer", getSide());
        telemetry.addData("Park zone", parkZone);
        telemetry.addData("Number of cycles", getNumCycles());
        telemetry.addLine(ramdomAutoCheckMessage());
    }

    int randomMessageIndex;

    String[] messageList = {
            "CHECK THE AUTO, REMEMBER NANO FINALS 3!",
            "Run the right auto kids!",
            "Is it red? is it blue?",
            "Is it left? is it right?",
            "Are you SURE this is the program you want?",
            "Ejecute el auto correcto!",
            "올바른 자동 실행",
            "Oi mate, didjya checkit eh?",
            "What do those numbers say, hmmmm? Hmmmm?",
            "C'mon man, just take a second to read the stuff",
            "运行正确的自动",
            "Don't waste the potential of this bot",
            "How many cycles are we doin'?",
            "Where are we parkin'?",
            "Look. At. The. Side.",
            "Look at the bot, now look at the screen",
            "(insert mildly funny comment about auto)",
            "ELYYYYY...",
            "LUUUKEE...",
            ":)",
            "Don't lose worlds!",
            "Pay attention bro",
            "Don't pull a brainstormers FF finals"
    };

    String ramdomAutoCheckMessage(){
        // Generate a random number and look up that index in the array of messages

        return messageList[randomMessageIndex];
    }
}
