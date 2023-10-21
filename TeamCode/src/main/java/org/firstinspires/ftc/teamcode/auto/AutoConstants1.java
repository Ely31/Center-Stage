package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;

import java.util.Random;

public class AutoConstants1 {
    SampleMecanumDrive drive;
    int randomMessageIndex;
    // Constructor
    public AutoConstants1(SampleMecanumDrive drive){
        this.drive = drive;
        randomMessageIndex = new Random().nextInt(messageList.length);
    }

    // 1 is red or left, -1 is blue or right
    int alliance = 1;
    public int getAlliance() {return alliance;}
    public void setAlliance(int alliance) {this.alliance = alliance;}

    public boolean allianceToBool(){
        return alliance != 1;
    }
    public String allianceToString(){
        if (alliance == 1) return "Left, red terminal";
        else return "Right, blue terminal";
    }

    int zone = 1;
    public int getDropLocation() {
        return zone;
    }

    boolean WingSide = false;
    public boolean isWingSide() {
        return WingSide;
    }
    public void setWingSide(boolean wingSide) {
        WingSide = wingSide;
    }

    int numCycles = 4;
    public int getNumCycles() {return numCycles;}
    public void setNumCycles(int numCycles) {this.numCycles = numCycles;}


    public void updateDropLocationFromVisionResult(int visionResult){
        switch(visionResult){
            case 0:
                // Switch 1 and 3 if we're on blue terminal
                if (getAlliance() == 1){
                    zone = 0;
                } else {
                    zone = 2;
                }
                break;
            case 1:
                // We don't have to change the middle positon however
                zone = 1;
                break;
            case 3:
                // Switch 3 and 1 if we're on blue terminal
                if (getAlliance() == 1) {
                    zone = 2;
                } else {
                    zone = 0;
                }
                break;
        }
    }

    public double grabApproachVelo = 25;

    // Pose2d's
    public Pose2d startPos = new Pose2d(-35.8, -63* alliance, Math.toRadians(-90* alliance));

    // Set parkPos to a default to avoid null issues
    public Pose2d dropPos = new Pose2d(-57, -12* alliance, Math.toRadians(180* alliance));
    public Pose2d spikeMarkPos = new Pose2d(-57, -12* alliance, Math.toRadians(180* alliance));

    public Pose2d[] dropPositions;
    public Pose2d[] spikeMarkPositions;

    public void updateParkPos(int posIndex){
        dropPositions = new Pose2d[]{
                // Pos 1
                new Pose2d(-61, -11 * alliance, Math.toRadians(180 * alliance)),
                // Pos 2
                new Pose2d(-37, -11 * alliance, Math.toRadians(180 * alliance)),
                // Pos 3
                new Pose2d(-13, -11 * alliance, Math.toRadians(180 * alliance))
        };
        spikeMarkPositions = new Pose2d[]{
                // Pos 1
                new Pose2d(-61, -11 * alliance, Math.toRadians(180 * alliance)),
                // Pos 2
                new Pose2d(-37, -11 * alliance, Math.toRadians(180 * alliance)),
                // Pos 3
                new Pose2d(-13, -11 * alliance, Math.toRadians(180 * alliance))
        };
        // Grab the correct pos from the array and set parkPos to it
        dropPos = dropPositions[posIndex];
        spikeMarkPos = spikeMarkPositions[posIndex];
    }

    public TrajectorySequence dropOffPurplePixel;
    public TrajectorySequence park;

    public void updateTrajectories() {

        startPos = new Pose2d(-35.8, -63* alliance, Math.toRadians(-90* alliance));

        dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-35.8, -9.7* alliance, Math.toRadians(130* alliance)))
                .build();

        park = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                .setTangent(Math.toRadians(-130 * alliance))
                .splineToSplineHeading(new Pose2d(-58, -12.2 * alliance, Math.toRadians(180 * alliance)), Math.toRadians(180 * alliance))
                .lineTo(new Vector2d(-64.8, -12.2 * alliance))
                .build();
    }

    public void saveAutoPose(){
        AutoToTele.endOfAutoPose = drive.getPoseEstimate();
        AutoToTele.endOfAutoHeading = drive.getPoseEstimate().getHeading();
    }


    // Telemetry stuff
    public void addTelemetry(Telemetry telemetry){
        telemetry.addLine(allianceToString());
        telemetry.addData("Alliance side as integer", getAlliance());
        telemetry.addData("Park zone", zone);
        telemetry.addData("Number of cycles", getNumCycles());
        telemetry.addLine(ramdomAutoCheckMessage());
    }
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
        //look up the index of the randomly generated number in the array of messages and return that message
        return messageList[randomMessageIndex];
    }
}
