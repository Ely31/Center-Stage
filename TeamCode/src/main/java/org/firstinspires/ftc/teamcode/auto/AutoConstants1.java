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

    // 1 is red, -1 is blue
    private int alliance = 1;
    public int getAlliance() {return alliance;}
    public void setAlliance(int alliance) {this.alliance = alliance;}
    public boolean allianceToBool(){
        return alliance == 1;
    }
    public String allianceToString(){
        if (alliance == 1) return "red alliance";
        else return "blue alliance";
    }

    boolean wingSide = false;
    public boolean isWingSide() {
        return wingSide;
    }
    public void setWingSide(boolean wingSide) {
        this.wingSide = wingSide;
    }

    boolean parkingClose = true;
    public boolean isParkingClose() {
        return parkingClose;
    }
    public void setParkingClose(boolean parkingClose) {
        this.parkingClose = parkingClose;
    }

    private int correctedSpikeMarkPos = 1;
    public int getCorrectedSpikeMarkPos() {
        return correctedSpikeMarkPos;
    }

    // This is used so we don't put our pixel in the same slot as our partner. Hopefully. We'll see.
    int dropOffset = 0;
    boolean dropIsOffset = false;
    public void setDropIsOffset(boolean value){
        dropIsOffset = value;
    }
    public boolean isDropOffset(){
        return dropIsOffset;
    }

    private int numCycles = 4;
    public int getNumCycles() {return numCycles;}
    public void setNumCycles(int numCycles) {this.numCycles = numCycles;}

    private int delaySeconds = 0; // To be used to avoid collisions
    public int getDelaySeconds(){return delaySeconds;}
    public void setDelaySeconds(int seconds){
        delaySeconds = seconds;
    }


    public void updateCorrectedSpikeMarkPos(int visionResult){
        switch(visionResult){
            case 1:
                // Switch 1 and 3 if we're on blue terminal
                if (getAlliance() == 1){
                    correctedSpikeMarkPos = 1;
                } else {
                    correctedSpikeMarkPos = 3;
                }
                break;
            case 2:
                // We don't have to change the middle positon however
                correctedSpikeMarkPos = 2;
                break;
            case 3:
                // Switch 3 and 1 if we're on blue terminal
                if (getAlliance() == 1) {
                    correctedSpikeMarkPos = 3;
                } else {
                    correctedSpikeMarkPos = 1;
                }
                break;
        }
    }

    // Pose2d's
    public Pose2d startPos = new Pose2d(-35.8, -63* alliance, Math.toRadians(-90* alliance));

    public TrajectorySequence dropOffPurplePixel;
    public TrajectorySequence scoreYellowPixel;
    public TrajectorySequence park;

    public void updateTrajectories() {
        // Change start pose, pretty important
        if (isWingSide()) {
            startPos = new Pose2d(-35.25, -61.5 * alliance, Math.toRadians(90 * alliance));
        } else {
            startPos = new Pose2d(11.75, -61.5 * alliance, Math.toRadians(90 * alliance));
        }
        // Switch this so we don't drop our pixel in the same spot as our partner
        dropOffset = (isDropOffset() ? 3 : 0);

        // Ahhhh we have to have six unique purple pixel trajectories
        double yellowPixelYCoord = -29;
        double yellowPixelXCoord = 54;

        if (isWingSide()){
            // Wing side
            double afterPurpleTangent = 180;
            switch (correctedSpikeMarkPos){
                case 1:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(-46, -29 * alliance, Math.toRadians(-90 * alliance)))
                            .build();
                    yellowPixelYCoord = -29-dropOffset;
                    afterPurpleTangent = 90;
                    break;
                case 2:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            //.lineToSplineHeading(new Pose2d(-38.5, -26 * alliance, Math.toRadians(0 * alliance)))
                            .lineToSplineHeading(new Pose2d(-40, -18.5 * alliance, Math.toRadians(-89.9 * alliance))) // 89.9 so it turns CW
                            .build();
                    yellowPixelYCoord = -29-6-dropOffset;
                    afterPurpleTangent = 90;
                    break;
                default:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(-32.5, -36 * alliance, Math.toRadians(0 * alliance)))
                            .build();
                    yellowPixelYCoord = -29-12-dropOffset;
                    break;
            }

            scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                    // Line up with the row os tiles to go under the stage door
                    .setTangent(Math.toRadians(afterPurpleTangent*alliance))
                    .splineToSplineHeading(new Pose2d(-29, -13*alliance, Math.toRadians(0*alliance)), Math.toRadians(0*alliance))
                    // Drive under the door
                    .splineTo(new Vector2d(12, -14*alliance), Math.toRadians(0*alliance))
                    // To the board
                    .splineToSplineHeading(new Pose2d(yellowPixelXCoord, yellowPixelYCoord*alliance, Math.toRadians(0*alliance)), Math.toRadians(0*alliance))
                    .build();

        } else {
            // Board side
            switch (correctedSpikeMarkPos){
                case 1:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(9.5, -35*alliance, Math.toRadians(180*alliance)))
                            .build();
                    yellowPixelYCoord = -29-dropOffset;
                    break;
                case 2:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(17, -28*alliance, Math.toRadians(180*alliance)))
                            .build();
                    yellowPixelYCoord = -29-6-dropOffset;
                    break;
                default:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(28.50, -31.80*alliance, Math.toRadians(180*alliance)))
                            .build();
                    yellowPixelYCoord = -29-12-dropOffset;
                    break;
            }

            scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                    // Having this wait at the beginning causes an empty sequence exception for some reason
                    // I have a feeling Noah wrote it in a hacky way
                    //.waitSeconds(0.5)
                    //.setTangent(Math.toRadians(0*alliance))
                    //.splineToSplineHeading(new Pose2d(46.48, -35.99*alliance, Math.toRadians(0*alliance)), Math.toRadians(0*alliance))
                    //.splineTo(new Vector2d(yellowPixelXCoord, yellowPixelYCoord*alliance), Math.toRadians(0*alliance))
                    // Go right to the board
                    .lineToSplineHeading(new Pose2d(yellowPixelXCoord, yellowPixelYCoord*alliance, Math.toRadians(0*alliance)))
                    .build();
        }

        if (parkingClose){
            park = drive.trajectorySequenceBuilder(scoreYellowPixel.end())
                    .setTangent(Math.toRadians(180 * alliance))
                    .splineToSplineHeading(new Pose2d(50, -60 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(90 * alliance))
                    .build();
        } else {
            park = drive.trajectorySequenceBuilder(scoreYellowPixel.end())
                    .setTangent(Math.toRadians(180 * alliance))
                    .splineToSplineHeading(new Pose2d(50, -13 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(-90 * alliance))
                    .build();
        }
    } // End of updateTrajectories

    public void saveAutoPose(){
        AutoToTele.endOfAutoPose = drive.getPoseEstimate();
        AutoToTele.endOfAutoHeading = drive.getPoseEstimate().getHeading();
    }

    // Telemetry stuff
    public void addTelemetry(Telemetry telemetry){
        // Write the alliance in its color
        telemetry.addLine("<font color =#"+ (allianceToBool() ? "ff0000>" : "0099ff>") + allianceToString() + "</font>" );
        telemetry.addData("Alliance side as integer", getAlliance());
        telemetry.addData("Side", (isWingSide() ? "Wing" : "Board")); // First time using this funny switchy thing
        telemetry.addData("Number of cycles", getNumCycles());
        telemetry.addData("Delay in seconds", getDelaySeconds());
        telemetry.addData("Parking close", isParkingClose());
        telemetry.addData("Corrected Spike Mark Pos", getCorrectedSpikeMarkPos());
        telemetry.addLine();
        telemetry.addLine(autoConfigToEnglish());
        telemetry.addLine();
        telemetry.addLine(ramdomAutoCheckMessage());
    }
    String[] messageList = {
            "CHECK THE AUTO, REMEMBER NANO FINALS 3!",
            "Ok apparently it was Nano Finals 2",
            "Run the right auto kids!",
            "Is it red? is it blue?",
            "Is it left? is it right?",
            "Are you SURE this is the program you want?",
            "¡Ejecute el auto correcto!",
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
            "Don't pull a brainstormers FF finals",
            "Guys this auto's more complicated than last year",
            "Is it wing? is it board?"
    };
    String ramdomAutoCheckMessage(){
        //look up the index of the randomly generated number in the array of messages and return that message
        return messageList[randomMessageIndex];
    }
    String autoConfigToEnglish(){
        String spikeMarkDescription;
        String yellowPixelDescription;
        switch (getCorrectedSpikeMarkPos()){
            case 1:
                spikeMarkDescription = "closest to the wing";
                yellowPixelDescription = "closest to the center of the field";
                break;
            case 2:
                spikeMarkDescription = "in the center";
                yellowPixelDescription = "in the center";
                break;
            default:
                spikeMarkDescription = "closest to the board";
                yellowPixelDescription = "closest to the wall";
                break;
        }
        return (
                "You are on the " + allianceToString() + "."
                + " You are on the side of the field closest to the " + (isWingSide() ? "wing" : "board") + "."
                + " The bot will move the purple pixel to the spike mark " + spikeMarkDescription
                + " and score the yellow pixel " + yellowPixelDescription + "."
                + " The yellow pixel will be placed in that slot closest to " + (isDropOffset() ? "you" : "the center of the field") + "."
                + " The bot will then park closest to " + (isParkingClose() ? "you" : "the center of the field") + "."
                + "\n" + "Sound right?"
                );
    }
}
