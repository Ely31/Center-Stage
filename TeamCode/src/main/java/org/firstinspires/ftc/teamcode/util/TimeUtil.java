package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class TimeUtil {
    private final int endgameTime = 120;
    private final int endgameWarningTime = 112;
    private ElapsedTime timer = new ElapsedTime();

    public Gamepad.RumbleEffect endgameWarningRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1,1,500)
            .addStep(0,0,400)
            .addStep(1,1,500)
            .build();

    public Gamepad.RumbleEffect endgameRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0,1,2000)
            .build();

    public Gamepad.LedEffect endgameLightEffect = new Gamepad.LedEffect.Builder()
            // Flash teal, pause, and flash orange, and repeat that a few times
            .addStep(0,1,1,100)
            .addStep(0,0,0,100)
            .addStep(1,0.6,0,100)
            .addStep(0,0,0,100)
            // End cycle
            .addStep(0,1,1,100)
            .addStep(0,0,0,100)
            .addStep(1,0.6,0,100)
            .addStep(0,0,0,100)
            // End cycle
            .addStep(0,1,1,100)
            .addStep(0,0,0,100)
            .addStep(1,0.6,0,100)
            .addStep(0,0,0,100)
            // End cycle
            .addStep(0,1,1,100)
            .addStep(0,0,0,100)
            .addStep(1,0.6,0,100)
            .addStep(0,0,0,100)
            // End cycle
            .addStep(0,1,1,100)
            .addStep(0,0,0,100)
            .addStep(1,0.6,0,100)
            .addStep(0,0,0,100)
            // End cycle
            .addStep(0,1,1,100)
            .addStep(0,0,0,100)
            .addStep(1,0.6,0,100)
            .addStep(0,0,0,100)
            // End cycle
            // I wish there was a shorter way of writing all those flashes
            .build();

    double latestLoopTime = 0;
    private double lastTime = 0;
    public ArrayList<Double> lastLoopTimes = new ArrayList<>();
    private double averageLoopTime = 0;

    public enum Period
    {
        TELEOP,
        ENDGAMEWARNING,
        ENDGAME
    }
    // Make a current and last period for rising edge detectors
    private volatile  Period currentPeriod;
    private volatile  Period lastPeriod;

    private boolean justEnteredEndgameWarning = false;
    private boolean justEnteredEndgame = false;

    public Period getPeriod(){
        return currentPeriod;
    }
    public boolean isInEndgame(){
        return (currentPeriod == Period.ENDGAME);
    }
    public boolean isInEndgameWarning(){
        return currentPeriod == Period.ENDGAMEWARNING;
    }
    public boolean isInTeleop(){
        return currentPeriod == Period.TELEOP;
    }

    public boolean justEnteredEndgameWarning(){
        return justEnteredEndgameWarning;
    }
    public boolean justEnteredEndgame(){
        return justEnteredEndgame;
    }

    public double getAverageLoopTime(){
        return averageLoopTime;
    }
    public double getLatestLoopTime(){
        return latestLoopTime;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(double currentTime){
        // Set which game period we're in by checking the time
        if ((currentTime/1000) < endgameWarningTime) currentPeriod = Period.TELEOP;
        if ((currentTime/1000) > endgameWarningTime && (currentTime/1000) < endgameTime) currentPeriod = Period.ENDGAMEWARNING;
        if ((currentTime/1000) > endgameTime) currentPeriod = Period.ENDGAME;

        // Get how long the loop took
        latestLoopTime = currentTime - lastTime;
        lastTime = currentTime;

        // If the array is full (every loop except the first 5), make room for the new time
        if (lastLoopTimes.size() >= 50){
            lastLoopTimes.remove(0);
        }
        // Add the latest time to the array
        lastLoopTimes.add(latestLoopTime);

        // Calculate the average of all entries in the array
        double totalOfTimes = 0;
        for (int i = 0; i< lastLoopTimes.size(); i++){
            totalOfTimes += lastLoopTimes.get(i);
        }
        averageLoopTime = totalOfTimes / lastLoopTimes.size();

        // Rising edge detectors
        // I like the look of if-else formatting better, but I suppose this is nicer code
        justEnteredEndgameWarning = currentPeriod == Period.ENDGAMEWARNING && lastPeriod == Period.TELEOP;
        justEnteredEndgame = currentPeriod == Period.ENDGAME && lastPeriod == Period.ENDGAMEWARNING;

        lastPeriod = currentPeriod;
    }
    public void update(){
        update(timer.milliseconds());
    }

    // Rumble and flash when endgame is near
    public void updateGamepads(Gamepad g1, Gamepad g2){
        if (justEnteredEndgameWarning()){
            g1.runRumbleEffect(endgameWarningRumbleEffect);
            g1.runLedEffect(endgameLightEffect);
            g2.runRumbleEffect(endgameWarningRumbleEffect);
            g2.runLedEffect(endgameLightEffect);
        }
        if (justEnteredEndgame()){
            g1.runRumbleEffect(endgameRumbleEffect);
            g1.runLedEffect(endgameLightEffect);
            g2.runRumbleEffect(endgameRumbleEffect);
            g2.runLedEffect(endgameLightEffect);
        }
    }
    public void updateAll(double currentTime, Gamepad g1, Gamepad g2){
        update(currentTime);
        updateGamepads(g1, g2);
    }
}
