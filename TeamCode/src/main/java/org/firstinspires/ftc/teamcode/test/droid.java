package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;

public class droid extends LinearOpMode {
    private String soundPath = "/FIRST/blocks/sounds";
    private File goldFile = new File(soundPath + "/droid.wav");
    private File silverFile = new File( soundPath + "/silver.wav");

    // Declare OpMode members.
    private boolean isX = false;    // Gamepad button state variables
    private boolean isB = false;

    private boolean wasX = false;   // Gamepad button history variables
    private boolean WasB = false;

    @Override
    public void runOpMode() {

        // Make sure that the sound files exist on the phone
        boolean goldFound = goldFile.exists();
        boolean silverFound = silverFile.exists();

        // Display sound status
        telemetry.addData("gold sound", goldFound ? "Found" : "NOT Found \nCopy gold.wav to " + soundPath);
        telemetry.addData("silver sound", silverFound ? "Found" : "NOT Found \nCopy silver.wav to " + soundPath);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to continue");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X or B to play sounds.");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // say Silver each time gamepad X is pressed (This sound is a resource)
            if (silverFound && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverFile);
                telemetry.addData("Playing", "Silver File");
                telemetry.update();
            }

            // say Gold each time gamepad B is pressed  (This sound is a resource)
            if (goldFound && (isB = gamepad1.b) && !WasB) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldFile);
                telemetry.addData("Playing", "Gold File");
                telemetry.update();
            }
        }
    }
}