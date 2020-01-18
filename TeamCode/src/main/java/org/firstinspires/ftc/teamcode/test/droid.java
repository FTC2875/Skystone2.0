package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.ftccommon.SoundPlayer;


import java.io.File;

public class droid{
    private String soundPath = "/FIRST/blocks/sounds";
    private File droidFile = new File( soundPath + "/droid.wav");

    public String hwmap = new String();

    public void playDroid(String hwmap) {

        // Make sure that the sound files exist on the phone

        // Display sound status
 //      telemetry.addData("gold sound", goldFound ? "Found" : "NOT Found \nCopy gold.wav to " + soundPath);
 //       SoundPlayer.getInstance().startPlaying(hwmap, droidFile);

    }
}
