package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Usage: Implements flipper controller.
 *
 *
 *
 * Author: Daniel
 */

public class FlipperController {

    private Servo flipper1;
    private boolean flipped = false;
    private Telemetry telemetry;

    public FlipperController(Servo flipper1,  Telemetry telemetry) {
        this.flipper1 = flipper1;
        this.telemetry = telemetry;
    }

    public void SetPosition(double flipper1Position, double flipper2Position) {
        telemetry.addData("FlipperController", "SetPosition: %f, %f", flipper1Position, flipper2Position);
        flipper1.setPosition(flipper1Position);
    }

    public void BeginFlip() {
        telemetry.addData("FlipperController", "BeginFlip: %b", flipped);
        if (flipped) {
            flipper1.setPosition(0.8);
        }
        else{
            flipper1.setPosition(0);
        }

        flipped = !flipped;
    }
}
