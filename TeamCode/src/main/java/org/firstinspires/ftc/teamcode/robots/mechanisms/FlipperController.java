package org.firstinspires.ftc.teamcode.robots.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Usage: Implements flipper controller.
 *
 *
 *
 * Author: Daniel
 */

public class FlipperController {

    private Servo flipper1;
    private Servo flipper2;
    private boolean flipped = false;

    public FlipperController(Servo flipper1, Servo flipper2) {
        this.flipper1 = flipper1;
        this.flipper2 = flipper2;
    }

    public void SetPosition(double flipper1Position, double flipper2Position) {
        flipper1.setPosition(flipper1Position);
        flipper2.setPosition(flipper2Position);
    }

    public void BeginFlip() {

        if (flipped) {
            flipper1.setPosition(0.8);
            flipper2.setPosition(0.8);
        }
        else{
            flipper1.setPosition(0);
            flipper2.setPosition(0);
        }

        flipped = !flipped;
    }
}
