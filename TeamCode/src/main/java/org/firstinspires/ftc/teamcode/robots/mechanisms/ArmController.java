package org.firstinspires.ftc.teamcode.robots.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Usage: Implements arm controller.
 *
 *
 *
 * Author: Daniel
 */

public class ArmController {

    private Servo armbase;
    private Servo armjoint; //upper near rotation point

    public ArmController(Servo armbase, Servo armjoint) {
        this.armbase = armbase;
        this.armjoint = armjoint;
    }

    public void SetPosition(double armBasePosition, double armJointPosition) {
        armbase.setPosition(armBasePosition);
        armjoint.setPosition(armJointPosition);
    }

    public void SetBasePosition(double armBasePosition){
        armbase.setPosition(armBasePosition);
    }

    public void SetJointPosition(double armJointPosition){
        armjoint.setPosition(armJointPosition);
    }

    public double getArmJointPosition() {
        return armjoint.getPosition();
    }
    public double getArmBasePosition() { return armbase.getPosition(); }

    public void BeginGrip() {
        // TODO implement grip
        // armbase
    }

    public void BeginRelease() {
        // TODO implement release
    }

    public void BeginRaise() {
        // TODO implement raise
    }
}
