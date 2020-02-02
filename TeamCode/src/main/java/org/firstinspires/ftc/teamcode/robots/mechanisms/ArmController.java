package org.firstinspires.ftc.teamcode.robots.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private Telemetry telemetry;

    public ArmController(Servo armbase, Servo armjoint, Telemetry telemetry) {
        this.armbase = armbase;
        this.armjoint = armjoint;
        this.telemetry = telemetry;
    }

    public void SetPosition(double armBasePosition, double armJointPosition) {
        telemetry.addData("ArmController:", "SetPosition: ", armBasePosition, armJointPosition);
        armbase.setPosition(armBasePosition);
        armjoint.setPosition(armJointPosition);
    }

    public void SetBasePosition(double armBasePosition){
        telemetry.addData("ArmController:", "SetBasePosition", armBasePosition);
        armbase.setPosition(armBasePosition);
    }

    public void SetJointPosition(double armJointPosition){
        telemetry.addData("ArmController:", "SetJointPosition", armJointPosition);
        armjoint.setPosition(armJointPosition);
    }

    public double getArmJointPosition() {
        return armjoint.getPosition();
    }
    public double getArmBasePosition() { return armbase.getPosition(); }

    public void BeginGrip() {
        // TODO implement grip
        // armbase
        telemetry.addData("ArmController:", "BeginGrip");
    }

    public void BeginRelease() {
        // TODO implement release
        telemetry.addData("ArmController:", "BeginRelease");
    }

    public void BeginRaise() {
        // TODO implement raise
        telemetry.addData("ArmController:", "BeginRaise");
    }
}
