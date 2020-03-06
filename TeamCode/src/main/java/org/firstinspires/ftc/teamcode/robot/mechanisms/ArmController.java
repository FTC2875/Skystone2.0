package org.firstinspires.ftc.teamcode.robot.mechanisms;

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

    private Servo gripper;
    private Servo linkage; //upper near rotation point
    private Telemetry telemetry;

    public ArmController(Servo gripper, Servo linkage, Telemetry telemetry) {
        this.gripper = gripper;
        this.linkage = linkage;
        this.telemetry = telemetry;
    }

    public void SetPosition(double armBasePosition, double armJointPosition) {
        telemetry.addData("ArmController", "SetPosition: %d, %d", armBasePosition, armJointPosition);
        gripper.setPosition(armBasePosition);
        linkage.setPosition(armJointPosition);
    }

    public void SetBasePosition(double armBasePosition){
        telemetry.addData("ArmController", "SetBasePosition: %d", armBasePosition);
        gripper.setPosition(armBasePosition);
    }

    public void SetJointPosition(double armJointPosition){
        //telemetry.addData("ArmController", "SetJointPosition", armJointPosition);
        linkage.setPosition(armJointPosition);
    }

    public double getArmJointPosition() {
        return linkage.getPosition();
    }
    public double getArmBasePosition() { return gripper.getPosition(); }

    public void BeginGrip() {
        // TODO implement grip
        // gripper
        telemetry.addData("ArmController", "BeginGrip");
    }

    public void BeginRelease() {
        // TODO implement release
        telemetry.addData("ArmController", "BeginRelease");
    }

    public void BeginRaise() {
        // TODO implement raise
        telemetry.addData("ArmController", "BeginRaise");
    }
}
