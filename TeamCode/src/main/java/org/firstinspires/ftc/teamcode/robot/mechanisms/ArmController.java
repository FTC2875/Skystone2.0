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

    public void SetPosition(double gripperPosition, double linkagePosition) {
        telemetry.addData("ArmController", "SetPosition: %f, %f", gripperPosition, linkagePosition);
        gripper.setPosition(gripperPosition);
        linkage.setPosition(linkagePosition);
    }

    public void SetGripperPosition(double gripperposition){
        telemetry.addData("ArmController", "SetGripperPosition: %f", gripperposition);
        gripper.setPosition(gripperposition);
    }

    public void SetLinkagePosition(double linkageposition){
        telemetry.addData("ArmController", "SetLinkagePosition: %f", linkageposition);
        linkage.setPosition(linkageposition);
    }

    public double getGripperPosition() { return gripper.getPosition();
    }
    public double getLinkagePosition() { return linkage.getPosition(); }

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
