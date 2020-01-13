package org.firstinspires.ftc.teamcode.robots.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Usage: Implements intake controller.
 *
 *
 *
 * Author: Daniel
 */

public class IntakeController {

    private DcMotor intake_left;
    private DcMotor intake_right;

    public IntakeController(DcMotor intake_left, DcMotor intake_right) {
        this.intake_left = intake_left;
        this.intake_right = intake_right;

        intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetPosition(int leftIntakePosition, int rightIntakePosition) {
        intake_left.setTargetPosition(leftIntakePosition);;
        intake_right.setTargetPosition(rightIntakePosition);
    }

    public void ActuateIntake(double leftIntakePower, double rightIntakePower) {
        intake_left.setPower(leftIntakePower);
        intake_right.setPower(rightIntakePower);
    }
}
