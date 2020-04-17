package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private Telemetry telemetry;

    public IntakeController(DcMotor intake_left, DcMotor intake_right, Telemetry telemetry) {
        this.intake_left = intake_left;
        this.intake_right = intake_right;
        this.telemetry = telemetry;

        intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetPosition(int leftIntakePosition, int rightIntakePosition) {
        telemetry.addData("IntakeController", "SetPosition: %d, %d", leftIntakePosition, rightIntakePosition);
        intake_left.setTargetPosition(leftIntakePosition);
        intake_right.setTargetPosition(rightIntakePosition);
    }

    public void BeginIntake(double leftIntakePower, double rightIntakePower) {
        telemetry.addData("IntakeController", "BeginIntake: %f, %f", leftIntakePower, rightIntakePower);
        intake_left.setPower(leftIntakePower);
        intake_right.setPower(rightIntakePower);
    }

    public void Stop(){
        intake_left.setPower(0);
        intake_right.setPower(0);
    }

    public boolean IsMoving() {
        return intake_left.isBusy() || intake_right.isBusy();
    }
}
