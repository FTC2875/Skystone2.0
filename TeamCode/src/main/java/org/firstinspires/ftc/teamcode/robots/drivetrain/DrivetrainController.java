package org.firstinspires.ftc.teamcode.robots.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Usage: Implements drivetrain controller.
 *
 *
 *
 * Author: Daniel
 */
public class DrivetrainController {

    private DcMotor front_left = null; //declare motors
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    public DrivetrainController(DcMotor frontLeft,
                                DcMotor frontRight,
                                DcMotor backLeft,
                                DcMotor backRight) {

        front_left = frontLeft;
        front_right = frontRight;
        back_left = backLeft;
        back_right = backRight;

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //encoder modes for motors
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void BeginScan(int targetPosition){
        Stop();

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(targetPosition);

        front_left.setPower(0.5);
        front_right.setPower(-0.5);
        back_left.setPower(-0.5);
        back_right.setPower(0.5);
    }

    public void BeginApproach(int targetPosition){
        Stop();

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(targetPosition);

        front_left.setPower(-0.2); //move left until it sees it
        front_right.setPower(-0.2);
        back_left.setPower(-0.2);
        back_right.setPower(-0.2);

        while (front_left.isBusy() && front_right.isBusy() && back_left.isBusy() && back_right.isBusy()) {
            // telemetry.addData("motor position:", front_left.getCurrentPosition());
            // telemetry.update();
        }

    }

    public boolean IsMoving() {
        return (front_left.isBusy() || front_right.isBusy() || back_left.isBusy() || back_right.isBusy());
    }
}
