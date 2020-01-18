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

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

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

//        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    }

    public void BeginUnload(int targetPosition){
        Stop();

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(targetPosition);

        front_left.setPower(-0.5);
        front_right.setPower(0.5);
        back_left.setPower(0.5);
        back_right.setPower(-0.5);
    }

    public boolean IsMoving() {
        return front_left.isBusy() || front_right.isBusy() || back_left.isBusy() || back_right.isBusy();
    }

    public void SetPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        front_left.setPower(frontLeftPower);
        front_right.setPower(frontRightPower);
        back_left.setPower(backLeftPower);
        back_right.setPower(backRightPower);
    }
    public double FLPower(){ return front_left.getPower(); }
    public double BLPower(){ return back_left.getPower(); }
    public double FRPower(){ return front_right.getPower(); }
    public double BRPower(){ return back_right.getPower(); }
    public double FLPos() { return front_left.getCurrentPosition(); }
    public double BLPos() { return back_left.getCurrentPosition(); }
    public double FRPos() { return front_right.getCurrentPosition(); }
    public double BRPos() { return back_right.getCurrentPosition(); }
}
