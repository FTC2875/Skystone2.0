package org.firstinspires.ftc.teamcode.robots.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Usage: Implements lift controller.
 *
 *
 *
 * Author: Daniel
 */

public class LiftController {
    private DcMotor lift;
    private int liftstage = 0;

    public enum Direction {
        Up,
        Down
    }

    public LiftController(DcMotor lift) {
        this.lift = lift;

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //if motor.setPower(0), set these motors to brake
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double liftposition = lift.getCurrentPosition();
    }

    public void BeginMovingLift(int position, double power) {

                lift.setTargetPosition(position);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(power);
                while(lift.isBusy()){}
                Stop();
    }

    public void Stop() { lift.setPower(0); }

    public void setTargetPosition(int position) {
        lift.setTargetPosition(position);
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    public double getCurrentPosition() {
        return lift.getCurrentPosition();
    }

    public boolean IsMoving() {
        return lift.isBusy();
    }

    public DcMotor GetDcMotor() {
        return lift;
    }
}
