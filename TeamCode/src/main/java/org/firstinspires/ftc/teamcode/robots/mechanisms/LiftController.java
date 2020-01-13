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

        //if motor.setPower(0), set these motors to brake
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void MoveLift(int position, double power, Direction direction) {
        double liftposition = lift.getCurrentPosition();

        switch (liftstage) {
            case (0): {
                lift.setTargetPosition(position);
            }   case(1): {

            }
        }

        //lift controls, @power 0 - the motor brakes
        switch (direction){
            case Up:
            lift.setPower(-power);
            break;

            case Down:
            lift.setPower(power);
            break;
        }
    }

    public void Stop() {
        lift.setPower(0);
    }

    public void setTargetPosition(int position) {
        lift.setTargetPosition(position);
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
