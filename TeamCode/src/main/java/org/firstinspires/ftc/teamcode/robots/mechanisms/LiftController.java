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

    public enum LiftTarget{
        Stop,
        Up,
        Down
    }

    public LiftController(DcMotor lift) {
        this.lift = lift;

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //if motor.setPower(0), set these motors to brake
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void MoveLift(int position, LiftTarget liftTarget) {
        double liftposition = lift.getCurrentPosition();

        switch (liftstage) {
            case (0): {
                lift.setTargetPosition(position);
            }   case(1): {

            }
        }

        //lift controls, @power 0 - the motor brakes
        switch (liftTarget){
            case Up:
            lift.setPower(-0.2);
            break;

            case Down:
            lift.setPower(0.1);
            break;

            case Stop:
            lift.setPower(0);
            break;
        }

    }
}
