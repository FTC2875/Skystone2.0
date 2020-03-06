package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Usage: Implements lift controller.
 *
 *
 *
 * Author: Daniel
 */

public class LiftController {
    private DcMotor lift1;
    private int liftstage = 0;
    private Telemetry telemetry;

    public enum Direction {
        Up,
        Down
    }

    public LiftController(DcMotor lift1, Telemetry telemetry) {
        this.lift1 = lift1;
        this.telemetry = telemetry;

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //if motor.setPower(0), set these motors to brake
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO: Encoder values?
        //TODO: get power coefficient so lift stays at constant height
        double lift1position = lift1.getCurrentPosition();
    }

    public void BeginMovingLift(int position, double power) {
        telemetry.addData("LiftController", "BeginMovingLift: %d, %f", position, power);
        lift1.setTargetPosition(position);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(power);
        while(lift1.isBusy())
        {
            // do nothing
        }

        Stop();

    }

    public void Stop() { lift1.setPower(0); }

    public void setTargetPosition(int position) {
        telemetry.addData("LiftController", "setTargetPosition: %d", position);
        lift1.setTargetPosition(position);
    }

    public void setPower(double power) {
        telemetry.addData("LiftController", "setPower: %f", power);
        lift1.setPower(power);
    }

    public double getCurrentPosition() {
        return lift1.getCurrentPosition();
    }

    public void ZeroBrake() {lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
    public void ZeroCoast() {lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);}

    public boolean IsMoving() {
        return lift1.isBusy();
    }

    public DcMotor GetDcMotor() {
        return lift1;
    }
}
