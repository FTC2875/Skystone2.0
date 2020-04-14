package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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
    private DcMotor lift2;
    private int liftstage = 0;
    private Telemetry telemetry;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    PIDFController controller;

    public enum Direction {
        Up,
        Down
    }

    public LiftController(DcMotor lift1, DcMotor lift2, PIDCoefficients coeffs, Telemetry telemetry) {
        this.lift1 = lift1;
        this.lift2 = lift2;
        this.telemetry = telemetry;


        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //if motor.setPower(0), set these motors to brake
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO: Encoder values?
        //TODO: get power coefficient so lift stays at constant height
        double lift1position = lift1.getCurrentPosition();
        double lift2position = lift2.getCurrentPosition();

        controller = new PIDFController(coeffs);

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

    public void Stop() { lift1.setPower(0); lift2.setPower(0);}


    public void setPID(double kP, double kI, double kD){
        // specify coefficients/gains
        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

        controller = new PIDFController(coeffs);
    }

    public void setTargetPosition(int position) {
        telemetry.addData("LiftController", "setTargetPosition: %d", position);
        lift1.setTargetPosition(position);
    }

    public void setPower(double power) {
        telemetry.addData("LiftController", "setPower: %f", power);
        lift1.setPower(power);
        lift2.setPower(-power);
    }

    public void setPowerPID(double power) {
        telemetry.addData("LiftController", "setPowerPID: %f", power);
        lift1.setPower(power);
        lift2.setPower(-power);
        // specify the setpoint
        controller.setTargetPosition(lift1.getCurrentPosition());

// in each iteration of the control loop
// measure the position or output variable
// apply the correction to the input variable
        double correction = controller.update(lift2.getCurrentPosition());
        try{
            lift2.setPower(correction);
        } catch(Exception e) {
            lift2.setTargetPosition((int)correction);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public double getLift1Pos() {
        return lift1.getCurrentPosition();
    }
    public double getLift2Pos() {
        return lift2.getCurrentPosition();
    }

    public void lift1ZeroBrake() {lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
    public void lift1ZeroCoast() {lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);}

    public boolean IsMoving() {
        return lift1.isBusy() || lift2.isBusy();
    }

    public DcMotor getLift1() { return lift1; }
    public DcMotor getLift2() { return lift2; }
}

