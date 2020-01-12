package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


/**
 * Adapted from Motor PIDF Tuning Guide by FIRST Global
 *
 * Author: Daniel
 */

@TeleOp(name="TunePID", group = "Concept")
public class TunePID extends LinearOpMode {

    // our DC motor.
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;

    public double P = 0;
    public double I = 0;
    public double D = 0;
    public double F = 0;

    public double posP = 0;

    @Override
    public void runOpMode() {

        motor = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_front");

        waitForStart();


        while(opModeIsActive()) {

            if (gamepad1.a) motor.setPower(1);
            else motor.setPower(0);

        currentVelocity = motor.getVelocity();

        if (currentVelocity > maxVelocity) maxVelocity = currentVelocity;


        F = 32767 / maxVelocity;
        P = 0.1*F;
        I = 0.1*P;
        D = 0;

        posP = 5;

            telemetry.addData("current velocity: ", currentVelocity);
            telemetry.addData("max velocity:", maxVelocity);
            telemetry.addData("F", F);
            telemetry.update();
        }
    }
}
