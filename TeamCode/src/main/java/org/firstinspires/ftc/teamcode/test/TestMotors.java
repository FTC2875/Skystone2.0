package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.drivetrain.DrivetrainController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.LiftController;

import java.util.concurrent.TimeUnit;


/**
 * Test motors and their encoders, or check a config file
 *
 *
 *  Author: Daniel
 */
@TeleOp
public class TestMotors extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_right = null;
    private DcMotor back_left = null;

    private DrivetrainController drivetrainController;

    @Override
    public void init() {

        drivetrainController = new DrivetrainController(
                hardwareMap.get(DcMotor.class, "left_front"),
                hardwareMap.get(DcMotor.class, "right_front"),
                hardwareMap.get(DcMotor.class, "left_back"),
                hardwareMap.get(DcMotor.class, "right_back"));
    }
    @Override
    public void loop() {
        if (gamepad1.dpad_up) drivetrainController.SetPower(1, 0, 0, 0);
        if (gamepad1.dpad_right) drivetrainController.SetPower(0, 1, 0, 0);
        if (gamepad1.dpad_down) drivetrainController.SetPower(0, 0, 1, 0);
        if (gamepad1.dpad_left) drivetrainController.SetPower(0, 0, 0, 1);

        telemetry.addData("fl:", drivetrainController.FLPos());
        telemetry.addData("bl:", drivetrainController.BLPos());
        telemetry.addData("fr:", drivetrainController.FRPos());
        telemetry.addData("br:", drivetrainController.BRPos());
        telemetry.update();
    }

}