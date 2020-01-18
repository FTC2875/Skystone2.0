package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

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

    @Override
    public void init() {

        front_left = hardwareMap.get(DcMotor.class, "left_front");
        front_right = hardwareMap.get(DcMotor.class, "right_front");
        back_right = hardwareMap.get(DcMotor.class, "right_back");
        back_left = hardwareMap.get(DcMotor.class, "left_back");

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void loop() {
        if (gamepad1.dpad_up) {front_left.setPower(1);}
        if (gamepad1.dpad_right) {front_right.setPower(1);}
        if (gamepad1.dpad_down) {back_right.setPower(1);}
        if (gamepad1.dpad_left) {back_left.setPower(1);}

        telemetry.addData("fl:", front_left.getCurrentPosition());
        telemetry.addData("bl:", back_left.getCurrentPosition());
        telemetry.addData("fr:", front_right.getCurrentPosition());
        telemetry.addData("br:", back_right.getCurrentPosition());
        telemetry.update();
    }

}