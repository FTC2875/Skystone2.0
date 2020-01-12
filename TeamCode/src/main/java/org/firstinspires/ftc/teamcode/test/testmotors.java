package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;


/**
 *
 *
 *
 * @author Brandon Gong
 */
@TeleOp(name="testmotors", group="Iterative Opmode")
public class testmotors extends OpMode {

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


        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");

    }
    @Override
    public void loop() {
        if (gamepad1.dpad_up) front_left.setPower(1);
        if (gamepad1.dpad_right) front_right.setPower(1);
        if (gamepad1.dpad_down) back_right.setPower(1);
        if (gamepad1.dpad_left) back_left.setPower(1);
    }

}