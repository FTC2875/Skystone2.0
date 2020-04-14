package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.drivetrain.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LiftController;


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
    private DcMotor lift = null;

    private DrivetrainController drivetrainController;
    private LiftController liftController;

    @Override
    public void init() {

        drivetrainController = new DrivetrainController(
                hardwareMap.get(DcMotor.class, "left_front"),
                hardwareMap.get(DcMotor.class, "right_front"),
                hardwareMap.get(DcMotor.class, "left_back"),
                hardwareMap.get(DcMotor.class, "right_back"),
                telemetry);

        drivetrainController.resetEncoders();

    //    liftController = new LiftController(
    //            hardwareMap.get(DcMotor.class, "lift"),
    //            telemetry);

    }
    @Override
    public void loop() {
        if (gamepad1.dpad_up){

            drivetrainController.SetPower(1, 0, 0, 0);
            telemetry.addLine("frontleft on");
        }
        else if (gamepad1.dpad_right) {
            drivetrainController.SetPower(0, 1, 0, 0);
            telemetry.addLine("frontright on");
        }
        else if (gamepad1.dpad_down) {
            drivetrainController.SetPower(0, 0, 1, 0);
            telemetry.addLine("backleft on");
        }
        else if (gamepad1.dpad_left) {
            drivetrainController.SetPower(0, 0, 0, 1);
            telemetry.addLine("backright on");
        }
        else if (gamepad1.y) {
            drivetrainController.SetPower(1,1,1,1);
            telemetry.addLine("all on");
        }
        else {
            drivetrainController.SetPower(0, 0, 0, 0);
            telemetry.clear();
            telemetry.addLine("all off");
        }

        telemetry.addData("fl:", drivetrainController.FLPos());
        telemetry.addData("bl:", drivetrainController.BLPos());
        telemetry.addData("fr:", drivetrainController.FRPos());
        telemetry.addData("br:", drivetrainController.BRPos());
        //telemetry.addData("lift: ", liftController.getCurrentPosition());
        telemetry.update();
    }

}