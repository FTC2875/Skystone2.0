package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;


/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
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
    private DcMotor motor10 = null;
    private DcMotor motor11 = null;
    private DcMotor motor12 = null;
    private DcMotor motor13 = null;
    private DcMotor motor20 = null;
    private DcMotor motor21 = null;
    private DcMotor motor22 = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        motor10 = hardwareMap.get(DcMotor.class, "motor10");
        motor11 = hardwareMap.get(DcMotor.class, "motor11");
        motor12 = hardwareMap.get(DcMotor.class, "motor12");
        motor13 = hardwareMap.get(DcMotor.class, "motor13");
        motor20 = hardwareMap.get(DcMotor.class, "motor20");
        motor21 = hardwareMap.get(DcMotor.class, "motor21");
        motor22 = hardwareMap.get(DcMotor.class, "motor22");

    }
    @Override
    public void loop() {
        motor20.setPower(0.25);
        telemetry.addData("Motor 20 running", 1);
        telemetry.update();
        telemetry.clear();
        wait(3000);
        motor10.setPower(0);
        motor11.setPower(0.25);
        telemetry.addData("Motor 11 running", 1);
        telemetry.update();
        telemetry.clear();
        wait(3000);
        motor11.setPower(0);
        motor12.setPower(0.25);
        telemetry.addData("Motor 12 running", 1);
        telemetry.update();
        telemetry.clear();
        wait(3000);
        motor12.setPower(0);
        motor13.setPower(0.25);
        telemetry.addData("Motor 13 running", 1);
        telemetry.update();
        telemetry.clear();
        wait(3000);
        motor13.setPower(0);
        motor20.setPower(0.25);
        telemetry.addData("Motor 20 running", 1);
        telemetry.update();
        telemetry.clear();
        wait(3000);
        motor20.setPower(0);
        motor21.setPower(0.25);
        telemetry.addData("Motor 21 running", 1);
        telemetry.update();
        telemetry.clear();
        wait(3000);
        motor21.setPower(0);
        motor22.setPower(0.25);
        telemetry.addData("Motor 22 running", 1);
        telemetry.update();
        telemetry.clear();
        wait(3000);
        motor22.setPower(0);
        stop();

    }
    public static void wait(int ms){
        try
        {
            Thread.sleep(ms);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
}