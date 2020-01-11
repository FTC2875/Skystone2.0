package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;
import java.util.*;


/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="MecanumDrive", group="Iterative Opmode")
public class MecanumDrive extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

        //declare and initialize drive motors

    DcMotor front_right;
    DcMotor front_left;
    DcMotor back_left;
    DcMotor back_right;
        //intake
    DcMotor intake_left;
    DcMotor intake_right;
        //lift
    DcMotor lift;
        //arm/gripper
    Servo armbase;
    Servo armjoint; //upper near rotation point
        //foundation movers
    Servo flipper1;
    Servo flipper2;

        //flipper boolean
    private boolean flippeddown = false;

    @Override
    public void init() {

        front_left = hardwareMap.get(DcMotor.class, "left_front");
        front_right = hardwareMap.get(DcMotor.class, "right_front");
        back_left = hardwareMap.get(DcMotor.class, "left_back");
        back_right = hardwareMap.get(DcMotor.class, "right_back");

        intake_left = hardwareMap.get(DcMotor.class, "intake_left");
        intake_right = hardwareMap.get(DcMotor.class, "intake_right");

        lift = hardwareMap.get(DcMotor.class, "lift");
        armbase = hardwareMap.get(Servo.class, "armbase");
        armjoint  = hardwareMap.get(Servo.class, "armjoint");

        flipper1 = hardwareMap.get(Servo.class, "flipper1");
        flipper2 = hardwareMap.get(Servo.class, "flipper2");





            //configure motors
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //need to code in lift positions
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        //Strafing and rotation definition
        double strafey  = gamepad1.left_stick_y;
        double strafex = gamepad1.left_stick_x;
        double turn  = gamepad1.right_stick_x;

        double strafeAngle = Math.atan2(strafey, strafex); //angle of strafe
        double strafeMag = 0.3 * Math.sqrt(strafex*strafex + strafey*strafey); //magnitude of strafe (pyth. theorum)
        //0.7 is correction factor, max val is 1.414, .7 is p=1

        //set motor power to calculated values

        double frPower = Math.sin(strafeAngle-(0.25*Math.PI))*strafeMag + turn;
        double blPower = -Math.sin(strafeAngle-(0.25*Math.PI))*strafeMag + turn;
        double flPower = -Math.sin(strafeAngle+(0.25*Math.PI))*strafeMag + turn; //+ turn
        double brPower = Math.sin(strafeAngle+(0.25*Math.PI))*strafeMag + turn;
        double max = Math.max(Math.abs(blPower),Math.abs(flPower));
        double powerFactor = -0.6;

        front_right.setPower(powerFactor * frPower/max);
        back_left.setPower(powerFactor * blPower/max);

        //right side is reversed because wheels are mounted in other direction
        front_left.setPower(powerFactor * flPower/max);
        back_right.setPower(powerFactor * brPower/max);


        //foundation flippers
        flippeddown = gamepad2.right_bumper ? !flippeddown : flippeddown;

        if(gamepad2.y) {
            flipper1.setPosition(0.8);
            flipper2.setPosition(0.8);
        }
        if (gamepad2.x) {
            flipper1.setPosition(0);
            flipper2.setPosition(0);
        }

        double intakepower = 0;
        if(gamepad1.right_trigger > 0.1) {
            intakepower = gamepad1.right_trigger;
            intake_left.setPower(intakepower*-1);
            intake_right.setPower(intakepower);
        }
        if(gamepad1.left_trigger > 0.1) {
            intakepower = gamepad1.left_trigger;
            intake_left.setPower(intakepower);
            intake_right.setPower(intakepower*-1);
        }else {
            intakepower = 0;
            intake_left.setPower(0);
            intake_right.setPower(0);
        }

        if(gamepad2.right_trigger > 0.05) {
            intakepower = 0;
            intake_right.setPower(intakepower);
            intake_left.setPower(intakepower);
        }


        armjoint.setPosition(-1 * gamepad2.left_stick_x);
        armbase.setPosition(0.59 + (-1 * gamepad2.right_stick_y)); //default position is 0.59: open

        double liftposition = lift.getCurrentPosition();

        int liftstage = 0;
        if (gamepad2.dpad_up == true && liftstage <= 4) liftstage = liftstage + 1;
        if (gamepad2.dpad_down == true && liftstage > 0 ) liftstage = liftstage - 1;

        /*
        switch (liftstage) {
        case (0){
        lift.setTargetPosition();
        }
        }
         */


        if(gamepad2.dpad_up == true){
            lift.setPower(0.5); }
        else if(gamepad2.dpad_down == true){
            lift.setPower(-0.5); }
        else {
            lift.setPower(0);
        }


        telemetry.addData("lift pos: ", liftposition);
        telemetry.addData("lift power: ", lift.getPower());
        telemetry.addData("liftstage", liftstage);
        telemetry.addData("arm joint: ", armjoint.getPosition());
        telemetry.addData("sv",Math.sin(strafeAngle-(0.25*Math.PI))*strafeMag + turn);



        telemetry.update();
    }
}
