package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;

import java.lang.Math;


/**
 * Usage: Implements PID Control with 2D strafing and rotation, and set
 * lift positions on a Mecanum-based chassis.
 *
 *
 * Motors (7) :  Drivetrain (*4), Intake(*2), Lift
 * Servos (4) : ArmJoint, ArmBase, Flipper(*2)
 *
 * Sensors
 *
 * Author: Daniel
 */

@TeleOp(name="MecanumDrive", group="Iterative Opmode")
public class MecanumDrive extends OpMode {


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

    boolean flipped = false;
    int liftstage = 0;
    int armpos = 0;

    ExpansionHubEx expansionHub;
    ExpansionHubEx expansionHub2;

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


        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 6");



            //configure motors
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //if motor.setPower(0), set these motors to brake
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void loop() {

        //Strafing and rotation definition
        double strafey  = gamepad1.left_stick_y; //basically forwards and backwards
        double strafex = -gamepad1.left_stick_x; //lateral movement
        double turn  = -gamepad1.right_stick_x; //rotation

        double strafeAngle = Math.atan2(strafey, strafex); //angle of strafe from leftstick x and y
        double strafeMag = 0.7 * Math.sqrt(strafex*strafex + strafey*strafey); //magnitude of strafe (pyth. theorum)
        //0.7 is correction factor, max val with this is 1.414 to scale to 1

        //do more trig to find power
        double negStrafePower = -Math.sin(strafeAngle-(0.25*Math.PI))*strafeMag;
        double posStrafeMath = Math.sin(strafeAngle+(0.25*Math.PI))*strafeMag;

        //set motor power to calculated values
        double frPower = negStrafePower + turn;
        double blPower = negStrafePower - turn;
        double flPower = posStrafeMath + turn;
        double brPower = posStrafeMath - turn;
        double max = Math.max(Math.abs(blPower),Math.abs(flPower));

        //slow mode
        double powerFactor = 1;
        if (gamepad1.right_bumper) powerFactor = 0.4;
        else powerFactor = 1;

        //divide by 2 to prevent overflow
        front_right.setPower(powerFactor * frPower/2);
        back_left.setPower(powerFactor * blPower/2);
        front_left.setPower(powerFactor * flPower/2);
        back_right.setPower(powerFactor * brPower/2);

        //RGB :D
        expansionHub.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));
        expansionHub2.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));



//        if the above doesn't work, use this
//        front_left.setPower(strafey + turn + strafex);
//        back_left.setPower(strafey + turn - strafex);
//        front_right.setPower(strafey - turn - strafex);
//        back_right.setPower(strafey - turn + strafex);


        //foundation flippers
        if(gamepad2.y) flipped = true;
        if (flipped) {
            flipper1.setPosition(0.8);
            flipper2.setPosition(0.8);
        }
        if (gamepad2.x) flipped = false;
        if (!flipped){
            flipper1.setPosition(0);
            flipper2.setPosition(0);
        }

        double intakepower = 0;
        if(gamepad1.right_trigger > 0.1) {
            intakepower = gamepad1.right_trigger;
            intake_left.setPower(intakepower*-1);
            intake_right.setPower(intakepower);
        }  else if(gamepad1.left_trigger > 0.1) {
            intakepower = gamepad1.left_trigger;
            intake_left.setPower(intakepower);
            intake_right.setPower(intakepower*-1);
        } else {
            intakepower = 0;
            intake_left.setPower(intakepower);
            intake_right.setPower(intakepower);
        }


        //arm control
        armbase.setPosition(0.7 + (-0.6 * gamepad2.right_stick_y)); //default open position is 0.6
        if (gamepad2.dpad_left == true) armjoint.setPosition(0.88);
        if (gamepad2.dpad_right == true) armjoint.setPosition(0);

        double liftposition = lift.getCurrentPosition();


        //if (gamepad2.dpad_up == true && liftstage <= 4) liftstage = liftstage + 1;
        //if (gamepad2.dpad_down == true && liftstage > 0 ) liftstage = liftstage - 1;


        switch (liftstage) {
            case (0): {
        lift.setTargetPosition(12);
        }   case(1): {

            }
        }



        //lift controls, @power 0 - the motor brakes
        if(gamepad2.dpad_up == true){
            lift.setPower(-0.2); }
        else if(gamepad2.dpad_down == true){
            lift.setPower(0.1); }
        else {
            lift.setPower(0);
        }


        telemetry.addData("lift pos: ", liftposition);
        telemetry.addData("liftstage", liftstage);
        telemetry.addData("arm joint: ", armjoint.getPosition());
        telemetry.addData("Power: ", powerFactor);
        telemetry.update();
    }
}
