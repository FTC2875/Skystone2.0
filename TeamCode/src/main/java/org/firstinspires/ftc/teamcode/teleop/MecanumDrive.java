package org.firstinspires.ftc.teamcode.teleop;

import android.media.MediaPlayer;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.drivetrain.DrivetrainController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.ArmController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.FlipperController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.LiftController;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.File;
import java.lang.Math;

import static java.lang.Math.abs;


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
//TODO: Fix liftstage + autonomous

@TeleOp(name="MecanumDrive", group="Iterative Opmode")
public class MecanumDrive extends OpMode {
    //drive motors
    private DrivetrainController drivetrainController;
    //intake
    private IntakeController intakeController;
    //lift
    private LiftController lift;
    //arm/gripper
    private ArmController armController;
    //foundation movers
    private FlipperController flipperController;

    private boolean flipped = false;
    private double powerFactor = 0.5;
    private double armjointPosition;
    private int slowstate = 1;
    private int liftstate = 0;
    private int liftstage = 0;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    private ExpansionHubEx expansionHub;
    private ExpansionHubEx expansionHub2;

    @Override
    public void init() {
        //drivetrainController = new DrivetrainController(
               frontLeft =  hardwareMap.get(DcMotor.class, "left_front");
               frontRight =  hardwareMap.get(DcMotor.class, "right_front");
               backLeft = hardwareMap.get(DcMotor.class, "left_back");
               backRight =  hardwareMap.get(DcMotor.class, "right_back");

        intakeController = new IntakeController(
                hardwareMap.get(DcMotor.class, "intake_left"),
                hardwareMap.get(DcMotor.class, "intake_right"));

        lift = new LiftController(
                hardwareMap.get(DcMotor.class, "lift"));

        armController = new ArmController(
                hardwareMap.get(Servo.class, "armbase"),
                hardwareMap.get(Servo.class, "armjoint"));

        flipperController = new FlipperController(
                hardwareMap.get(Servo.class, "flipper1"),
                hardwareMap.get(Servo.class, "flipper2"));


        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 6");

        expansionHub.setPhoneChargeEnabled(true);
       playdroid();

    }


    @Override
    public void loop() {

        /// DRIVETRAIN CONTROL ///
        //Strafing and rotation definition
        double strafey  = powerFactor * gamepad1.left_stick_y; //basically forwards and backwards
        double strafex = powerFactor * -gamepad1.left_stick_x; //lateral movement
        double turn  = powerFactor * -gamepad1.right_stick_x; //rotation

        double strafeAngle = Math.atan2(strafey, strafex); //angle of strafe from leftstick x and y
        double strafeMag = 0.7 * Math.sqrt(strafex*strafex + strafey*strafey); //magnitude of strafe (pyth. theorum)
        //0.7 is correction factor, max val with this is 1.414 to scale to 1

        //do more trig to find power
        double negStrafePower =  -Math.sin(strafeAngle-(0.25*Math.PI))*strafeMag;
        double posStrafePower =   Math.sin(strafeAngle+(0.25*Math.PI))*strafeMag;

        //set motor power to calculated values
        double frPower = negStrafePower + turn;
        double blPower = negStrafePower - turn;
        double flPower = posStrafePower + turn;
        double brPower = posStrafePower - turn;

//        if (flPower < -1) flPower = -1;
//        if (flPower > 1) flPower = 1;
//        if (blPower < -1) blPower = -1;
//        if (blPower > 1) blPower = 1;
//
//        if (blPower < -1) blPower = -1;
//        if (blPower > 1) blPower = 1;
//        if (blPower < -1) blPower = -1;
//        if (brPower > 1) brPower = 1;

        //slow mode
        if (gamepad1.right_bumper && slowstate == 0){ powerFactor = 0.5; slowstate = 2;}
        if (!gamepad1.right_bumper && slowstate == 2){ slowstate = 1; }
        if (gamepad1.right_bumper && slowstate == 1){ powerFactor = 1; slowstate = 3; }
        if (!gamepad1.right_bumper && slowstate == 3){ slowstate = 0; }

        //divide by 2 to prevent overflow
        //drivetrainController.SetPower(
        //       flPower/2,
        //        frPower/2,
        //        blPower/2,
        //        brPower/2);

        frontLeft.setPower(flPower  );
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);

        //RGB :D
        expansionHub.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));
        expansionHub2.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));

        double currentdraw = expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) + expansionHub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);


//        if the above doesn't work, use this
//        drivetrainController.SetPower(
//                strafey + turn + strafex,
//                strafey - turn - strafex,
//                strafey + turn - strafex,
//                strafey - turn + strafex);




        /// FLIPPER CONTROL ///
        if(gamepad2.y) flipped = true;
        if (flipped) {
            flipperController.SetPosition(0.8, 0.8);
        }
        if (gamepad2.x) flipped = false;
        if (!flipped){
            flipperController.SetPosition(0, 0);
        }


        /// INTAKE ///
        double leftIntakepower = 0;
        double rightIntakepower = 0;
        if (gamepad1.right_trigger > 0.1) {
            leftIntakepower = -1 * gamepad1.right_trigger;
            rightIntakepower = gamepad1.right_trigger;
        }  else if (gamepad1.left_trigger > 0.1) {
            leftIntakepower = gamepad1.left_trigger;
            rightIntakepower = -1 * gamepad1.left_trigger;
        }

        intakeController.BeginIntake(leftIntakepower, rightIntakepower);



        /// ARM CONTROL ///
        armjointPosition = 0;
        if (gamepad2.dpad_left) armjointPosition = 0.88;
        if (gamepad2.dpad_right) armjointPosition = 0;
        armController.SetPosition(gamepad2.right_stick_y, gamepad2.right_stick_x);


        /// LIFT CONTROL ///
        //if (gamepad2.dpad_up && liftstage < 4 && liftstate == 0) {liftstage++; liftstate = 1; moveLift();}
        //if (!gamepad2.dpad_up && !gamepad2.dpad_down) {liftstate = 0;}
        //if (gamepad2.dpad_down && liftstage > 0 && liftstate == 0) {liftstage--; liftstate = 1; moveLift();}

        lift.setPower(gamepad2.left_stick_y);



        /// DEBUG ///
        telemetry.addData("lift pos: ", lift.getCurrentPosition());
        telemetry.addData("liftstage", liftstage);
        telemetry.addData("arm joint: ", armController.getArmJointPosition());
        telemetry.addData("Drive Power: ", powerFactor);
        telemetry.addData("Slow?", slowstate);
        //telemetry.addData("fl Power: ", drivetrainController.FLPower());
        //telemetry.addData("fl Pos: ", drivetrainController.FLPos());
        telemetry.addData("Total Current Draw:", (int)currentdraw + "mA");
        telemetry.update();
    }

    public void moveLift(){
        switch (liftstage) {
            case (0): { lift.BeginMovingLift(0, 0.15);}
            case(1): { lift.BeginMovingLift(200, 0.15); }
            case(2): { lift.BeginMovingLift(400, 0.15); }
            case(3): {lift.BeginMovingLift(600, 0.15); }
        }
    }


    /// SOUNDS ///
    public void playdroid(){
        MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.droid);
        mediaPlayer.start();
    }
}
