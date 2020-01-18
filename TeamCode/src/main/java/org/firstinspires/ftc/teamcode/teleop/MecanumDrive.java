package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.drivetrain.DrivetrainController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.ArmController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.FlipperController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.LiftController;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.File;
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
    private int liftstage = 0;
    private double powerFactor;

    private ExpansionHubEx expansionHub;
    private ExpansionHubEx expansionHub2;

    @Override
    public void init() {
        drivetrainController = new DrivetrainController(
                hardwareMap.get(DcMotor.class, "left_front"),
                hardwareMap.get(DcMotor.class, "right_front"),
                hardwareMap.get(DcMotor.class, "left_back"),
                hardwareMap.get(DcMotor.class, "right_back"));

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
       // playdroid();

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
        double posStrafePower = Math.sin(strafeAngle+(0.25*Math.PI))*strafeMag;

        //set motor power to calculated values
        double frPower = negStrafePower + turn;
        double blPower = negStrafePower - turn;
        double flPower = posStrafePower + turn;
        double brPower = posStrafePower - turn;
        double max = Math.max(Math.abs(blPower),Math.abs(flPower));

        //slow mode
        if (gamepad1.right_bumper) powerFactor = 0.4;
        else  powerFactor = 1.0;

        //divide by 2 to prevent overflow
        drivetrainController.SetPower(
                powerFactor * flPower/2,
                powerFactor * frPower/2,
                powerFactor * blPower/2,
                powerFactor * brPower/2);

        //RGB :D
        expansionHub.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));
        expansionHub2.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));


//        if the above doesn't work, use this
//        drivetrainController.SetPower(
//                strafey + turn + strafex,
//                strafey - turn - strafex,
//                strafey + turn - strafex,
//                strafey - turn + strafex);

        if(gamepad2.y) flipped = true;
        if (flipped) {
            flipperController.SetPosition(0.8, 0.8);
        }
        if (gamepad2.x) flipped = false;
        if (!flipped){
            flipperController.SetPosition(0, 0);
        }

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

        //arm control
        double armjointPosition = 0;
        if (gamepad2.dpad_left) armjointPosition = 0.88;
        if (gamepad2.dpad_right) armjointPosition = 0;
        armController.SetPosition(
                0.7 + (-0.6 * gamepad2.right_stick_y),
                armjointPosition);

        double liftposition = lift.getCurrentPosition();

        //if (gamepad2.dpad_up == true && liftstage <= 4) liftstage++;
        //if (gamepad2.dpad_down == true && liftstage > 0 ) liftstage--;

        switch (liftstage) {
            case (0): {
        lift.setTargetPosition(0);
        }   case(1): {
            lift.setTargetPosition(154);
            }
        }

        //lift controls, @power 0 - the motor brakes
        if(gamepad2.dpad_up){
            lift.setPower(0.2); }
        else if(gamepad2.dpad_down){
                lift.setPower(-0.1);}
        else {
            lift.Stop();
        }


        telemetry.addData("lift pos: ", liftposition);
        telemetry.addData("liftstage", liftstage);
        telemetry.addData("arm joint: ", armController.getArmJointPosition());
        telemetry.addData("Power: ", powerFactor);
        telemetry.update();
    }

    public void playdroid(){
        String soundPath = "/FIRST/blocks/sounds";
        File droidFile = new File(soundPath + "/droid.wav");
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, droidFile);
    }
}
