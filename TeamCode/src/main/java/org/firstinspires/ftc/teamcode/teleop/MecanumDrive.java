package org.firstinspires.ftc.teamcode.teleop;

import android.media.MediaPlayer;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robot.drivetrain.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ArmController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.FlipperController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LiftController;
import org.openftc.revextensions2.ExpansionHubEx;

import static com.qualcomm.robotcore.util.Range.clip;


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
    private double powerFactor = 0.3;
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

    double liftposition = 0;
    double lift2position = 0;
    double liftinit;
    double lift2init;

    @Override
    public void init() {
        drivetrainController = new DrivetrainController(
               frontLeft =  hardwareMap.get(DcMotor.class, "left_front"),
               frontRight =  hardwareMap.get(DcMotor.class, "right_front"),
               backLeft = hardwareMap.get(DcMotor.class, "left_back"),
               backRight =  hardwareMap.get(DcMotor.class, "right_back"),
                hardwareMap, telemetry);

        intakeController = new IntakeController(
                hardwareMap.get(DcMotor.class, "intake1"),
                hardwareMap.get(DcMotor.class, "intake2"),
                telemetry);

        lift = new LiftController(
                hardwareMap.get(DcMotor.class, "lift"),
                hardwareMap.get(DcMotor.class, "lift2"),
                new PIDCoefficients(5,0,2),
                telemetry);

        armController = new ArmController(
                hardwareMap.get(Servo.class, "gripper"),
                hardwareMap.get(Servo.class, "linkage"),
                telemetry);

        flipperController = new FlipperController(
                hardwareMap.get(Servo.class, "flipper"),
                telemetry);


        //IMU = new IMUController(hardwareMap.get(BNO055IMU.class, "imu";)

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        expansionHub.setPhoneChargeEnabled(true);
        armController.SetGripperPosition(0.25);

        liftinit = lift.getLift1Pos();
        lift2init = lift.getLift2Pos();
        //armController.SetLinkagePosition(0.7);

        playdroid();
    }


    @Override
    public void loop() {

        /// DRIVETRAIN CONTROL ///
        //Strafing and rotation definition

        liftposition = lift.getLift1Pos() - liftinit;

        //slow mode
        if (gamepad1.right_bumper && slowstate == 0){ powerFactor = 0.3; slowstate = 2;}
        if (!gamepad1.right_bumper && slowstate == 2){ slowstate = 1; }
        if (gamepad1.right_bumper && slowstate == 1){ powerFactor = 1.4; slowstate = 3; }
        if (!gamepad1.right_bumper && slowstate == 3){ slowstate = 0; }

        double strafey  = clip(powerFactor * gamepad1.left_stick_y, -1, 1); //basically forwards and backwards
        double strafex = clip(powerFactor * -gamepad1.left_stick_x, -1, 1); //lateral movement
        double turn  = clip(powerFactor * -gamepad1.right_stick_x, -1, 1); //rotation

        double strafeAngle = Math.atan2(strafey, strafex); //angle of strafe from leftstick x and y
        double strafeMag = Math.sqrt(strafex*strafex + strafey*strafey); //magnitude of strafe (pyth. theorum)

        //do more trig to find power
        double negStrafePower =  -Math.sin(strafeAngle-(0.25*Math.PI))*strafeMag;
        double posStrafePower =   Math.sin(strafeAngle+(0.25*Math.PI))*strafeMag;

        //set motor power to calculated values
        double frPower = negStrafePower + turn;
        double blPower = negStrafePower - turn;
        double flPower = posStrafePower + turn;
        double brPower = posStrafePower - turn;

        //overflow prevention in controller
        drivetrainController.SetPower(flPower, frPower, blPower, brPower);

        //frontLeft.setPower(flPower);
        //frontRight.setPower(frPower);
        //backLeft.setPower(blPower);
        //backRight.setPower(brPower);

        //RGB :D
        expansionHub.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));
        expansionHub2.setLedColor((int)(strafex*255), (int)(strafey*255), (int)(turn*255));

        //double currentdraw = expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) + expansionHub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);
        double currentdrawAMPS = expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + expansionHub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);

//        if the above doesn't work, use this
//        drivetrainController.SetPower(
//                strafey + turn + strafex,
//                strafey - turn - strafex,
//                strafey + turn - strafex,
//                strafey - turn + strafex);




        /// FLIPPER CONTROL ///
        if(gamepad2.y) flipped = true;
        if (flipped) {
            flipperController.SetPosition(0);
        }
        if (gamepad2.x) flipped = false;
        if (!flipped){
            flipperController.SetPosition(1);
        }


        /// INTAKE ///
        double leftIntakepower = 0;
        double rightIntakepower = 0;
        if (gamepad1.right_trigger > 0.1) {
            leftIntakepower = gamepad1.right_trigger;
            rightIntakepower = -gamepad1.right_trigger;
        }  else if (gamepad1.left_trigger > 0.1) {
            leftIntakepower = -gamepad1.left_trigger;
            rightIntakepower = gamepad1.left_trigger;
        }

        intakeController.BeginIntake(leftIntakepower, rightIntakepower);



        /// ARM CONTROL ///
        //armjointPosition = 0;
        //TODO LINKAGE FUNCTION
        if (gamepad2.a) armController.SetGripperPosition(1);
        if (gamepad2.b) armController.SetGripperPosition(0.25);

        if (gamepad2.dpad_right) armController.SetLinkagePosition(0.17);
        if (gamepad2.dpad_left) armController.SetLinkagePosition(0.63);


        /// LIFT CONTROL ///
        //if (gamepad2.dpad_up && liftstage < 4 && liftstate == 0) {liftstage++; liftstate = 1; moveLift();}
        //if (!gamepad2.dpad_up && !gamepad2.dpad_down) {liftstate = 0;}
        //if (gamepad2.dpad_down && liftwstage > 0 && liftstate == 0) {liftstage--; liftstate = 1; moveLift();}
        //if (liftposition >= -1200) {double liftinit = lift.getCurrentPosition();
            if (gamepad2.dpad_up) lift.setPower(-0.35);
            else if (gamepad2.dpad_down && gamepad2.right_trigger < 0.05) lift.setPower(0.15);
            else if (gamepad2.right_trigger > 0.05 && !gamepad2.dpad_down) lift.setPower(0.08);
            //TODO LIFT STATE MACHINE FOR MAX HEIGHT
            if (!gamepad2.dpad_up  && !gamepad2.dpad_down && !gamepad2.left_bumper && gamepad2.left_trigger < 0.05) lift.setPower(-0.0009); //resist Fg pushing down on lift
        //}.
        //else lift.setPower(0.05);

        if(gamepad2.left_bumper && !gamepad2.dpad_down) {
            //lift.ZeroCoast();
            lift.setPower(0.4);
        }


        /// DEBUG ///
        telemetry.addData("lift pos: ", lift.getLift1Pos());
        telemetry.addData("lift2 pos:", lift.getLift2Pos());
        telemetry.addData("Drive Power: ", powerFactor);
        telemetry.addData("Total Current Draw:", Math.round(currentdrawAMPS * 100d) / 100d + "A");
        telemetry.addData("Gripper Position: ", armController.getGripperPosition());
        telemetry.addData("Linkage Position: ", armController.getLinkagePosition());
        telemetry.update();
    }

//    public void moveLift(){
//        switch (liftstage) {
//            case (0): { lift.BeginMovingLift(0, 0.15);}
//            case(1): { lift.BeginMovingLift(200, 0.15); }
//            case(2): { lift.BeginMovingLift(400, 0.15); }
//            case(3): {lift.BeginMovingLift(600, 0.15); }
//        }
//    }




    /// SOUNDS ///
    public void playdroid(){
        MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.droid);
        mediaPlayer.start();
    }
}
