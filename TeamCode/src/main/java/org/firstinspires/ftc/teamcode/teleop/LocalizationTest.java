package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ArmController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.FlipperController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LiftController;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.List;

import static java.lang.Math.abs;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private boolean flipped = false;
    private double powerFactor = 1;
    private double armjointPosition;
    private int slowstate = 1;
    private double liftinit;
    private double lift2init;

    private ExpansionHubEx expansionHub;
    private ExpansionHubEx expansionHub2;

    IntakeController intakeController;
    LiftController lift;
    ArmController armController;
    FlipperController flipperController;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        intakeController = new IntakeController(
                hardwareMap.get(DcMotor.class, "intake1"),
                hardwareMap.get(DcMotor.class, "intake2"),
                telemetry);

        lift = new LiftController(
                hardwareMap.get(DcMotor.class, "lift"),
                hardwareMap.get(DcMotor.class, "lift2"),
                new PIDCoefficients(5,0,1),
                telemetry);

        armController = new ArmController(
                hardwareMap.get(Servo.class, "gripper"),
                hardwareMap.get(Servo.class, "linkage"),
                telemetry);

        flipperController = new FlipperController(
                hardwareMap.get(Servo.class, "flipper"),
                telemetry);

        expansionHub.setPhoneChargeEnabled(true);
        armController.SetGripperPosition(0.25);

        liftinit = lift.getLift1Pos();
        lift2init = lift.getLift2Pos();
        lift.setPID(5, 0, 1);

        waitForStart();

        while (!isStopRequested()) {
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            Pose2d vel;
            if (abs(baseVel.getX()) + abs(baseVel.getY()) + abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * abs(baseVel.getX())
                    + VY_WEIGHT * abs(baseVel.getY())
                    + OMEGA_WEIGHT * abs(baseVel.getHeading());
                vel = new Pose2d(
                    VX_WEIGHT * baseVel.getX(),
                    VY_WEIGHT * baseVel.getY(),
                    OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);

            drive.update();

            List<Double> velocities = drive.getWheelVelocities();
            double total = 0;

            for (int i = 0; i < velocities.size(); i++ ){
                total += abs(velocities.get(i));
            }

            double avgvel = total / velocities.size();

            if (gamepad1.right_bumper && slowstate == 0){ powerFactor = 1; slowstate = 2;}
            if (!gamepad1.right_bumper && slowstate == 2){ slowstate = 1; }
            if (gamepad1.right_bumper && slowstate == 1){ powerFactor = 0.3; slowstate = 3; }
            if (!gamepad1.right_bumper && slowstate == 3){ slowstate = 0; }


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
            if (gamepad2.dpad_up) lift.setPowerPID(0.5);
            else if (gamepad2.dpad_down && gamepad2.right_trigger < 0.05) lift.setPowerPID(-0.15);
            else if (gamepad2.right_trigger > 0.05 && !gamepad2.dpad_down) lift.setPowerPID(-0.08);
            //TODO: LIFT STATE MACHINE FOR MAX HEIGHT
            if (!gamepad2.dpad_up  && !gamepad2.dpad_down && !gamepad2.left_bumper && gamepad2.left_trigger < 0.05) lift.setPowerPID(0.0009); //resist Fg pushing down on lift
            //}.
            //else lift.setPower(0.05);

            if(gamepad2.left_bumper && !gamepad2.dpad_down) {
                //lift.ZeroCoast();
                lift.setPower(0.4);
            }


            expansionHub.setLedColor((int)(baseVel.getX()*6), (int)(baseVel.getY()*6), (int)(baseVel.getHeading()*6));
            expansionHub2.setLedColor((int)(baseVel.getX()*6), (int)(baseVel.getY()*6), (int)(baseVel.getHeading()*6));
            double currentdrawAMPS = expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + expansionHub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("velocity", avgvel);
            telemetry.addData("Drawing", Math.round(currentdrawAMPS * 100d) / 100d + "A");
            telemetry.update();
        }
    }
}