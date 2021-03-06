package org.firstinspires.ftc.teamcode.robot.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.util.Range.clip;
import org.firstinspires.ftc.teamcode.robot.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.MockDcMotor;

/**
 * Usage: Implements a controller for the drivetrain.
 *
 *
 *
 * Author: Daniel
 */
public class DrivetrainController {

    private MockDcMotor front_left;
    private MockDcMotor front_right;
    private MockDcMotor back_left;
    private MockDcMotor back_right;

    private int frontLeftPosition = 0;
    private int frontRightPosition = 0;
    private int backLeftPosition = 0;
    private int backRightPosition = 0;
    private Telemetry telemetry;

    private SampleMecanumDrive drive;
    public boolean ScanningRight = false;
    public boolean ScanningLeft = false;

    public DrivetrainController(DcMotor frontLeft,
                                DcMotor frontRight,
                                DcMotor backLeft,
                                DcMotor backRight,
                                HardwareMap hardwareMap,
                                Telemetry telemetry) {

        front_left = new MockDcMotor("front_left", telemetry);
        front_right = new MockDcMotor("front_right", telemetry);
        back_left = new MockDcMotor("back_left", telemetry);
        back_right = new MockDcMotor("back_right", telemetry);

        this.telemetry = telemetry;

        drive = new SampleMecanumDrive(hardwareMap);

//        front_left.setDirection(DcMotor.Direction.REVERSE); //TODO: uncomment
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();


//        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //optional aggresive braking, no coasting
//        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Stop() {
        telemetry.addData("drivetrain:", "stop");
        drive.setMotorPowers(0,0,0,0);

        frontLeftPosition = 0;
        frontRightPosition = 0;
        backLeftPosition = 0;
        backRightPosition = 0;

        resetEncoders();
    }

    public void CrawlForward() {
        telemetry.addData("drivetrain:", "crawling forward");
        front_left.setPower(-0.3);
        front_right.setPower(0.3);
        back_left.setPower(-0.3);
        back_right.setPower(0.3);
    }

    public void BeginScan(int targetPosition){
        telemetry.addData("drivetrain:", "BeginScan, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();

        //TODO: check encoder values on new drivetrain, is br still negative?

        frontLeftPosition = -targetPosition;
        frontRightPosition = targetPosition;
        backLeftPosition = -targetPosition;
        backRightPosition = targetPosition;
        front_left.setTargetPosition(-targetPosition);
        back_left.setTargetPosition(-targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(targetPosition); //back right has reverse polarity

        front_left.setPower(-0.2);
        front_right.setPower(0.2);
        back_left.setPower(-0.2);
        back_right.setPower(0.2);
    }

    public void BeginApproach(double power){
        telemetry.addData("drivetrain:", "BeginApproach, targetPosition: %d");

            Stop();
            Pose2d baseVel = new Pose2d(power, 0, 0);
            drive.setDrivePower(baseVel);

        drive.update();
    }

    public void BeginUnload(int targetPosition){
        telemetry.addData("drivetrain:", "BeginUnload, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();

        frontLeftPosition = targetPosition;
        frontRightPosition = targetPosition;
        backLeftPosition = targetPosition;
        backRightPosition = -targetPosition;

        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(-targetPosition);

        front_left.setPower(-0.2);
        front_right.setPower(0.2);
        back_left.setPower(0.2);
        back_right.setPower(-0.2);
    }

    public void BeginScanRight(double powerLevel) {
        telemetry.addData("drivetrain:", "ScanRight at Power: %f", powerLevel);

        if (!ScanningRight) {
            Stop();
            ScanningRight = true;
            Pose2d baseVel = new Pose2d(0, -powerLevel, 0);
            drive.setDrivePower(baseVel);
        }

        drive.update();
    }

    public void BeginScanLeft(double powerLevel) {
        telemetry.addData("drivetrain:", "ScanLeft at Power: %f", powerLevel);

        if (!ScanningLeft) {
            Stop();
            ScanningLeft = true;
            Pose2d baseVel = new Pose2d(0, powerLevel, 0);
            drive.setDrivePower(baseVel);
        }

        drive.update();

    }

    public void BeginTurnRight(int targetPosition) {
        telemetry.addData("drivetrain:", "BeginTurnRight, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();

        frontLeftPosition = targetPosition;
        frontRightPosition = targetPosition;
        backLeftPosition = targetPosition;
        backRightPosition = -targetPosition;

        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(-targetPosition);

        front_left.setPower(-0.2);
        front_right.setPower(-0.2);
        back_left.setPower(0.2);
        back_right.setPower(0.2);
    }

    public void BeginTurnLeft(int targetPosition) {
        telemetry.addData("drivetrain:", "BeginTurnLeft, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();

        frontLeftPosition = targetPosition;
        frontRightPosition = targetPosition;
        backLeftPosition = targetPosition;
        backRightPosition = -targetPosition;

        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(-targetPosition);

        front_left.setPower(0.2);
        front_right.setPower(0.2);
        back_left.setPower(-0.2);
        back_right.setPower(-0.2);
    }

    public void MoveToFoundation(int targetPosition) {
        telemetry.addData("drivetrain:", "MoveToFoundation, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();

        frontLeftPosition = targetPosition;
        frontRightPosition = targetPosition;
        backLeftPosition = targetPosition;
        backRightPosition = -targetPosition;

        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(-targetPosition);

        front_left.setPower(-0.5);
        front_right.setPower(-0.5);
        back_left.setPower(-0.5);
        back_right.setPower(-0.5);

    }

    public void ApproachFoundation(int targetPosition) {
        telemetry.addData("drivetrain:", "MoveToFoundation, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();

        MoveForward(targetPosition, 0.2);

    }

    public void MoveBackToBlocks(int targetPosition) {
        telemetry.addData("drivetrain:", "MoveToFoundation, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();
        MoveLeft(targetPosition, 0.2);

    }
        public boolean IsMoving() {
        return front_left.isBusy() || front_right.isBusy() || back_left.isBusy() || back_right.isBusy()
                || front_left.getCurrentPosition() != frontLeftPosition
                || front_right.getCurrentPosition() != frontRightPosition
                || back_left.getCurrentPosition() != backLeftPosition
                || back_right.getCurrentPosition() != backRightPosition;
    }

    public void SetPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {

        drive.setMotorPowers(frontLeftPower,backLeftPower,backRightPower,frontRightPower);

        telemetry.addData("drivetrain", "setpower: %f, %f, %f, %f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        front_left.setPower(frontLeftPower);
        front_right.setPower(frontRightPower);
        back_left.setPower(backLeftPower);
        back_right.setPower(backRightPower); //change min/max to doubles?
    }

    public void setPositionMode(){
/*        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
    }

    public void resetEncoders() {
        telemetry.addData("drivetrain", "reset encoders");

        // try without encoders
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //encoder modes for motors
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void MoveLeft(int target, double power){
        frontLeftPosition = target;
        frontRightPosition = target;
        backLeftPosition = target;
        backRightPosition = -target;

        front_left.setTargetPosition(target);
        back_left.setTargetPosition(target);
        front_right.setTargetPosition(target);
        back_right.setTargetPosition(-target);

        front_left.setPower(power); //TODO: verify that this works, check back right reversed encoder
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }
    public void MoveRight(int target, double power){
        frontLeftPosition = target;
        frontRightPosition = target;
        backLeftPosition = target;
        backRightPosition = -target;

        front_left.setTargetPosition(target);
        back_left.setTargetPosition(target);
        front_right.setTargetPosition(target);
        back_right.setTargetPosition(-target);

        front_left.setPower(power); //TODO: verify that this works, check back right reversed encoder
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }
    public void MoveForward(int target, double power){
        frontLeftPosition = -target;
        frontRightPosition = target;
        backLeftPosition = -target;
        backRightPosition = target;

        front_left.setTargetPosition(-target);
        back_left.setTargetPosition(-target);
        front_right.setTargetPosition(target);
        back_right.setTargetPosition(target);

        front_left.setPower(-power);
        front_right.setPower(-power);
        back_left.setPower(power);
        back_right.setPower(power);
    }

    public void update() {
        new Thread(() -> {
            drive.update();
        }).start();

    }

    public double FLPower(){ return front_left.getPower(); }
    public double BLPower(){ return back_left.getPower(); }
    public double FRPower(){ return front_right.getPower(); }
    public double BRPower(){ return back_right.getPower(); }
    public double FLPos() { return front_left.getCurrentPosition(); }
    public double BLPos() { return back_left.getCurrentPosition(); }
    public double FRPos() { return front_right.getCurrentPosition(); }
    public double BRPos() { return -back_right.getCurrentPosition(); }
}