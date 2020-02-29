package org.firstinspires.ftc.teamcode.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Usage: Implements a controller for the drivetrain.
 *
 *
 *
 * Author: Daniel
 */
public class DrivetrainController {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private int frontLeftPosition = 0;
    private int frontRightPosition = 0;
    private int backLeftPosition = 0;
    private int backRightPosition = 0;
    private Telemetry telemetry;

    public DrivetrainController(DcMotor frontLeft,
                                DcMotor frontRight,
                                DcMotor backLeft,
                                DcMotor backRight,
                                Telemetry telemetry) {

        front_left = frontLeft;
        front_right = frontRight;
        back_left = backLeft;
        back_right = backRight;

        this.telemetry = telemetry;

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();


//        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //optional aggresive braking, no coasting
//        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Stop() {
        telemetry.addData("drivetrain:", "stop");

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

        frontLeftPosition = 0;
        frontRightPosition = 0;
        backLeftPosition = 0;
        backRightPosition = 0;

        resetEncoders();
    }

    public void CrawlForward() {
        telemetry.addData("drivetrain:", "crawling forward");
        front_left.setPower(0.1);
        front_right.setPower(0.1);
        back_left.setPower(0.1);
        back_right.setPower(0.1);
    }

    public void BeginScan(int targetPosition){
        telemetry.addData("drivetrain:", "BeginScan, targetPosition: %d", targetPosition);
        Stop();

        resetEncoders();
        setPositionMode();

        //TODO: check encoder values on new drivetrain, is br still negative?

        frontLeftPosition = targetPosition;
        frontRightPosition = targetPosition;
        backLeftPosition = targetPosition;
        backRightPosition = -targetPosition;
        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(-targetPosition); //back right has reverse polarity

        front_left.setPower(0.2);
        front_right.setPower(0.2);
        back_left.setPower(0.2);
        back_right.setPower(0.2);
    }

    public void BeginApproach(int targetPosition){
        telemetry.addData("drivetrain:", "BeginApproach, targetPosition: %d", targetPosition);
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

        front_left.setPower(0.2); //move left until it sees it
        front_right.setPower(0.2);
        back_left.setPower(-0.2);
        back_right.setPower(-0.2);
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

    public void ScanRight(double powerLevel) {
        telemetry.addData("drivetrain:", "ScanLeft at Power: %f", powerLevel);
        Stop();

        resetEncoders();
        setPositionMode();

        front_left.setPower(-powerLevel); //this moves left
        front_right.setPower(powerLevel);
        back_left.setPower(-powerLevel);
        back_right.setPower(powerLevel);
    }

    public void ScanLeft(double powerLevel) {
        telemetry.addData("drivetrain:", "ScanRight at Power: %f", powerLevel);
        Stop();

        resetEncoders();
        setPositionMode();

        front_left.setPower(powerLevel); //this moves left
        front_right.setPower(-powerLevel);
        back_left.setPower(powerLevel);
        back_right.setPower(-powerLevel);
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
        back_left.setPower(0.2);
        back_right.setPower(0.2);

    }

    public void MoveBackToBlocks(int targetPosition) {
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

        front_left.setPower(0.5);
        front_right.setPower(0.5);
        back_left.setPower(0.5);
        back_right.setPower(0.5);

    }
        public boolean IsMoving() {
        return front_left.isBusy() || front_right.isBusy() || back_left.isBusy() || back_right.isBusy()
                || front_left.getCurrentPosition() != frontLeftPosition
                || front_right.getCurrentPosition() != frontRightPosition
                || back_left.getCurrentPosition() != backLeftPosition
                || back_right.getCurrentPosition() != backRightPosition;
    }

    public void SetPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        telemetry.addData("drivetrain", "setpower: %f, %f, %f, %f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        //TODO format all telemetry with %d
        front_left.setPower(frontLeftPower);
        front_right.setPower(frontRightPower);
        back_left.setPower(backLeftPower);
        back_right.setPower(backRightPower);
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
/*        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //encoder modes for motors
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
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
