package org.firstinspires.ftc.teamcode.autonomous;

/*
Copyright (c) 2019 OpenFTC Team
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

import android.media.MediaPlayer;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drivetrain.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ArmController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.FullAutoHelper;
import org.firstinspires.ftc.teamcode.robot.mechanisms.FlipperController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LiftController;
import org.firstinspires.ftc.teamcode.test.MockDcMotor;
import org.firstinspires.ftc.teamcode.test.MockServo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="FullAuto")
public class FullAuto extends OpMode {

    private CameraController cameraController;

    private DrivetrainController drivetrainController;
    private FullAutoHelper fullAutoHelper;

    // Count of processed blocks
    private int blockCount = 0;

    // TODO: change testing to false
    private boolean testing = true;

    // The robot states
    private enum RobotStates
    {
        Initialization,
        LookingForBlock,
        ApproachingBlock,
        WaitingToGrabBlock,
        GoingToUnloadBlock,
        WaitingToDropBlock,
        WaitingForBlockToDrop,
        Done
    }

    private RobotStates robotState = RobotStates.Initialization;

    BNO055IMU imu;

    @Override
    public void init() {

        blockCount = 0;
        robotState = RobotStates.Initialization;

        // Create the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera openCvCamera;
        boolean usePhoneCamera = testing;
        if (usePhoneCamera) {
            openCvCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }
        else {
            openCvCamera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        }

        String templateImageFileName = Environment.getExternalStorageDirectory()+ "/skystonetemplate.png";
        cameraController = new CameraController(openCvCamera, 640, 480, OpenCvCameraRotation.UPRIGHT, templateImageFileName);

        // Create all controllers

        if (testing) {
                drivetrainController = new DrivetrainController(
                        new MockDcMotor("left_front", telemetry),
                        new MockDcMotor("right_front", telemetry),
                        new MockDcMotor("left_back", telemetry),
                        new MockDcMotor("right_back", telemetry),
                telemetry);

            fullAutoHelper = new FullAutoHelper(
                    new FlipperController(
                            new MockServo("flipper1", telemetry),
                            new MockServo("flipper2", telemetry),
                            telemetry),
                    new ArmController(
                            new MockServo("armbase", telemetry),
                            new MockServo("armjoint", telemetry),
                            telemetry),
                    new IntakeController(
                            new MockDcMotor("intake_left", telemetry),
                            new MockDcMotor("intake_right", telemetry),
                            telemetry),
                    new LiftController(
                            new MockDcMotor("lift", telemetry),
                            telemetry),
                    telemetry);
        }
        else {
            drivetrainController = new DrivetrainController(
                    hardwareMap.get(DcMotor.class, "left_front"),
                    hardwareMap.get(DcMotor.class, "right_front"),
                    hardwareMap.get(DcMotor.class, "left_back"),
                    hardwareMap.get(DcMotor.class, "right_back"),
                    telemetry);

            fullAutoHelper = new FullAutoHelper(
                    new FlipperController(
                            hardwareMap.get(Servo.class, "flipper1"),
                            hardwareMap.get(Servo.class, "flipper2"),
                            telemetry),
                    new ArmController(
                            hardwareMap.get(Servo.class, "armbase"),
                            hardwareMap.get(Servo.class, "armjoint"),
                            telemetry),
                    new IntakeController(
                            hardwareMap.get(DcMotor.class, "intake_left"),
                            hardwareMap.get(DcMotor.class, "intake_right"),
                            telemetry),
                    new LiftController(
                            hardwareMap.get(DcMotor.class, "lift"),
                            telemetry),
                    telemetry);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;

            imu.initialize(parameters);

        }


        fullAutoHelper.run();
    }

    public void loop() {

        switch (robotState) {
            case Initialization: {
                // TODO: do any additional initialization
                // liftController.BeginMovingLift(12, LiftController.Direction.Down);
                playdroid();
                LookForBlock();
                break;
            }

            case LookingForBlock: {
                ProcessCameraState();
                //if CameraController.States.valueOf() {

               // }
                // TODO: if the robot did not find block before it it reached the end of platform, set robotState to Done
                break;
            }

            case ApproachingBlock: {
                ProcessCameraState();
                break;
            }

            case WaitingToGrabBlock: {
                // Waiting until block is grabbed
                if (fullAutoHelper.GetRunningState() == FullAutoHelper.RunningStates.Ready) {

                    fullAutoHelper.Wait(1000);
                    UnloadBlock();
                }
                break;
            }

            case GoingToUnloadBlock: {
                // Waiting until the block is unloaded
                break;
            }

            case WaitingToDropBlock: {
                DropBlock();
                break;
            }

            case WaitingForBlockToDrop: {
                // Waiting dropping the block
                if (fullAutoHelper.GetRunningState() == FullAutoHelper.RunningStates.Ready) {
                    // Processed a block
                    blockCount++;
                    telemetry.addData("Robot: ", "DroppedBlock %d", blockCount);
                    telemetry.update();

                    // TODO: go again?
                    fullAutoHelper.Wait(5000);

                    LookForBlock();
                }
                break;
            }

            case Done: {
                // TODO: no more blocks, what now?
                telemetry.addData("Robot: ", "Done");
                telemetry.update();
                break;
            }

            default:

                break;
        }
    }

    @Override
    public void stop() {
        robotState = RobotStates.Done;

        // Terminate the background thread
        fullAutoHelper.interrupt();

        try {
            fullAutoHelper.join();
        }
        catch(InterruptedException e) {
            telemetry.addData("fullAutoHelper: ", "thread exited");
            telemetry.update();
        }
    }

    private void ProcessCameraState() {
        switch (cameraController.State) {
            case Undetermined: {
                telemetry.addData("Camera: ", "Object Undetermined");
                telemetry.update();
                break;
            }

            case ObjectFound: {
                telemetry.addData("Camera: ", "ObjectFound");
                telemetry.update();
                ApproachBlock();
                break;
            }

            case ObjectLost: {
                telemetry.addData("Camera: ", "ObjectLost");
                telemetry.update();

                // Stop driving to load the block
                if (drivetrainController.IsMoving()) {

                    telemetry.addData("Robot: ", "Stop");
                    telemetry.update();
                    drivetrainController.Stop();
                }

                GrabBlock();
                break;
                    }
        }
    }

    private void LookForBlock() {
        if (robotState == RobotStates.LookingForBlock) {

            telemetry.addData("Robot: ", "Error: already LookingForBlock");
            telemetry.update();
            return;
        }

        robotState = RobotStates.LookingForBlock;

        telemetry.addData("Robot: ", "LookingForBlock");
        telemetry.update();

        if (drivetrainController.IsMoving()) {
            drivetrainController.Stop();
        }

        drivetrainController.BeginScan(950);
    }

    private void ApproachBlock() {
        if (robotState == RobotStates.ApproachingBlock) {

            telemetry.addData("Robot: ", "Error: already ApproachingBlock");
            telemetry.update();
            return;
        }

        robotState = RobotStates.ApproachingBlock;

        telemetry.addData("Robot: ", "ApproachingBlock");
        telemetry.update();

        // Drive to load the block
        if (drivetrainController.IsMoving()) {
            drivetrainController.Stop();
        }

        drivetrainController.BeginApproach(50);
    }

    private void GrabBlock() {
        if (robotState == RobotStates.WaitingToGrabBlock) {

            telemetry.addData("Robot: ", "Error: already WaitingToGrabBlock");
            telemetry.update();
            return;
        }

        robotState = RobotStates.WaitingToGrabBlock;

        telemetry.addData("Robot: ", "WaitingToGrabBlock");
        telemetry.update();

        fullAutoHelper.BeginLoad();
    }

    private void UnloadBlock() {
        if (robotState == RobotStates.GoingToUnloadBlock) {

            telemetry.addData("Robot: ", "Error: already GoingToUnloadBlock");
            telemetry.update();
            return;
        }

        robotState = RobotStates.GoingToUnloadBlock;

        telemetry.addData("Robot: ", "GoingToUnloadBlock");
        telemetry.update();

        // Drive to load the block
        if (drivetrainController.IsMoving()) {
            drivetrainController.Stop();
        }

        drivetrainController.BeginUnload(50);

        robotState = RobotStates.WaitingToDropBlock;
    }

    private void DropBlock() {
        if (robotState == RobotStates.WaitingToDropBlock && !drivetrainController.IsMoving()) {
            telemetry.addData("Robot: ", "WaitingToDropBlock");
            telemetry.update();

            fullAutoHelper.BeginUnload();

            robotState = RobotStates.WaitingForBlockToDrop;
        }

    }
    private void playdroid(){
        MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.droid);
        mediaPlayer.start();
    }
}
