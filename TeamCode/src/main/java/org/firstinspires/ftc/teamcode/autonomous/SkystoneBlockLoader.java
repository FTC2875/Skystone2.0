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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.CameraController;
import org.firstinspires.ftc.teamcode.robots.drivetrain.DrivetrainController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.ArmController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.BlockHelperController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.FlipperController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.robots.mechanisms.LiftController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="SkystoneBlockLoader")
public class SkystoneBlockLoader extends OpMode {

    private CameraController cameraContoller;

    private DrivetrainController drivetrainController;
    private BlockHelperController blockHelperController;

    // Count of processed blocks
    private int blockCount = 0;

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

    @Override
    public void init() {

        blockCount = 0;
        robotState = RobotStates.Initialization;

        // Create the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera openCvCamera;
        // TODO: Update to use the desired camera
        boolean usePhoneCamera = false;
        if (usePhoneCamera) {
            openCvCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }
        else {
            openCvCamera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        }

        String templateImageFileName = Environment.getExternalStorageDirectory()+ "/skystonetemplate.png";
        cameraContoller = new CameraController(openCvCamera, 640, 480, OpenCvCameraRotation.UPRIGHT, templateImageFileName);

        // Create all controllers

        drivetrainController = new DrivetrainController(
                hardwareMap.get(DcMotor.class, "left_front"),
                hardwareMap.get(DcMotor.class, "right_front"),
                hardwareMap.get(DcMotor.class, "left_back"),
                hardwareMap.get(DcMotor.class, "right_back"));

        blockHelperController = new BlockHelperController(
                new FlipperController(
                        hardwareMap.get(Servo.class, "flipper1"),
                        hardwareMap.get(Servo.class, "flipper2")),
                new ArmController(
                        hardwareMap.get(Servo.class, "armbase"),
                        hardwareMap.get(Servo.class, "armjoint")),
                new IntakeController(
                        hardwareMap.get(DcMotor.class, "intake_left"),
                        hardwareMap.get(DcMotor.class, "intake_right")),
                new LiftController(
                        hardwareMap.get(DcMotor.class, "lift")),
                telemetry
        );

        blockHelperController.start();
    }

    @Override
    public void loop() {

        switch (robotState) {
            case Initialization: {
                // TODO: do any additional initialization, raise arm, move flpper, etc?
                // liftController.BeginMovingLift(12, LiftController.Direction.Down);

                LookForBlock();
                break;
            }

            case LookingForBlock: {
                ProcessCameraState();

                // TODO: if the robot did not find block before it it reached the end of platform, set robotState to Done
                break;
            }

            case ApproachingBlock: {
                ProcessCameraState();
                break;
            }

            case WaitingToGrabBlock: {
                // Waiting until block is grabbed
                if (blockHelperController.GetRunningState() == BlockHelperController.RunningStates.Ready) {

                    blockHelperController.Wait(1000);
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
                if (blockHelperController.GetRunningState() == BlockHelperController.RunningStates.Ready) {
                    // Processed a block
                    blockCount++;
                    telemetry.addData("Robot: ", "DroppedBlock ", blockCount);

                    // TODO: go again?
                    blockHelperController.Wait(5000);

                    LookForBlock();
                }
                break;
            }

            case Done: {
                // TODO: no more blocks, what now?
                telemetry.addData("Robot: ", "Done");
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
        blockHelperController.interrupt();

        try {
            blockHelperController.join();
        }
        catch(InterruptedException e) {
            telemetry.addData("blockHelperController: ", "thread exited");
        }
    }

    private void ProcessCameraState() {
        switch (cameraContoller.State) {
            case Undetermined: {
                break;
            }

            case ObjectFound: {
                telemetry.addData("Camera: ", "ObjectFound");
                ApproachBlock();
                break;
            }

            case ObjectLost: {
                telemetry.addData("Camera: ", "ObjectLost");

                // Stop driving to load the block
                if (drivetrainController.IsMoving()) {

                    telemetry.addData("Robot: ", "Stop");
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
            return;
        }

        robotState = RobotStates.LookingForBlock;

        telemetry.addData("Robot: ", "LookingForBlock");

        // Scan for a block or move to unload it
        if (drivetrainController.IsMoving()) {
            drivetrainController.Stop();
        }

        drivetrainController.BeginScan(500);
    }

    private void ApproachBlock() {
        if (robotState == RobotStates.ApproachingBlock) {

            telemetry.addData("Robot: ", "Error: already ApproachingBlock");
            return;
        }

        robotState = RobotStates.ApproachingBlock;

        telemetry.addData("Robot: ", "ApproachingBlock");

        // Drive to load the block
        if (drivetrainController.IsMoving()) {
            drivetrainController.Stop();
        }

        drivetrainController.BeginApproach(50);
    }

    private void GrabBlock() {
        if (robotState == RobotStates.WaitingToGrabBlock) {

            telemetry.addData("Robot: ", "Error: already WaitingToGrabBlock");
            return;
        }

        robotState = RobotStates.WaitingToGrabBlock;

        telemetry.addData("Robot: ", "WaitingToGrabBlock");

        blockHelperController.BeginLoad();
    }

    private void UnloadBlock() {
        if (robotState == RobotStates.GoingToUnloadBlock) {

            telemetry.addData("Robot: ", "Error: already GoingToUnloadBlock");
            return;
        }

        robotState = RobotStates.GoingToUnloadBlock;

        telemetry.addData("Robot: ", "GoingToUnloadBlock");

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

            blockHelperController.BeginUnload();

            robotState = RobotStates.WaitingForBlockToDrop;
        }
    }
}
