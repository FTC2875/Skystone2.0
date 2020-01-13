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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.CameraController;
import org.firstinspires.ftc.teamcode.robots.drivetrain.DrivetrainController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="SkystoneBlockLoader")
public class SkystoneBlockLoader extends OpMode {

    private CameraController cameraContoller;
    private DrivetrainController drivetrainController;

    // Count of processed blocks
    private int blockCount;

    private enum RobotStates
    {
        Initialization,
        LookingForBlock,
        ApproachingBlock,
        GrabBlock,
        UnloadBlock,
        CycleComplete,
        Done
    }

    RobotStates robotState;

    SkystoneBlockLoader() {
        robotState = RobotStates.Initialization;
    }

    @Override
    public void init() {

        telemetry.addData("Robot: ", "init0");
        blockCount = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera openCvCamera;

        telemetry.addData("Robot: ", "init1");
        // TODO: Update to use the desired camera
        boolean usePhoneCamera = true;
        if (usePhoneCamera) {
            openCvCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }
        else {
            openCvCamera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        }

        telemetry.addData("Robot: ", "init2");
        cameraContoller = new CameraController(openCvCamera, 640, 480);

        telemetry.addData("Robot: ", "init3");

        // Define motors
        drivetrainController = new DrivetrainController(
                hardwareMap.get(DcMotor.class, "left_front"),
                hardwareMap.get(DcMotor.class, "right_front"),
                hardwareMap.get(DcMotor.class, "left_back"),
                hardwareMap.get(DcMotor.class, "right_back"));

        telemetry.addData("Robot: ", "init4");
    }

    public void init_loop() {
        telemetry.addData("Robot: ", "init_loop");
    }

    public void start() {
        telemetry.addData("Robot: ", "start");
    }

    @Override
    public void loop() {

        telemetry.addData("Robot: ", "loop");

        switch (robotState) {
            case Initialization: {
                // TODO: do any additional initialization, raise grabber?

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

            case GrabBlock: {
                // Waiting until block is grabbed
                // TODO: wait until the block is grabbed

                UnloadBlock();
                break;
            }

            case UnloadBlock: {
                // Waiting until the block is unloaded
                break;
            }

            case CycleComplete: {

                // Processed a block
                blockCount++;
                telemetry.addData("Robot: ", "CycleComplete ", blockCount);
                LookForBlock();
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

        drivetrainController.BeginScan(50);
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
        if (robotState == RobotStates.GrabBlock) {

            telemetry.addData("Robot: ", "Error: already GrabBlock");
            return;
        }

        robotState = RobotStates.GrabBlock;

        telemetry.addData("Robot: ", "GrabBlock");

        // TODO: grab the block
    }

    private void UnloadBlock() {
        if (robotState == RobotStates.UnloadBlock) {

            telemetry.addData("Robot: ", "Error: already UnloadBlock");
            return;
        }

        robotState = RobotStates.UnloadBlock;

        telemetry.addData("Robot: ", "UnloadBlock");

        // Drive to load the block
        if (drivetrainController.IsMoving()) {
            drivetrainController.Stop();
        }

        drivetrainController.BeginUnload(50);
    }
}
