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

import java.lang.Math.*;

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

    private double ticks = 767.2; //# encoder ticks per revolution
    private double wheelC = 100 * 0.0393701 * Math.PI; //100mm wheel diameter to circumference in inches
    private double distanceToBlock = 30; //inches to skystone accounting for robot width

    private int ticksToDetect = (int)(ticks * ((distanceToBlock - 15) / wheelC)); //~1860, 15 inches for CV to safely detect
    private int ticksToBlock = (int)((ticks * (distanceToBlock / wheelC)) - ticksToDetect); //remaining ticks to stones

    // Count of processed blocks
    private int blockCount = 0;

    // TODO: change testing to false
    private boolean testing = true;

    // The robot states
    private enum RobotStates
    {
        Initialization,
        LookingForBlock,
        MoveToBlock,
        ApproachingBlock,
        WaitingToGrabBlock,
        DrivingToFoundation,
        GoingToUnloadBlock,
        WaitingToDropBlock,
        WaitingForBlockToDrop,
        MoveBackToBlock,
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
                drivetrainController,
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
                    drivetrainController,
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

        fullAutoHelper.Start();

        Wait(1000); // gives camera some time to initialize properly
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

            case MoveToBlock: {
                AlignWithBlock();
                //TODO: move left and right for alignment with the block, compare center.x to mid point of viewframe
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

            case DrivingToFoundation: {
                // TODO: move to foundation side
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

                    // TODO: go again?
                    fullAutoHelper.Wait(5000);

                    LookForBlock();
                }
                break;
            }

            case MoveBackToBlock: {
                //TODO move back to other side for second loop
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

        fullAutoHelper.Stop();
    }

    private void ProcessCameraState() {
        switch (cameraController.State) {
            case Undetermined: {
                telemetry.addData("Camera: ", "Object Undetermined");
                break;
            }

            case ObjectFound: {
                telemetry.addLine("Camera: ObjectFound at ");
                telemetry.addData("x: ", cameraController.center.x);
                telemetry.addData("y: ", cameraController.center.y);
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

        if (drivetrainController.IsMoving()) {
            drivetrainController.Stop();
        }

        drivetrainController.BeginScan(ticksToDetect);
    }

    private void AlignWithBlock(){
        if (robotState == RobotStates.MoveToBlock) {
            telemetry.addData("Robot: ", "Error: already MovingToBlock");
            return;
        } //TODO: Align with the block by moving left and right and processing camerastate, add PID control with center.x value to get to 240



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

        drivetrainController.BeginApproach(ticksToBlock);
    }


    private void GrabBlock() {
        if (robotState == RobotStates.WaitingToGrabBlock) {

            telemetry.addData("Robot: ", "Error: already WaitingToGrabBlock");
            return;
        }

        robotState = RobotStates.WaitingToGrabBlock;

        telemetry.addData("Robot: ", "WaitingToGrabBlock");

        fullAutoHelper.BeginLoad();
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

            fullAutoHelper.BeginUnload();

            robotState = RobotStates.WaitingForBlockToDrop;
        }

    }
    public void Wait(int millisec) {
        try {
            Thread.sleep(millisec);
        }
        catch(InterruptedException e) {
            telemetry.addData("FullAutoHelper: ", "Wait interrupted: %s", e.getMessage());
        }
    }
    private void playdroid(){
        MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.droid);
        mediaPlayer.start();
    }
}
