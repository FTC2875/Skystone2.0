package org.firstinspires.ftc.teamcode.robot.mechanisms;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drivetrain.DrivetrainController;

/**
 * Usage: Combines controllers for lifting and dropping blocks.
 *
 *
 *
 * Author: Daniel
 */
public class FullAutoHelper extends Thread {
    private DrivetrainController drivetrainController;
    private FlipperController flipperController;
    private ArmController armController;
    private IntakeController intakeController;
    private LiftController liftController;
    private Telemetry telemetry;
    private final Object lock1 = new Object();

    // The robot states
    public enum RunningStates
    {
        Ready,
        Loading,
        Unloading,
    }

    private RunningStates runningState = RunningStates.Ready;

    public FullAutoHelper(
            DrivetrainController drivetrainController,
            FlipperController flipperController,
            ArmController armController,
            IntakeController intakeController,
            LiftController liftController,
            Telemetry telemetry) {
        this.drivetrainController = drivetrainController;
        this.flipperController = flipperController;
        this.armController = armController;
        this.intakeController = intakeController;
        this.liftController = liftController;
        this.telemetry = telemetry;
    }

    public RunningStates GetRunningState() {
        synchronized (lock1) {
            return runningState;
        }
    }

    private void SetRunningState(RunningStates state) {
        synchronized (lock1) {
            telemetry.addData("FullAutoHelper: ", "SetRunningState: %s", state.name());
            runningState = state;
        }
    }

    public void run() {
       // while (isAlive()) {
        while(true) {
            switch (runningState) {
                case Loading:
                    Load();
                    break;

                case Unloading:
                    Unload();
                    break;
            }

            Wait(100);
        }
    }

    public void BeginLoad() {
        // Assumes ready starting position
        // This method block until all operations are executed.
        // Must wait after each operation before starting the next, unless both can run together
        SetRunningState(RunningStates.Loading);
        run();
    }

    private void Load() {
        // Assumes ready starting position
        // This method block until all operations are executed.
        // Must wait after each operation before starting the next, unless both can run together

        telemetry.addData("FullAutoHelper: ", "Loading started");

        // TODO: fix the load sequence

        //TODO move DT at same time
        drivetrainController.BeginApproach(100);

        intakeController.BeginIntake(-1, 1);
        Wait(2000);

        armController.BeginGrip();
        Wait(1000);

        liftController.BeginMovingLift(12, 0.2);
        Wait(5000);

        flipperController.BeginFlip();
        Wait(2000);

        // liftController.BeginMovingLift(12, 0.1, LiftController.Direction.Down);
        // flipperController.BeginFlip();

        telemetry.addData("FullAutoHelper: ", "Loading finished");
        SetRunningState(RunningStates.Ready);
        run();
    }

    public void BeginUnload() {
        SetRunningState(RunningStates.Unloading);
    }

    private void Unload() {
        // Assumes ready starting position
        // This method block until all operations are executed.
        // Must wait after each operation before starting the next, unless both can run together

        telemetry.addData("FullAutoHelper: ", "Unloading started");

        // TODO: fix the unload sequence

        // these operations just begin the motion, therefore chain them outside
        // liftController.BeginMovingLift(12, 0.1, LiftController.Direction.Down);

        // flipperController.BeginFlip();
        // liftController.BeginMovingLift(12, 0.2, LiftController.Direction.Up);


        // flipperController.BeginFlip();

        telemetry.addData("FullAutoHelper: ", "Unloading finished");
        // TODO: reset to starting position

        SetRunningState(RunningStates.Ready);
    }

    public void Wait(int millisec) {
        try {
            Thread.sleep(millisec);
        }
        catch(InterruptedException e) {
            telemetry.addData("FullAutoHelper: ", "Wait interrupted: %s", e.getMessage());
        }
    }
}
