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
        BeginLoading,
        BeginUnloading,
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
            telemetry.addData("FullAutoHelper", "SetRunningState: %s", state.name());
            runningState = state;
        }
    }

    public void Start() {
        if (!this.isAlive()) {
            telemetry.addData("FullAutoHelper", "start thread");
            this.start();
            this.setName("FullAutoHelper");
        }
    }

    public void Stop() {
        if (this.isAlive()) {
            telemetry.addData("FullAutoHelper", "stop thread");
            this.interrupt();
        }
    }

    public void run() {
        if (!isAlive()) {
            telemetry.addData("FullAutoHelper", "thread is not alive");
            return;
        }

        while (isAlive()) {
            switch (runningState) {
                case BeginLoading:
                    Load();
                    break;

                case BeginUnloading:
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
        SetRunningState(RunningStates.BeginLoading);
    }

    private void Load() {
        // Assumes ready starting position
        // This method block until all operations are executed.
        // Must wait after each operation before starting the next, unless both can run together

        telemetry.addData("FullAutoHelper", "Loading started");
        SetRunningState(RunningStates.Loading);

        // TODO: fix the load sequence

        //TODO move DT at same time
        //drivetrainController.BeginApproach(0.3);



        intakeController.BeginIntake(1, -1);
        Wait(2000);

        //drivetrainController.Stop();
        armController.BeginGrip();
        Wait(1000);

        intakeController.Stop();


        // liftController.BeginMovingLift(12, 0.1, LiftController.Direction.Down);
        // flipperController.BeginFlip();

        telemetry.addData("FullAutoHelper", "Loading finished");
        SetRunningState(RunningStates.Ready);
    }

    public void BeginUnload() {
        SetRunningState(RunningStates.BeginUnloading);
    }

    private void Unload() {
        // Assumes ready starting position
        // This method block until all operations are executed.
        // Must wait after each operation before starting the next, unless both can run together

        telemetry.addData("FullAutoHelper", "Unloading started");
        SetRunningState(RunningStates.Unloading);

        int liftTime = 2400;

        liftController.setPowerUp(0.4);
        Wait(liftTime);
        liftController.setPowerUp(0.0009);
        Wait(500);
        armController.extendLinkage();
        Wait(750);

        liftController.setPowerDown(-0.4);
        Wait(liftTime);

        liftController.setPowerDown(0);
        Wait(1000);
        armController.BeginRelease();
        Wait(1500);

        liftController.setPowerUp(0.4);
        Wait(liftTime);
        liftController.setPowerUp(0.0009);
        Wait(500);
        armController.retractLinkage();
        Wait(750);
        liftController.setPowerDown(-0.4);
        Wait(liftTime);
        liftController.setPower(0);

        telemetry.addData("FullAutoHelper", "Unloading finished");
        // TODO: reset to starting position

        SetRunningState(RunningStates.Ready);
    }

    public void Wait(int millisec) {
        try {
            Thread.sleep(millisec);
        }
        catch(InterruptedException e) {
            telemetry.addData("FullAutoHelper", "Wait interrupted: %s", e.getMessage());
        }
    }
}
