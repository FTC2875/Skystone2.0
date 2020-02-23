package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

    public class MockDcMotor implements DcMotor {

    private String name;
    private Telemetry telemetry;
    private int position = 0;
    private int currentPosition = 0;
    private double power = 0.0;
    private Direction direction = Direction.FORWARD;

    public MockDcMotor(String name, Telemetry telemetry) {
        this.name = name;
        this.telemetry = telemetry;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {

    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return ZeroPowerBehavior.UNKNOWN;
    }

    @Override
    public void setPowerFloat() {
    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {
        telemetry.addData(name, "setTargetPosition: %d", position);
        this.position = position;
        this.currentPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return position;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public void setMode(RunMode mode) {
        telemetry.addData(name, "setMode: %s", mode);
    }

    @Override
    public RunMode getMode() {
        return null;
    }

    @Override
    public void setDirection(Direction direction) {
        telemetry.addData(name, "setDirection, %s",
                direction == Direction.FORWARD ? "FORWARD": "BACKWARD");

        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {
        telemetry.addData(name, "setTargetPosition: %f", power);
        this.power = power;
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return name;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
