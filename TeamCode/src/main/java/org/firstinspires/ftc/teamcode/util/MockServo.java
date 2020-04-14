package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Usage: Mocks Servo.
 *
 *
 *
 * Author: Daniel
 */

public class MockServo implements Servo {
    private String name;
    private Telemetry telemetry;
    private double position = 0.0;
    private Direction direction = Direction.FORWARD;
    private double minScaleRange = 0.0;
    private double maxScaleRange = 0.0;

    public MockServo(String name, Telemetry telemetry) {
        this.name = name;
        this.telemetry = telemetry;
    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
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
    public void setPosition(double position) {
        telemetry.addData(name, "setPosition, %f", position);

        this.position = position;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void scaleRange(double min, double max) {
        telemetry.addData(name, "scaleRange, %f, %f", min, max);

        minScaleRange = min;
        maxScaleRange = max;
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
