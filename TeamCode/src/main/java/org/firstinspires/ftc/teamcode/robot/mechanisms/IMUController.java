package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;


public class IMUController {

    BNO055IMU imu;

    public IMUController(){
        this.imu = imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }
}
