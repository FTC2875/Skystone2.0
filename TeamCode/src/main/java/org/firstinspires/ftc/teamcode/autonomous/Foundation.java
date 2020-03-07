package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.FlipperController;

@Autonomous(name="Foundation", group="Iterative Opmode")
public class Foundation extends OpMode {


    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    private FlipperController flipperController;

    @Override
    public void init(){
        frontLeft =  hardwareMap.get(DcMotor.class, "left_front");
        frontRight =  hardwareMap.get(DcMotor.class, "right_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backRight =  hardwareMap.get(DcMotor.class, "right_back");

        flipperController = new FlipperController(
                hardwareMap.get(Servo.class, "flipper"), telemetry);
    }
    @Override
    public void loop(){
        long t = System.currentTimeMillis();
        long end = t + 900;
        while(System.currentTimeMillis() <  end)

            frontLeft.setPower(-0.2);
        frontRight.setPower(-0.2);
        backLeft.setPower(-0.2);
        backRight.setPower(-0.2);


            flipperController.SetPosition(0.8);
        }
    }
//}
