package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous public class BridgeParkAuto extends LinearOpMode {

    private DcMotor front_left = null; //declare motors
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    private DcMotor intake_left = null;
    private DcMotor intake_right = null;

    @Override
    public void runOpMode() {

        front_left = hardwareMap.get(DcMotor.class, "left_front"); //define motors
        front_right = hardwareMap.get(DcMotor.class, "right_front");
        back_left = hardwareMap.get(DcMotor.class, "left_back");
        back_right = hardwareMap.get(DcMotor.class, "right_back");

        intake_left = hardwareMap.get(DcMotor.class, "intake_left");
        intake_right = hardwareMap.get(DcMotor.class, "intake_right");

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //encoder modes for motors
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();

            int skystonethresh = 1150; //24" to blocks is 1125ticks
            int intakethresh = 1200;


            sleep(10000);

            long t = System.currentTimeMillis();
            long end = t + 900;
            while(System.currentTimeMillis() <  end) {
                front_left.setPower(0.2);
                front_right.setPower(-0.2);
                back_left.setPower(-0.2);
                back_right.setPower(0.2);
            }


            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            intake_left.setPower(0);
            intake_right.setPower(0);

            sleep(30000);
        }

    }
}


//
//
