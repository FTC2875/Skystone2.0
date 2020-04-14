package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class runTo extends OpMode {

    public static int distance;

    //method, looking, loading, etc looking for block
        //declare and initialize drive motors
        DcMotor front_right;
        DcMotor front_left;
        DcMotor back_left;
        DcMotor back_right;

        @Override
        public void init() {

            front_left = hardwareMap.get(DcMotor.class, "left_front");
            front_right = hardwareMap.get(DcMotor.class, "right_front");
            back_left = hardwareMap.get(DcMotor.class, "left_back");
            back_right = hardwareMap.get(DcMotor.class, "right_back");

            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

        @Override
        public void loop() {
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                distance = 770; //24" to blocks is 1125ticks
            //383.6 per revolution


              //  front_left.setTargetPosition(skystonethresh);
//                back_left.setTargetPosition(skystonethresh);
//                front_right.setTargetPosition(skystonethresh);
//                back_right.setTargetPosition(skystonethresh);


               // front_left.setPower(0.5);
//                front_right.setPower(-0.5);
//                back_left.setPower(-0.5);
//                back_right.setPower(0.5);

                //while (front_left.isBusy() /*&& front_right.isBusy() && back_left.isBusy() && back_right.isBusy()*/) {
                    telemetry.addData("motor position:", front_left.getCurrentPosition());
                    telemetry.update();
                //}

               // front_left.setPower(0);
//                front_right.setPower(0);
//                back_left.setPower(0);
//                back_right.setPower(0);
               // stop();

        }
}
