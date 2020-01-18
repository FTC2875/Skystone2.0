package org.firstinspires.ftc.teamcode.autonomous;/*

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
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import org.opencv.core.Scalar;

import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.opencv.core.Core.minMaxLoc;
import static org.opencv.imgcodecs.Imgcodecs.imread;

@Autonomous
public class SkystoneAuto extends LinearOpMode {
    OpenCvCamera webcam;
    SkystonePipeline SkystonePipeline;

    private DcMotor front_left = null; //declare motors
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    List<MatOfPoint> Contours = new ArrayList<>(); //define global vars
    public int numtaps = 1;

    public int templateMatchMethod = 0;

    public double templatethresh = 0.45;

    public double templateScore = 0;
    Mat templ;

    private Random rdm = new Random(12345);

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        SkystonePipeline = new SkystonePipeline();
        webcam.setPipeline(SkystonePipeline);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


        String path = Environment.getExternalStorageDirectory() + "/skystonetemplate.png";

        //URL res = getClass().getClassLoader().getResource("template0.png");
        templ = imread(path);


        front_left = hardwareMap.get(DcMotor.class, "left_front"); //define motors
        front_right = hardwareMap.get(DcMotor.class, "right_front");
        back_left = hardwareMap.get(DcMotor.class, "left_back");
        back_right = hardwareMap.get(DcMotor.class, "right_back");


        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //encoder modes for motors
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        int autostate = 0;
    }


    public class SkystonePipeline extends OpenCvPipeline {

        Mat templout = new Mat();
        Mat inputrgb = new Mat();
        Mat input2 = new Mat();
        int autostate = 0;


        public Mat processFrame(Mat input) {
            waitForStart();
            while (opModeIsActive()) {

                input2 = input;

                Imgproc.cvtColor(input, inputrgb, Imgproc.COLOR_RGBA2RGB);


                numtaps = 2;
                switch (numtaps) {
                    case 1: {
                        templateMatchMethod = Imgproc.TM_CCORR_NORMED;
                        break;
                    }
                    case 2: {
                        templateMatchMethod = Imgproc.TM_CCOEFF_NORMED;
                        break;
                    }
                    case 3: {
                        templateMatchMethod = Imgproc.TM_SQDIFF_NORMED;
                        break;
                    }

                }

                Imgproc.matchTemplate(inputrgb, templ, templout, templateMatchMethod);
                Core.MinMaxLocResult mmr = minMaxLoc(templout);

                Point matchLoc;
                double maxVal;
                if (templateMatchMethod == Imgproc.TM_SQDIFF || templateMatchMethod == Imgproc.TM_SQDIFF_NORMED) {
                    matchLoc = mmr.minLoc;
                    maxVal = mmr.minVal;
                } else {
                    matchLoc = mmr.maxLoc;
                    maxVal = mmr.maxVal;
                }

                templateScore = maxVal;

                double xmid = matchLoc.x + templ.cols()/2; //divide by 2 for midpoint of width/heigioht of box + its location
                double ymid = matchLoc.y + templ.rows()/2;
                Point center = new Point(xmid, ymid);

                if (templateMatchMethod == 5 & templateScore >= templatethresh) {
                    Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templ.cols(), matchLoc.y + templ.rows()), new Scalar(0, 255, 0), 5);
                    Imgproc.drawMarker(input,center, new Scalar(240,130,240));
                } else if (templateScore < templatethresh) {
                    Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templ.cols(), matchLoc.y + templ.rows()), new Scalar(255, 0, 0), 2);
                }


                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                if (autostate == 0) {

                    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    int skystonethresh = 50; //24" to blocks is 1125ticks


                    front_left.setTargetPosition(skystonethresh);
                    back_left.setTargetPosition(skystonethresh);
                    front_right.setTargetPosition(skystonethresh);
                    back_right.setTargetPosition(skystonethresh);


                    front_left.setPower(0.5);
                    front_right.setPower(-0.5);
                    back_left.setPower(-0.5);
                    back_right.setPower(0.5);

                    while (front_left.isBusy() && front_right.isBusy() && back_left.isBusy() && back_right.isBusy()) {
                        telemetry.addData("motor position:", front_left.getCurrentPosition());
                        telemetry.update();
                    }

                    front_left.setPower(0);
                    front_right.setPower(0);
                    back_left.setPower(0);
                    back_right.setPower(0);

                    autostate = 1;
                }

                if (templateScore <= templatethresh && autostate == 1){ //doesn't see skystone yet
                    front_left.setPower(-0.2); //move left until it sees it
                    front_right.setPower(-0.2);
                    back_left.setPower(-0.2);
                    back_right.setPower(-0.2);
                } else if (templateScore <= templatethresh && autostate == 1){ //it sees it
                    if(center.x > 0){ //center with the block
                        //move left till it gets close to x = 0
                    } else if(center.x < 0){
                        //move right till it gets close to 0

                    }
                }



                telemetry.update();
            }

            return input; //render to viewport

        }

    }


// if (templateMatchMethod == 0 || templateMatchMethod == 2) { // normalize(templout, templout, 0, 1, NORM_MINMAX, -1); // }


}

//
//

