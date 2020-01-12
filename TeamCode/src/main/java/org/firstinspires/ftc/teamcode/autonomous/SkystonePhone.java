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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.opencv.core.Core.minMaxLoc;
import static org.opencv.imgcodecs.Imgcodecs.imread;

@Autonomous
public class SkystonePhone extends LinearOpMode {
    OpenCvCamera phonecam;
    SkystonePipeline SkystonePipeline;

    List<MatOfPoint> Contours = new ArrayList<>(); //define global vars
    public int numtaps = 1;

    public int templateMatchMethod = 0;

    public double templatethresh = 0.45;

    public double templateScore = 0;
    Mat templ;


    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phonecam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phonecam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        phonecam.openCameraDevice();
        SkystonePipeline = new SkystonePipeline();
        phonecam.setPipeline(SkystonePipeline);
        phonecam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


        String path = Environment.getExternalStorageDirectory() + "/skystonetemplate.png";

        //URL res = getClass().getClassLoader().getResource("template0.png");
        templ = imread(path);

        waitForStart();
        int autostate = 0;

        while (opModeIsActive()) {
            telemetry.update();

            telemetry.addData("templateScore: ", templateScore);
        }
    }


    public class SkystonePipeline extends OpenCvPipeline {

        Mat templout = new Mat();
        Mat inputrgb = new Mat();
        Mat input2 = new Mat();
        int autostate = 0;


        public Mat processFrame(Mat input) {

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

            double xmid = matchLoc.x + templ.cols() / 2; //divide by 2 for midpoint of width/height of box + its location
            double ymid = matchLoc.y + templ.rows() / 2;
            Point center = new Point(xmid, ymid);

            if (templateMatchMethod == 5 & templateScore >= templatethresh) {
                Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templ.cols(), matchLoc.y + templ.rows()), new Scalar(0, 255, 0), 5);
                Imgproc.drawMarker(input, center, new Scalar(240, 130, 240));
            } else if (templateScore < templatethresh) {
                Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templ.cols(), matchLoc.y + templ.rows()), new Scalar(255, 0, 0), 2);
            }


            if (autostate == 0) {
                telemetry.update();

            }
            return input; //render to viewport
        }
    }
}




// if (templateMatchMethod == 0 || templateMatchMethod == 2) { // normalize(templout, templout, 0, 1, NORM_MINMAX, -1); // }




//
//

