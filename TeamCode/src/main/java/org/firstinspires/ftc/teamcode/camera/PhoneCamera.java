package org.firstinspires.ftc.teamcode.camera;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.core.Core.minMaxLoc;
import static org.opencv.imgcodecs.Imgcodecs.imread;

/**
 * Usage: Implements pphone camera control
 *
 *
 *
 * Author: Daniel
 */
@Autonomous
public class PhoneCamera extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private SkystonePipeline SkystonePipeline;

    private int templateMatchMethod =  0;

    private double templateScore =  0;
    private Mat templ;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        String path= Environment.getExternalStorageDirectory()+ "/skystonetemplate.png";

        templ = imread(path);

        waitForStart();

        SkystonePipeline = new SkystonePipeline();
        phoneCam.setPipeline(SkystonePipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        while (opModeIsActive()) {
            telemetry.update();

            telemetry.addData("Method: ", templateMatchMethod);
            telemetry.addData("TemplateScore: ", templateScore);
        }
    }

    public class SkystonePipeline extends OpenCvPipeline {

        public void onViewportTapped() {

        }

        public Mat processFrame(Mat input) {

            Mat inputrgb = new Mat();
            Imgproc.cvtColor(input, inputrgb, Imgproc.COLOR_RGBA2RGB);

            Mat templout = new Mat();
            try {
                Imgproc.matchTemplate(inputrgb, templ, templout, templateMatchMethod);
            }
            catch (Exception e) {
                telemetry.addData("exception", e.getMessage());
            }

            Core.MinMaxLocResult mmr = minMaxLoc(templout);

            Point matchLoc = mmr.maxLoc;
            double maxVal = mmr.maxVal;

            templateScore = maxVal;

            if (templateScore > 0.5) {
                Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templ.cols(), matchLoc.y + templ.rows()), new Scalar(0, 255, 0), 5);
            }

            return input; //render to viewport
        }
    }
}
