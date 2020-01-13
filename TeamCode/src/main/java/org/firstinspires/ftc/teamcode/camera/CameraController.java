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
 * Usage: Implements camera controller.
 *
 *
 *
 * Author: Daniel
 */
public class CameraController extends OpenCvPipeline {
    private OpenCvCamera openCvCamera;

    private double templateScore =  0;
    private Mat templ;

    public enum States {
        Undetermined,
        ObjectFound,
        ObjectLost,
    }

    public States State;

    public CameraController(OpenCvCamera camera, int width, int height) {
        String path= Environment.getExternalStorageDirectory()+ "/skystonetemplate.png";
        templ = imread(path);

        openCvCamera = camera;
        openCvCamera.openCameraDevice();
        openCvCamera.setPipeline(this);
        openCvCamera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
    }

    public void onViewportTapped() {

    }

    public Mat processFrame(Mat input) {

        Mat inputrgb = new Mat();
        Imgproc.cvtColor(input, inputrgb, Imgproc.COLOR_RGBA2RGB);

        Mat templout = new Mat();
        try {
            Imgproc.matchTemplate(inputrgb, templ, templout, Imgproc.TM_CCOEFF_NORMED);
        }
        catch (Exception e) {
            // telemetry.addData("exception", e.getMessage());
        }

        Core.MinMaxLocResult mmr = minMaxLoc(templout);

        Point matchLoc = mmr.maxLoc;
        double maxVal = mmr.maxVal;

        templateScore = maxVal;

        if (templateScore > 0.5) {
            State = States.ObjectFound;
            Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templ.cols(), matchLoc.y + templ.rows()), new Scalar(0, 255, 0), 5);
        }
        else
        {
            if (State == States.ObjectFound) {
                State = States.ObjectLost;
            }
            else {
                State = States.Undetermined;
            }

        }

        return input; //render to viewport
    }
}
