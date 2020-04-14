package org.firstinspires.ftc.teamcode.robot.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
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
    private Mat skystoneTemplateImage;

    public Point center;

    public enum States {
        Undetermined,
        ObjectFound,
        ObjectLost,
    }

    public States State;

    public CameraController(OpenCvCamera camera, int width, int height, OpenCvCameraRotation cameraRotation, String templateImageFileName) {
        skystoneTemplateImage = imread(templateImageFileName);

        openCvCamera = camera;
        openCvCamera.openCameraDevice();
        openCvCamera.setPipeline(this);
        openCvCamera.startStreaming(width, height, cameraRotation);
    }

    public void onViewportTapped() {

    }

    public double TemplateScore = templateScore;


    public Mat processFrame(Mat input) {

        Mat inputrgb = new Mat();
        Imgproc.cvtColor(input, inputrgb, Imgproc.COLOR_RGBA2RGB);

        Mat templout = new Mat();
        try {
            Imgproc.matchTemplate(inputrgb, skystoneTemplateImage, templout, Imgproc.TM_CCOEFF_NORMED);
        }
        catch (Exception e) {
            // telemetry.addData("exception", e.getMessage());
        }

        Core.MinMaxLocResult mmr = minMaxLoc(templout);

        Point matchLoc = mmr.maxLoc;
        double maxVal = mmr.maxVal;

        templateScore = maxVal;

        double xmid = matchLoc.x + skystoneTemplateImage.cols()/2; //divide by 2 for midpoint of width/height of box + its location
        double ymid = matchLoc.y + skystoneTemplateImage.rows()/2;
        center = new Point(xmid, ymid);

        if (templateScore > 0.5) {
            State = States.ObjectFound;
            Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + skystoneTemplateImage.cols(), matchLoc.y + skystoneTemplateImage.rows()), new Scalar(0, 255, 0), 5);
            Imgproc.drawMarker(input,center, new Scalar(240,130,240));
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
