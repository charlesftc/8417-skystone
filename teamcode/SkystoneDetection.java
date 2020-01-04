package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetection {
    LinearOpMode opmode;
    OpenCvCamera phoneCam;
    private int width = 480;
    private int height = 640;

    private Rect leftRect = new Rect(101, 200, 25, 150);
    private Rect centerRect = new Rect(227, 200, 25, 150);
    private Rect rightRect = new Rect(353, 200, 25, 150);
    private Rect[] rects = {leftRect, centerRect, rightRect};

    private int leftVal = -1;
    private int centerVal = -1;
    private int rightVal = -1;

    public SkystoneDetection(LinearOpMode op) {
        opmode = op;
    }

    public void initialize() {
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().
            getIdentifier("cameraMonitorViewId", "id",
            opmode.hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.
            CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.
        //  CameraDirection.BACK);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SkystonePipeline());
    }

    public void startStream() {
        phoneCam.startStreaming(height, width, OpenCvCameraRotation.UPRIGHT);
    }

    public void stopStream() {
        phoneCam.stopStreaming();
        //webcam.closeCameraDevice();
    }

    class SkystonePipeline extends OpenCvPipeline {
        //Mat mat = new Mat();
        @Override
        public Mat processFrame(Mat input) {

            Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, rightRect, new Scalar(255, 0, 0), 2);

            leftVal = getRectVal(input, leftRect);
            centerVal = getRectVal(input, centerRect);
            rightVal = getRectVal(input, rightRect);

            return input;
        }
    }

    private int getRectVal(Mat input, Rect rect) {
        int val = 0;
        for (int i = rect.x; i < rect.x + rect.width; i++) {
            for (int j = rect.y; j < rect.y + rect.height; j++) {
                double[] rgb = input.get(j, i);
                val += (int)rgb[0] + (int)rgb[1];
            }
        }
        return val;
    }

    public int getSkystonePos() {
        if (leftVal < centerVal && leftVal < rightVal) {
            return 0;
        } else if (centerVal < leftVal && centerVal < rightVal) {
            return 1;
        } else {
            return 2;
        }
    }
}