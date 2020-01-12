package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetection {
    LinearOpMode opmode;
    OpenCvCamera usbCam;
    private int width = 640;
    private int height = 480;

    private Rect leftRect;
    private Rect centerRect;
    private Rect rightRect;
    private Rect[] rects = new Rect[3];

    private int leftVal = -1;
    private int centerVal = -1;
    private int rightVal = -1;

    public SkystoneDetection(LinearOpMode op, boolean isRed) {
        opmode = op;
        if (isRed) {
            /*leftRect = new Rect(75, 20, 25, 170);
            centerRect = new Rect(307, 20, 25, 170);
            rightRect = new Rect(540, 20, 25, 170);*/
            leftRect = new Rect(25, 20, 25, 170);
            centerRect = new Rect(257, 20, 25, 170);
            rightRect = new Rect(490, 20, 25, 170);
        } else {
            leftRect = new Rect(125, 20, 25, 170);
            centerRect = new Rect(357, 20, 25, 170);
            rightRect = new Rect(590, 20, 25, 170);
        }
        rects[0] = leftRect;
        rects[1] = centerRect;
        rects[2] = rightRect;
    }

    public void initialize() {
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().
            getIdentifier("cameraMonitorViewId", "id",
            opmode.hardwareMap.appContext.getPackageName());
        /*phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.
            CameraDirection.BACK, cameraMonitorViewId);*/
        usbCam = OpenCvCameraFactory.getInstance().createWebcam(opmode.hardwareMap.get(
                WebcamName.class, "usb_cam"), cameraMonitorViewId);
        usbCam.openCameraDevice();
        usbCam.setPipeline(new SkystonePipeline());
    }

    public void startStream() {
        usbCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
    }

    public void stopStream() {
        usbCam.stopStreaming();
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