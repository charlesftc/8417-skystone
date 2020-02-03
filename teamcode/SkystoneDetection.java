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
            leftRect = new Rect(50, 20, 25, 120);
            centerRect = new Rect(257, 20, 25, 120);
            rightRect = new Rect(515, 20, 25, 120);
        } else {
            leftRect = new Rect(150, 20, 25, 120);
            centerRect = new Rect(357, 20, 25, 120);
            rightRect = new Rect(565, 20, 25, 120);
        }
        rects[0] = leftRect;
        rects[1] = centerRect;
        rects[2] = rightRect;
    }

    public void initialize(boolean shouldMonitor) {
        if (shouldMonitor) {
            int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().
                    getIdentifier("cameraMonitorViewId", "id",
                            opmode.hardwareMap.appContext.getPackageName());
            usbCam = OpenCvCameraFactory.getInstance().createWebcam(opmode.hardwareMap.get(
                    WebcamName.class, "usb_cam"), cameraMonitorViewId);
        } else {
            usbCam = OpenCvCameraFactory.getInstance().createWebcam(opmode.hardwareMap.get(
                    WebcamName.class, "usb_cam"));
        }
        usbCam.openCameraDevice();
        usbCam.setPipeline(new SkystonePipeline());
    }

    public void startStream() {
        usbCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
    }

    public void stopStream() {
        usbCam.stopStreaming();
        usbCam.closeCameraDevice();
    }

    class SkystonePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            //draw the three stone detection rectangles on the screen (if monitoring is enabled)
            Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, rightRect, new Scalar(255, 0, 0), 2);
            //get the updated pixel values for each of the three rectangles
            leftVal = getRectVal(input, leftRect);
            centerVal = getRectVal(input, centerRect);
            rightVal = getRectVal(input, rightRect);
            return input;
        }
    }

    private int getRectVal(Mat input, Rect rect) {
        //for the given rectangle, create a val variable
        int val = 0;
        //for each column:
        for (int i = rect.x; i < rect.x + rect.width; i++) {
            //for each row:
            for (int j = rect.y; j < rect.y + rect.height; j++) {
                //increment val by this pixel's red and green color values
                double[] rgb = input.get(j, i);
                val += (int)rgb[0] + (int)rgb[1];
            }
        }
        return val;
    }

    public int getSkystonePos() {
        //if left rectangle has the smallest red + green value, the skystone is in the left position
        if (leftVal < centerVal && leftVal < rightVal) {
            return 0;
        } else if (centerVal < leftVal && centerVal < rightVal) { //likewise for the center one
            return 1;
        } else { //otherwise, the skystone must be in the right position
            return 2;
        }
    }
}