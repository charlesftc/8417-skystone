package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="SkystoneDetection", group="Linear Opmode")
public class SkystoneDetection extends LinearOpMode {
    OpenCvCamera phoneCam;
    private int width = 480;
    private int height = 640;
    /*private Point leftPos = new Point((double)width * (1f / 4f), (double)height * (1f / 2f));
    private Point centerPos = new Point((double)width * (1f / 2f), (double)height * (1f / 2f));
    private Point rightPos = new Point((double)width * (3f / 4f), (double)height * (1f / 2f));*/

    private Rect leftRect = new Rect(101, 200, 25, 150);
    private Rect centerRect = new Rect(227, 200, 25, 150);
    private Rect rightRect = new Rect(353, 200, 25, 150);
    private Rect[] rects = {leftRect, centerRect, rightRect};

    private int leftVal = -1;
    private int centerVal = -1;
    private int rightVal = -1;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SkystonePipeline());
        phoneCam.startStreaming(height, width, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("pos", getSkystonePos());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.update();
            /*if(gamepad1.a)
            {
                phoneCam.stopStreaming();
                //webcam.closeCameraDevice();
            } else if(gamepad1.x) {
                phoneCam.pauseViewport();
            } else if(gamepad1.y) {
                phoneCam.resumeViewport();
            }*/
            sleep(100);
        }
    }

    class SkystonePipeline extends OpenCvPipeline {
        //Mat mat = new Mat();
        @Override
        public Mat processFrame(Mat input) {
            /*Imgproc.circle(input, leftPos, 5, new Scalar(255, 0, 0), 2);
            Imgproc.circle(input, centerPos, 5, new Scalar(255, 0, 0), 2);
            Imgproc.circle(input, rightPos, 5, new Scalar(255, 0, 0), 2);*/

            Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, centerRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, rightRect, new Scalar(255, 0, 0), 2);

            leftVal = scanRect(input, leftRect);
            centerVal = scanRect(input, centerRect);
            rightVal = scanRect(input, rightRect);

            /*leftVal = (int)(input.get((int)leftPos.y, (int)leftPos.x))[0];
            centerVal = (int)(input.get((int)centerPos.y, (int)centerPos.x))[0];
            rightVal = (int)(input.get((int)rightPos.y, (int)rightPos.x))[0];*/

            /*telemetry.addData("left", leftVal);
            telemetry.addData("center", centerVal);
            telemetry.addData("right", rightVal);

            telemetry.addData("stuff", "rows: %d, cols: %d", input.rows(), input.cols());
            telemetry.update();*/

            return input;
        }
    }

    private int scanRect(Mat input, Rect rect) {
        int val = 0;
        for (int i = rect.x; i < rect.x + rect.width; i++) {
            for (int j = rect.y; j < rect.y + rect.height; j++) {
                double[] rgb = input.get(j, i);
                val += (int)rgb[0] + (int)rgb[1];
            }
        }
        return val / (int)rect.area();
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