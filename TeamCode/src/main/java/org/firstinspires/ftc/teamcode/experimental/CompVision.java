package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.ArrayList;

public class CompVision {
    private OpenCvCamera cam;

    public CompVision(HardwareMap map) {
        int monId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, monId);
        cam.setPipeline(new DetectShippingElementPipeLine());
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    OpenCvCamera webcam1;
    OpenCvCamera webcam2;
    OpenCvCamera[] webcams;
    public CompVision(HardwareMap map, int camsN) {
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        camsN, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally


        for (int i = 0; i < camsN; i++) {
            webcams[i] = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam " + (i+1)), viewportContainerIds[i]);
        }

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.setPipeline(new DetectShippingElementPipeLine());
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam2.setPipeline(new DetectCubesPipeline());
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    public void setCam(int i) {

    }

    public void printStats(Telemetry t) {
        t.addData("Frame Count", cam.getFrameCount());
        t.addData("FPS", String.format("%.2f", cam.getFps()));
        t.addData("Total frame time ms", cam.getTotalFrameTimeMs());
        t.addData("Pipeline time ms", cam.getPipelineTimeMs());
    }
    public void stop() {
        cam.stopStreaming();
    }


    private int path = -1;
    private float d0;
    private float d1;
    private float d2;
    public int getPath() {
        return path;
    }
    public void printDebug(Telemetry t) {
        t.addLine("d0: " + d0);
        t.addLine("d1: " + d1);
        t.addLine("d2: " + d2);
        t.addLine("Path chosen: " + path);
    }
    //320, 240
    class DetectShippingElementPipeLine extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        @Override
        public Mat processFrame(Mat input) {
            choosePath(input, false);
            return input;
        }

        private void choosePath(Mat input, boolean debug) {
            boolean go;
            if (debug) {
                go = path >= -1;
            } else {
                go = path == -1;
            }

            if (go) {
//              d0 = greenDensityRatio(input, 140, 0, 70, 70);
//              d1 = greenDensityRatio(input, 110, 130, 80, 90);
//              d2 = greenDensityRatio(input, 90, 290, 80, 40);
                d0 = greenDensityRatio(input, 0, 0, input.height(), 100);
                d1 = greenDensityRatio(input, 0, 100, input.height(), 120);
                d2 = greenDensityRatio(input, 0, 220, input.height(), 100);
                if (d0 > d1) {
                    if (d0 > d2) {
                        path = 0;
                    } else {
                        path = 2;
                    }
                } else {
                    if (d1 > d2) {
                        path = 1;
                    } else {
                        path = 2;
                    }
                }
            }
        }

        private float greenDensityRatio(Mat input, int _y, int _x, int height, int width) {
            float r = 0;
            float g = 0;
            float b = 0;
            int tot = 0;
            for (int x = _x; x < _x + width && x < input.height(); x++) {
                for (int y = _y; y < _y + height && y < input.width(); y++) {
                    double[] d = input.get(x, y);
                    r += d[0];
                    g += d[1];
                    b += d[2];
                    tot++;
                }
            }

            return 100 + (g*3 - r - b)/3.0f/tot;
        }


        //Old Functions --

        private float greenDensityRatioOld(Mat input, int _y, int _x, int height, int width) {
            float r = 0;
            float g = 0;
            float b = 0;
            int tot = 0;
            for (int x = _x; x < _x + width && x < input.height(); x++) {
                for (int y = _y; y < _y + height && y < input.width(); y++) {
                    double[] d = input.get(x, y);
                    r += d[0];
                    g += d[1];
                    b += d[2];
                    tot++;
                }
            }
            return g / tot - r / tot - b / tot;
        }
        private float greenDensity(Mat input, int _y, int _x, int height, int width) {
            float green = 0;
            int tot = 0;
            for (int x = _x; x < _x + width && x < input.height(); x++) {
                for (int y = _y; y < _y + height && y < input.width(); y++) {
                    double[] d = input.get(x, y);
                    green += d[1];
                    tot++;
                }
            }
            return green / tot;
        }
        @Override
        public void onViewportTapped()
        {

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                cam.pauseViewport();
            }
            else
            {
                cam.resumeViewport();
            }
        }
    }

    class DetectTowerPipeLine extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat src) {
            Mat gray = new Mat(src.rows(), src.cols(), src.type());
            Mat edges = new Mat(src.rows(), src.cols(), src.type());
            Mat dst = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));
            //Converting the image to Gray
            Imgproc.cvtColor(src, gray, Imgproc.COLOR_RGB2GRAY);
            //Blurring the image
            Imgproc.blur(gray, edges, new Size(3, 3));
            //Detecting the edges
            Imgproc.Canny(edges, edges, 100, 100*3);
            //Copying the detected edges to the destination matrix
            src.copyTo(dst, edges);
            return dst;
        }
    }


    class DetectCubesPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat src) {
            Mat usedMat = prepareFrame(src);
            getCubes(usedMat, 20);
            return usedMat;
        }

        public Mat prepareFrame(Mat src) {
            int iLowH = 0;
            int iHighH = 255;
            int iLowS = 0;
            int iHighS = 135;
            int iLowV = 0;
            int iHighV = 255;

            Mat hsv = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));
            Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV);
            Mat imgThresholded = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));

            Core.inRange(hsv, new Scalar(iLowH, iLowS, iLowV), new Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

            Imgproc.erode(imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)) );
            Imgproc.dilate( imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)) );

            Imgproc.dilate( imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)) );
            Imgproc.erode(imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)) );

            Imgproc.resize(imgThresholded, imgThresholded, new Size(80, 60));

            return imgThresholded;
        }

        public ArrayList<Vector2> getCubes(Mat src, int tolerance) {
            ArrayList<Vector2> positions = new ArrayList<Vector2>();
            for (int x = 0; x < src.rows(); x++) {
                for (int y = 0; y < src.cols(); y++) {

                    Vector2 p = new Vector2(0, 0);
                    fill(src, p, x, y);

                    if (p.c > tolerance) {
                        positions.add(p);
                        p.clean();
                    }
                }
            }

            for (Vector2 p : positions) {
                putDot(src, p, p.x, p.y);
            }
            return positions;
        }

        public boolean valid(Mat src, int x, int y) {
            return !(x < 0 || y < 0 || x >= src.rows() || y >= src.cols());
        }

        public void fill(Mat src, Vector2 p, int x, int y) {
            if (!valid(src, x, y) || src.get(x, y)[0] > 150) { return; }

            src.put(x, y, 200);
            p.x += x;
            p.y += y;
            p.c++;

            fill(src, p, x+3, y);
            fill(src, p, x, y+3);
            fill(src, p, x, y-3);
            fill(src, p, x-3, y);

            fill(src, p, x+1, y);
            fill(src, p, x, y+1);
            fill(src, p, x, y-1);
            fill(src, p, x-1, y);
        }

        public void putDot(Mat src, Vector2 p, int x, int y) {
            final int dist = 1;
            if (!valid(src, x, y) || Math.abs(p.x - x) > dist || Math.abs(p.y - y) > dist || src.get(x, y)[0] < 10) {
                return;
            }

            src.put(x, y, new double[]{0});

            putDot(src, p, x - 1, y);
            putDot(src, p, x, y - 1);
            putDot(src, p, x + 1, y);
            putDot(src, p, x, y + 1);
        }

    }

    private class Vector2 {
        public Vector2(int _x, int _y) {
            x = _x;
            y = _y;
            c = 0;
        }

        public void clean() {
            if (c != 0) {
                x /= c;
                y /= c;

            }
        }
        public int x;
        public int y;
        public int c;
    }
}
