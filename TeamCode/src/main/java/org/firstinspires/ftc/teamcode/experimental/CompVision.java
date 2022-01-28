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

import java.util.ArrayList;
import java.util.Vector;

public class CompVision {
    private OpenCvCamera[] cams;
    private int camNum = 0;

    public CompVision(HardwareMap map, int n) {
        camNum = n;
        cams = new OpenCvCamera[camNum];
        int monId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());

        if (camNum > 1) {
            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            monId, //The container we're splitting
                            camNum, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY
                    ); //Whether to split the container vertically or horizontally
            for (int i = 0; i < camNum; i++) {
                cams[i] = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam " + (i + 1)), viewportContainerIds[i]);
            }

        } else {
            cams[0] = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"));
        }

        if (camNum > 0) {
            cams[0].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    cams[0].setPipeline(new ShippingElementPipeLine());
                    cams[0].startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
            if (camNum > 1) {
                cams[1].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        cams[1].setPipeline(new CubesPipeLine());
                        cams[1].startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                });
            }
        }
    }

    public void printStats(Telemetry t) {
        for (int i = 0; i < camNum; i++) {
            try {
                t.addLine("Cam" + (i + 1));
                t.addData("Frame Count", cams[i].getFrameCount());
                t.addData("FPS", String.format("%.2f", cams[i].getFps()));
                t.addData("Total frame time ms", cams[i].getTotalFrameTimeMs());
                t.addData("Pipeline time ms", cams[i].getPipelineTimeMs());
            } catch (Exception e) {
                t.addLine("Cam" + (i + 1) + " not available");
            }
        }
    }

    private int path = -1;
    private float d0;
    private float d1;
    private float d2;

    public int getPath() {
        return path;
    }


    private ArrayList<Vector2> cubes = new ArrayList<>();

    public Vector2[] getCubes() {
        Vector2[] result = new Vector2[cubes.size()];
        for (int i = 0; i < result.length; i++) {
            result[i] = cubes.get(i);
        }
        for (int i = 0; i < result.length; i++) {
            Vector2 max = result[i];
            int maxI = i;
            for (int j = i+1; j < result.length; j++) {
                Vector2 temp = result[j];
                if (temp.x > max.x) {
                    max = temp;
                    maxI = j;
                }
            }
            result[maxI] = result[i];
            result[i] = max;
        }
        return result;
    }


    public void printDebugCam1(Telemetry t) {
        t.addLine("d0: " + d0);
        t.addLine("d1: " + d1);
        t.addLine("d2: " + d2);
        t.addLine("Path chosen: " + path);
    }

    public void stopAll() {
        for (int i = 0; i < camNum; i++) {
            cams[i].stopStreaming();
        }
    }

    public void stop(int i) {
        cams[i].stopStreaming();
    }

    public void changePipeLine(int i, int pipeline) {
        OpenCvCamera cam = cams[i];
        switch (pipeline) {
            case 0:
                cam.setPipeline(new ShippingElementPipeLine());
                break;
            case 1:
                cam.setPipeline(new CubesPipeLine());
                break;
            case 2:
                cam.setPipeline(new TestPipeLine());
        }
    }
    //320, 240

    class ShippingElementPipeLine extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.resize(input, input, new Size(80, 60));
            Vector2 ele = getShippingE(input, 10);


            if (ele == null) {
                path = 0;
            } else {
                //path = ele.y;

                if (ele.y < 20) {
                    path = 0;
                } else if ( ele.y < 55) {
                    path = 1;
                } else {
                    path = 2;
                }

            }


            return input;
        }

        public Vector2 getShippingE(Mat src, int tolerance) {
            ArrayList<Vector2> positions = new ArrayList<Vector2>();
            for (int x = 0; x < src.rows(); x++) {
                for (int y = 0; y < src.cols(); y++) {

                    Vector2 p = new Vector2(0, 0);
                    Efill(src, p, x, y);

                    if (p.c > tolerance) {
                        positions.add(p);
                        p.clean();
                    }
                }
            }

            Vector2 maxVec = null;
            //System.out.println(positions.size());
            if (positions.size() > 0) {
                maxVec = positions.get(0);
                for (Vector2 p : positions) {
                    if (p.c > maxVec.c) {
                        maxVec = p;
                    }
                    //putDotE(src, p, p.x, p.y);
                }
                putDotE(src, maxVec, maxVec.x, maxVec.y);

            }

            return maxVec;
        }

        public void Efill(Mat src, Vector2 p, int x, int y) {
            if (!valid(src, x, y) ) { return; }

            double[] pix = src.get(x, y);
            double r = pix[0];
            double g = pix[1];
            double b = pix[2];

            double tot = r + g + b;

            if (!(106 < (g / tot)*300 && tot/4 > 30) || tot == 765) { return; }

            pix[0] = 255;
            pix[1] = 255;
            pix[2] = 255;

            src.put(x, y, pix);

            p.x += x;
            p.y += y;
            p.c++;

            Efill(src, p, x+2, y);
            Efill(src, p, x, y+2);
            Efill(src, p, x, y-2);
            Efill(src, p, x-2, y);

            Efill(src, p, x+1, y);
            Efill(src, p, x, y+1);
            Efill(src, p, x, y-1);
            Efill(src, p, x-1, y);

        }

        public boolean valid(Mat src, int x, int y) {
            return !(x < 0 || y < 0 || x >= src.rows() || y >= src.cols() || src.get(x, y) == null);
        }

        public void putDotE(Mat src, Vector2 p, int x, int y) {
            final int dist = 1;

            if (Math.abs(p.x-x) > dist || Math.abs(p.y-y) > dist || src.get(x, y)[0] < 10) {
                return;
            }


            double[] c = src.get(x, y);
            c[0] = 0;
            c[1] = 0;
            c[2] = 0;
            src.put(x, y, c);

            if (x > 0) {
                putDotE(src, p, x-1, y);
            }
            if (y > 0) {
                putDotE(src, p, x, y-1);
            }
            if (x < src.rows() - 1) {
                putDotE(src, p, x+1, y);
            }
            if (y < src.cols() - 1) {
                putDotE(src, p, x, y+1);
            }
        }
    }

    class ShippingElementPipeLineOLD extends OpenCvPipeline {
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
            return 100 + (g * 3 - r - b) / 3.0f / tot;
        }
    }
    class CubesPipeLine extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat src) {

            int iLowH = 0;
            int iHighH = 255;
            int iLowS = 0;
            int iHighS = 11/6;
            int iLowV = 0;
            int iHighV = 255;

            Mat hsv = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));
            Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV);
            Mat imgThresholded = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));

            Core.inRange(hsv, new Scalar(iLowH, iLowS, iLowV), new Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

            Imgproc.erode(imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
            Imgproc.dilate(imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

            Imgproc.dilate(imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
            Imgproc.erode(imgThresholded, imgThresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

            Imgproc.resize(imgThresholded, imgThresholded, new Size(80, 60));

            cubes = getCubes(imgThresholded, 20);

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
            return !(x < 0 || y < 0 || x >= src.rows() || y >= src.cols() || src.get(x, y) == null);
        }

        public void fill(Mat src, Vector2 p, int x, int y) {
            if (!valid(src, x, y) || src.get(x, y)[0] > 150) {
                return;
            }

            src.put(x, y, 200);
            p.x += x;
            p.y += y;
            p.c++;

            int tol = 3;
            fill(src, p, x + tol, y);
            fill(src, p, x, y + tol);
            fill(src, p, x, y - tol);
            fill(src, p, x - tol, y);

            fill(src, p, x + 1, y);
            fill(src, p, x, y + 1);
            fill(src, p, x, y - 1);
            fill(src, p, x - 1, y);
        }

        public void putDot(Mat src, Vector2 p, int x, int y) {
            final int dist = 1;

            if (Math.abs(p.x - x) > dist || Math.abs(p.y - y) > dist || src.get(x, y)[0] < 10) {
                return;
            }

            src.put(x, y, new double[]{0});

            if (x > 0) {
                putDot(src, p, x - 1, y);
            }
            if (y > 0) {
                putDot(src, p, x, y - 1);
            }
            if (x < src.rows() - 1) {
                putDot(src, p, x + 1, y);
            }
            if (y < src.cols() - 1) {
                putDot(src, p, x, y + 1);
            }
        }
    }
    class TestPipeLine extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            return input;
        }
    }
}
