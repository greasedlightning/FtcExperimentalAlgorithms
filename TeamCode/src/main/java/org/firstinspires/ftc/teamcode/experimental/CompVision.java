package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

public class CompVision {
    private OpenCvCamera cam;

    public CompVision(HardwareMap map) {
        int monId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, monId);
        cam.setPipeline(new PipeLine());
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

    WebcamName[] webcams;
    OpenCvSwitchableWebcam switchableWebcam;
    public CompVision(HardwareMap map, int camsN) {
        camsN = 2;
        webcams = new WebcamName[camsN];
        for (int i = 0; i < camsN; i++) {
            webcams[i] = map.get(WebcamName.class, "Webcam " + camsN);
        }
        int monId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(monId, webcams[0], webcams[1]);
        PipeLine[] camPipeline = { new PipeLine(), new PipeLine() };

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                String name = switchableWebcam.getActiveCamera().getDeviceName();
                int id = Integer.valueOf(name.charAt(7));

                switchableWebcam.setPipeline(camPipeline[id]);
                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

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
        switchableWebcam.setActiveCamera(webcams[i]);
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
    class PipeLine extends OpenCvPipeline
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
}
