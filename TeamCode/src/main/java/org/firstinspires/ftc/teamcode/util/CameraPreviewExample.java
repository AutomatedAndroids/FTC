//package org.firstinspires.ftc.teamcode.util;
//
//// Inludes common necessary includes for development using depthai library
//
//import static org.bytedeco.opencv.global.opencv_core.CV_8UC3;
//import static org.bytedeco.opencv.global.opencv_highgui.imshow;
//import static org.bytedeco.opencv.global.opencv_highgui.waitKey;
//
//import org.bytedeco.depthai.ColorCamera;
//import org.bytedeco.depthai.ColorCameraProperties;
//import org.bytedeco.depthai.DataOutputQueue;
//import org.bytedeco.depthai.Device;
//import org.bytedeco.depthai.ImgFrame;
//import org.bytedeco.depthai.Pipeline;
//import org.bytedeco.depthai.XLinkOut;
//import org.bytedeco.javacpp.IntPointer;
//import org.bytedeco.opencv.opencv_core.Mat;
//
//public class CameraPreviewExample {
//    static Pipeline createCameraPipeline() {
//        Pipeline p = new Pipeline();
//
//        ColorCamera colorCam = p.createColorCamera();
//        XLinkOut xlinkOut = p.createXLinkOut();
//        xlinkOut.setStreamName("preview");
//
//        colorCam.setPreviewSize(300, 300);
//        colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);
//        colorCam.setInterleaved(true);
//
//        // Link plugins CAM -> XLINK
//        colorCam.preview().link(xlinkOut.input());
//
//        return p;
//    }
//
//    public static void main(String[] args) {
//        Pipeline p = createCameraPipeline();
//
//        // Start the pipeline
//        Device d = new Device();
//
//        System.out.print("Connected cameras: ");
//        IntPointer cameras = d.getConnectedCameras();
//        for (int i = 0; i < cameras.limit(); i++) {
//            System.out.print(cameras.get(i) + " ");
//        }
//        System.out.println();
//
//        // Start the pipeline
//        d.startPipeline(p);
//
//        Mat frame;
//        DataOutputQueue preview = d.getOutputQueue("preview");
//
//        while (true) {
//            ImgFrame imgFrame = preview.getImgFrame();
//            if (imgFrame != null) {
//                System.out.printf("Frame - w: %d, h: %d\n", imgFrame.getWidth(), imgFrame.getHeight());
//                frame = new Mat(imgFrame.getHeight(), imgFrame.getWidth(), CV_8UC3, imgFrame.getData());
//                imshow("preview", frame);
//                int key = waitKey(1);
//                if (key == 'q') {
//                    System.exit(0);
//                }
//            } else {
//                System.out.println("Not ImgFrame");
//            }
//        }
//    }
//}