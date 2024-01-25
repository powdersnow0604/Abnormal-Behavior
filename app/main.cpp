#include "detect.h"
#include <fstream>
#include <chrono>


using namespace std;
using namespace cv;
using namespace cv::dnn;


// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;

// Colors.
Scalar BLACK = Scalar(0, 0, 0);
Scalar BLUE = Scalar(255, 178, 50);
Scalar YELLOW = Scalar(0, 255, 255);
Scalar RED = Scalar(0, 0, 255);


int main()
{
    // Load class list.
    vector<string> class_list;
    ifstream ifs("../coco.names");
    string line;

    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }

    // Load image.
    const char *resrcpath = std::getenv("RESOURCE_PATH");
    auto vid = VideoCapture(resrcpath + string("video/amazon.mp4"));
    Mat frame, frame_b, frame_p;

    // Load model.
    Net net;
    net = readNet("../models/yolov5s.onnx");

    vector<Mat> detections;
    det_init();

    if (vid.isOpened())
    {
        while ((waitKey(1) & 0xff) != 'q')
        {
            if (vid.get(CAP_PROP_FRAME_COUNT) == vid.get(CAP_PROP_POS_FRAMES))
                break;
            (void)vid.read(frame);
            frame.copyTo(frame_b);

            auto s = chrono::high_resolution_clock::now();
            detections = pre_process(frame, net);

            (void)post_process(frame, frame_b, detections, class_list);

            // Put efficiency information.
            // The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)

            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(chrono::high_resolution_clock::now() - s);
            string label = format("Inference time : %.2f ms", (float)(duration.count()) / 1000000.);
            putText(frame, label, Point(20, 40), FONT_FACE, FONT_SCALE, RED);

            hconcat(frame, frame_b, frame_p);

            imshow("Output", frame_p);
        }
    }

    det_destroy();
    destroyAllWindows();
    vid.release();

    return 0;
}