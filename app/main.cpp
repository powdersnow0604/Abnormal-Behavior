#include "detect.h"
#include <fstream>
#include <chrono>
#include <unistd.h>


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

    string vidpath[2] = {string("video/MOT17-03-SDP-raw.webm"), string("video/amazon.mp4")};

    // Load image.
    const char *resrcpath = std::getenv("RESOURCE_PATH");
    auto vid = VideoCapture(resrcpath + vidpath[0]);
    auto res = VideoWriter();

    Mat frame, frame_b, frame_p;

    // Load model.
    Net net;
    net = readNet("../models/yolov5m.onnx");

    float videoFPS = vid.get(cv::CAP_PROP_FPS);
	int videoWidth = vid.get(cv::CAP_PROP_FRAME_WIDTH);
	int videoHeight = vid.get(cv::CAP_PROP_FRAME_HEIGHT);

    string output_path = resrcpath + string("video/otres.mp4");
    if(access(output_path.c_str(), F_OK) == 0){
        remove(output_path.c_str());
    }

    res.open(output_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
    videoFPS , cv::Size(videoWidth << 1, videoHeight), true);

    vector<Mat> detections;
    det_init(videoWidth, videoHeight);
    //det_init();

    auto total_s = chrono::high_resolution_clock::now();

    //vid.set(CAP_PROP_POS_FRAMES, 350);

    uint8_t key;

    if (vid.isOpened() && res.isOpened())
    {
        while ((key = (waitKey(1) & 0xff)) != 'q')
        {
            if(key == 'w') waitKey();

            if (vid.get(CAP_PROP_FRAME_COUNT) == vid.get(CAP_PROP_POS_FRAMES))
                break;

            vid >> frame;
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
            res << frame_p;
        }
    }
    else{
        cerr << "open error\n";
    }

    auto total_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(chrono::high_resolution_clock::now() - total_s);
    printf("total time = %.2f ms\n", (float)(total_duration.count()) / 1000000.);
    printf("fps = %f\n", vid.get(CAP_PROP_POS_FRAMES) / ((float)(total_duration.count() / 1000000000.)));

    det_destroy();
    cv::destroyAllWindows();
    vid.release();
    res.release();

    return 0;
}