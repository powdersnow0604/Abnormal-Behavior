// Include Libraries.
#include <fstream>
#include "detect.h"
#include "tracker.h"
#include "iou.h"

// Namespaces.
using namespace cv;
using namespace std;
using namespace cv::dnn;

// Constants.
static const float INPUT_WIDTH = 640.0;
static const float INPUT_HEIGHT = 640.0;
static const float SCORE_THRESHOLD = 0.5;
static const float NMS_THRESHOLD = 0.45;
static const float CONFIDENCE_THRESHOLD = 0.45;

// Text parameters.
static const float FONT_SCALE = 0.7;
static const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
static const int THICKNESS = 1;

// Colors.
static Scalar BLACK = Scalar(0, 0, 0);
static Scalar BLUE = Scalar(255, 178, 50);
static Scalar YELLOW = Scalar(0, 255, 255);
static Scalar RED = Scalar(0, 0, 255);

// boxes
detection_t *det_boxes;

// Draw the predicted bounding box.
void draw_label(Mat &input_image, string label, int left, int top)
{
    // Display the label at the top of the bounding box.
    int baseLine;
    Size label_size = getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    top = max(top, label_size.height);
    // Top left corner.
    Point tlc = Point(left, top);
    // Bottom right corner.
    Point brc = Point(left + label_size.width, top + label_size.height + baseLine);
    // Draw black rectangle.
    rectangle(input_image, tlc, brc, BLACK, FILLED);
    // Put the label on the black rectangle.
    putText(input_image, label, Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}

vector<Mat> pre_process(Mat &input_image, Net &net)
{
    // Convert to blob.
    Mat blob;
    blobFromImage(input_image, blob, 1. / 255., Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);

    net.setInput(blob);

    // Forward propagate.
    vector<Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    return outputs;
}

Mat post_process(Mat &input_image, Mat &backup_image, vector<Mat> &outputs, const vector<string> &class_name)
{
    // frame cnt
    static int frame_cnt = 0;

    // Initialize vectors to hold respective outputs while unwrapping detections.
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;

    // Resizing factor.
    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;

    float *data = (float *)outputs[0].data;

    // const int dimensions = 85;
    const int rows = 25200;
    // Iterate through 25200 detections.
    for (int i = 0; i < rows; ++i)
    {
        float confidence = data[4];
        // Discard bad detections and continue.
        if (confidence >= CONFIDENCE_THRESHOLD)
        {
            float *classes_scores = data + 5;
            // Create a 1x85 Mat and store class scores of 80 classes.
            Mat scores(1, class_name.size(), CV_32FC1, classes_scores);
            // Perform minMaxLoc and acquire index of best class score.
            Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            // Continue if the class score is above the threshold.
            if (max_class_score > SCORE_THRESHOLD)
            {
                // Store class ID and confidence in the pre-defined respective vectors.

                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                // Center.
                float cx = data[0];
                float cy = data[1];
                // Box dimension.
                float w = data[2];
                float h = data[3];
                // Bounding box coordinates.
                int left = int((cx - 0.5 * w) * x_factor);
                int top = int((cy - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                // Store good detections in the boxes vector.
                boxes.push_back(Rect(left, top, width, height));
            }
        }
        // Jump to the next column.
        data += 85;
    }

    // Perform Non Maximum Suppression and draw predictions.
    vector<int> indices;
    NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
    for (size_t i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        Rect box = boxes[idx];

        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;
        // Draw bounding box.
        rectangle(backup_image, Point(left, top), Point(left + width, top + height), BLUE, 3 * THICKNESS);

        // Get the label for the class name and its confidence.
        string label = format("%.2f", confidences[idx]);
        label = class_name[class_ids[idx]] + ":" + label;
        // Draw class labels.
        draw_label(backup_image, label, left, top);
    }

    uint32_t j = 0;
    for (uint32_t i = 0; i < indices.size() && j < tk_max_tracks; i++)
    {
        int idx = indices[i];
        if (class_ids[idx] == 0)
        {
            Rect box = boxes[idx];
            det_boxes[j].x1 = box.x;
            det_boxes[j].y1 = box.y;
            det_boxes[j].x2 = box.x + box.width;
            det_boxes[j].y2 = box.y + box.height;
            ++j;
        }
    }

    // tracking
    index_t det_num = j;

    customer_t *tracks = tk_get_tracks();

    if (++frame_cnt == 1)
    {
        tk_create_new_track(det_boxes, det_num);
    }
    else
    {
        tk_predict();
        tk_update(det_boxes, det_num);
    }

    // draw
    // tracks = tk_get_tracks();
    for (index_t i = 0; i < tk_get_track_num(); i++)
    {
        QELEM_T tlbr = to_tlbr(tracks[i].statemean);

        // Draw bounding box.
        rectangle(input_image, Point(tlbr.e1, tlbr.e2), Point(tlbr.e3, tlbr.e4), BLUE, 3 * THICKNESS);
        // printf("[frame %d]customer found at x1=%lf y1=%lf x2=%lf y2=%lf\n", frame_cnt, tlbr.e1, tlbr.e2, tlbr.e3, tlbr.e4);

        // Draw class labels.
        draw_label(input_image, to_string(tracks[i].id), tlbr.e1, tlbr.e2);
    }
    string label = format("current tracks: %d", tk_get_track_num());
    string label_d = format("current detection: %d", det_num);
    string label_f = format("current frame: %u", frame_cnt);
    int baseLine;
    Size label_size = getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    putText(input_image, label, Point(input_image.rows - label_size.width + 1, label_size.height), FONT_FACE, FONT_SCALE, RED);
    putText(input_image, label_d, Point(input_image.rows - label_size.width + 1, label_size.height << 1), FONT_FACE, FONT_SCALE, RED);
    putText(input_image, label_f, Point(input_image.rows - label_size.width + 1, label_size.height * 3), FONT_FACE, FONT_SCALE, RED);


    return input_image;
}

void det_init()
{
    tk_init();
    det_boxes = (detection_t *)malloc(tk_max_tracks * sizeof(detection_t));
}

void det_destroy()
{
    tk_destroy();
    free(det_boxes);
}