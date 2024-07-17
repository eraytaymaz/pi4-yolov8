// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2020 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

//modified 1-14-2023 Q-engineering

#include "yoloV8.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>

#include <filesystem>
#include <string>

YoloV8 yolov8;
int target_size = 640; //416; //320;  must be divisible by 32.


int main(int argc, char** argv)
{
    /*const char* imagepath = argv[1];

    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s [imagepath]\n", argv[0]);
        return -1;
    }

    cv::Mat m = cv::imread(imagepath, 1);
    if (m.empty())
    {
        fprintf(stderr, "cv::imread %s failed\n", imagepath);
        return -1;
    }

    yolov8.load(target_size);       //load model (once) see yoloyV8.cpp line 246

    std::string directory_path = "data"; // Replace with your directory path


    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (entry.is_regular_file()) {
            std::cout << entry.path() << std::endl;
        }
        cv::Mat m = cv::imread(entry.path(), 1);

        std::vector<Object> objects;
        yolov8.detect(m, objects);      //recognize the objects
        yolov8.draw(m, objects);        //show the outcome

        cv::imshow("RPi4 - 1.95 GHz - 2 GB ram",m);
    //    cv::imwrite("out.jpg",m);
        cv::waitKey(0);
        */
    float f;
    float FPS[16];
    int i,Fcnt=0;
    cv::Mat frame;
    //some timing
    std::chrono::steady_clock::time_point Tbegin, Tend;

    for(i=0;i<16;i++) FPS[i]=0.0;

    //yolov8.init(false); //we have no GPU

    //yolov8.loadModel("yolo-fastestv2-opt.param","yolo-fastestv2-opt.bin");

    cv::VideoCapture cap("James.mp4");
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Unable to open the camera" << std::endl;
        return 0;
    }
    yolov8.load(target_size);
    std::cout << "Start grabbing, press ESC on Live window to terminate" << std::endl;
	while(1){
//        frame=cv::imread("000139.jpg");  //need to refresh frame before dnn class detection
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Unable to grab from the camera" << std::endl;
            break;
        }

        Tbegin = std::chrono::steady_clock::now();

        std::vector<Object> objects;
        yolov8.detect(frame, objects);      //recognize the objects
        yolov8.draw(frame, objects);
        Tend = std::chrono::steady_clock::now();

        //calculate frame rate
        f = std::chrono::duration_cast <std::chrono::milliseconds> (Tend - Tbegin).count();
        if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
        for(f=0.0, i=0;i<16;i++){ f+=FPS[i]; }
        putText(frame, cv::format("FPS %0.2f", f/16),cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX,0.6, cv::Scalar(0, 0, 255));

        //show outputstd::cerr << "ERROR: Unable to grab from the camera" << std::endl;
        cv::imshow("Jetson Nano",frame);
        //cv::imwrite("test.jpg",frame);
        char esc = cv::waitKey(5);
        if(esc == 27) break;

    }
    return 0;
}
