//
//  main.cpp
//  matching
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "iotools.hpp"
#include "sfm.hpp"
using namespace utils;
using namespace cv;
int main(int argc, const char * argv[]) {
    //图片路径
    string dirName="images";
    vector<Mat> images;
    loadImages(dirName, images);
    Mat cameraMatrix;
    loadMatrix("images/K.txt", cameraMatrix);
    cout<<cameraMatrix<<endl;
    SFM sfm(cameraMatrix,images);
    sfm.reconstruct();
    sfm.plotCloudPoints(images);
    sfm.saveCloudPointsToPly(images, "a.ply");

    return 0;
}
