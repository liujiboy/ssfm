//
//  IOTools.hpp
//  matching
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef IOTools_hpp
#define IOTools_hpp
#include <iostream>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
namespace utils
{
    /*
        加载矩阵
     */
    void loadMatrix(string fileName,Mat& m);
    /*
        加载dirName目录中的图像
     */
    void loadImages(string dirName,vector<Mat>& images,string sep="/");
}
#endif /* IOTools_hpp */
