//
//  IOTools.cpp
//  matching
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "IOTools.hpp"
#include <opencv2/opencv.hpp>
#include "stringtools.hpp"
using namespace cv;
//using namespace stringtools;
namespace utils
{
    void loadMatrix(string fileName,Mat& m)
    {
        vector<double> vals;
        int rows=0;
        ifstream in(fileName);
        string line;
        getline(in, line);
        while (in.good()) {
            rows++;
            stringstream ss;
            ss<<line;
            double v;
            while(ss.good())
            {
                ss>>v;
                // cout<<v<<" ";
                vals.push_back(v);
            }
            // cout<<endl;
            getline(in, line);
        }
        in.close();
        int cols=(int)vals.size()/rows;
        
        m.create(rows,cols,CV_64F);
        for (int i=0; i<rows; i++) {
            for(int j=0;j<cols;j++)
            {
                m.at<double>(i,j)=vals[i*cols+j];
            }
        }
    }
    void loadImages(string dirName,vector<Mat>& images,string sep)
    {
        Directory directory;
        vector<string> fileNames=directory.GetListFiles(dirName);
        for (vector<string>::iterator i=fileNames.begin(); i!=fileNames.end(); i++) {
            if(endWith(toUpperCase(*i),".JPG")||endWith(toUpperCase(*i),".PNG"))
                images.push_back(imread(dirName+sep+*i));
        }
    }
}
