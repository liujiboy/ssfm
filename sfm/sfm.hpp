//
//  SFM.h
//  sfm
//
//  Created by  刘骥 on 15/4/21.
//  Copyright (c) 2015年  刘骥. All rights reserved.
//

#ifndef __sfm__SFM__
#define __sfm__SFM__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "cloud.hpp"
#include "readparams.h"
#include "imgproj.h"
#include "sba.h"
#include <math.h>
using namespace cv;
using namespace std;
//Structure From Motion
class SFM{
private:
    
    Mat cameraMatrix;//相机矩阵
    vector<Mat>pmatrices;//相机投影矩阵，pmatrices[i]表示视图i的投影矩阵
    vector<vector<Point2d> > keypoints;//keypoints[i][j]表示第i视图第j个特征点
    vector<Mat> descriptors; //descriptors[i]表示第i视图特征点的描述
    vector<vector<Mat> > fmatrices; //fmatrices[i][j]表示视图i和视图j间的基础矩阵
    vector<vector<vector<DMatch> > > matches;//matches[i][j]表示视图i和视图j的特征点匹配
    vector<int> knowFramevec;//已重建视图索引
    
    int nframes;//视图数量
    
    Cloud *cloud;//重建的点云
    /*
        计算视图特征
     */
    void computeFeatures(const vector<Mat>&images);
    /*
        计算视图匹配
     */
    void pairwiseMatch();
    /*
     初始重建
     */
    void initialReconstruct();
    /*
        N视图bundle ajdustment
        nconstframes：几个视图的投影矩阵固定不变，默认第1个视图投影矩阵不变
        nconstpts3D：几个三维点的坐标保持不变
        maxiter：最大迭代次数
        verbose：调试信息
     */
    void sba(int nconstframes=0,int nconstpts3D=0,int maxiter=250,int verbose=0);
public:
    SFM(Mat cameraMatrix,const vector<Mat>& images);
    /*
        重建其他视图
     */
    void reconstruct();
    /*
        绘制结果
     */
    void plotCloudPoints(const vector<Mat>&images);
    /*
        保存结果
     */
    void saveCloudPointsToPly(const vector<Mat>&images,const char* fileName);
    
    /*
        返回视图i的投影矩阵
     */
    Mat getPMatrix(int i);
    /*
        保存投影矩阵
     */
    void savePmatrix(const string& fileName,int i);
    ~SFM();
};
#endif /* defined(__sfm__SFM__) */
