//
//  Cloud.h
//  sfm
//
//  Created by  刘骥 on 15/4/20.
//  Copyright (c) 2015年  刘骥. All rights reserved.
//

#ifndef __sfm__Cloud__
#define __sfm__Cloud__

#include <iostream>
#include <fstream>
#include <vector>
#include "cloudpoint.hpp"
#include <opencv2/opencv.hpp>
#include "cvtools.hpp"
using namespace cv;
using namespace std;
using namespace utils;
//此类描述所有的三维点
class Cloud{
private:
    //点云点
    vector<CloudPoint> cloudPoints;
    /*
        将视图frame中的特征点加入到Cloud
        keypoints[i]:视图i对应的特征点
        matches[i][j]:视图i和视图j匹配的特征点
     */
    void addFrame(int frame,const vector<vector<Point2d> >& keypoints,vector<vector<vector<DMatch> > > &matches);
    /*
        重投影误差
        frame：视图编号
        cameraMatrix：相机矩阵
        pmat：投影矩阵
        objectPoints：返回frame视图对应的3D点
        imagePoints：返回frame视图对应的2D点
        projectedPoints：返回重投影后的2D点
        idxvec：返回frame视图对应的CloudPoint
        errors：返回每个3D点对应的重投影误差
     */
    double reprojectionError(int frame,const Mat&cameraMatrix,const Mat&pmat,vector<Point3d>&objectPoints,vector<Point2d>&imagePoints,vector<Point2d>&projectedPoints,vector<int>&idxvec,vector<double>&errors);
public:
    /*
        frame：视图编号
        keypoints[i]:视图i对应的特征点
        matches[i][j]:视图i和视图j匹配的特征点
     */
    Cloud(const vector<vector<Point2d> >& keypoints,vector<vector<vector<DMatch> > > &matches,int nframes);
    /*
        返回视图frame1和frame2的匹配点（且匹配点对应的3D点未知）
        point1：frame1对应的2D点
        point2：frame2对应的2D点
        idxvec[i]：points1[i],points2[i]d对应的CloudPoint的编号
     */
    void findUnKnowPoints(int frame1,int frame2,vector<Point2d>& points1,vector<Point2d>&points2,vector<int>&idxvec);
    /*
        更新点云
        points：3D点
        idxvec[i]：CloudPoint的编号
        mask[i]：true表示可以更新对应的CloudPoint
     */
    void updatePoints(const vector<Point3d>& points,const vector<int>& idxvec,const vector<bool>& mask);
    /*
        对frame视图进行过滤，清除重投影误差过大的点
        knowFramevec[i]：已重建的视图编号
        cameraMatrix：相机矩阵
        pmat：投影矩阵
     */
    void filter(int frame,const vector<int>& knowFramevec,const Mat&cameraMatrix,const Mat&pmat);
    /*
        删除离群点(此处的算法可以优化)
     */
    void deleteOutliers();
    
    /*
        增加点
     */
    void addPoint(const CloudPoint&cp);
    /*
        打印knowFramevec所含视图的重投影误差
        cameraMatrix：相机矩阵
        pmatrices[i]：视图i对应的投影矩阵
     */
    void printReprojectionError(const vector<int>&knowFramevec,const Mat&cameraMatrix,const vector<Mat> &pmatrices);
    /*
        绘制knowFramevec所含视图及其特征点、重投影点
        cameraMatrix：相机矩阵
        pmatrices[i]：视图i对应的投影矩阵
        images[i]：视图i对应的照片
     */
    void plotCloudPoints(const vector<int>&knowFramevec,const Mat&cameraMatrix,const vector<Mat> &pmatrices,const vector<Mat>&images);
    /*
        保存CloudPoint点到文件
     */
    void saveCloudPoints(const char* fileName);
    /*
        保存CloudPoint点到ply文件
     */
    void saveCloudPointsToPly(const vector<Mat>&images,const char* fileName);
   
    /*
        获取CloudPoint数量
     */
    int getPointSize();
    /*
        返回已知点的索引（known==true的点）
     */
    void getKnownPoints(vector<int>& knownIdxvec);
    /*
        获取CloudPoint
     */
    CloudPoint&getPoint(int i);
    /*
        选择下一个重建视图
        knowFramevec：已经重建的视图
        nframes：视图总量
     */
    int chooseNextFrame(const vector<int>&knowFramevec,int nframes);
    /*
        用已知点估计frame视图的投影矩阵
        p：返回投影矩阵
     */
    void findPmatrixByKnown(int frame,const Mat&cameraMatrix,Mat&p);
    /*
        输出运算符重载
     */
    friend ostream&operator<<(ostream&out,const Cloud&cloud);
};
#endif /* defined(__sfm__Cloud__) */
