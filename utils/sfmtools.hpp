//
//  sfmtools.hpp
//  matching
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef sfmtools_hpp
#define sfmtools_hpp

#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
namespace utils{
    /*
        从本质矩阵E中重建投影矩阵
        E：本质矩阵
        pvec：返回的4个可能投影矩阵
     */
    void computePMatFromE(const Mat&E,Mat *pvec);
    /*
        三角化并返回投影矩阵
        points1,points2：匹配的特征点
        p1：points1对应的投影矩阵
        F：基础矩阵
        cameraMatrix：相机矩阵
        p2：返回的points2对应的投影矩阵
        points3d：返回的重建的3d点
        frontMask[i]：返回值，表示points3d[i]是否同时位于p1和p2两个相机之前
     */
    void trianglulateAndFindPMatrix(const vector<Point2d>&points1,const vector<Point2d>&points2,const Mat&p1,const Mat&F,const Mat&cameraMatrix,Mat&p2,vector<Point3d>&points3d,vector<bool>&frontMask);
    /*
        三角化
        points1,points2：匹配的特征点
        p1，p2：投影矩阵
        points3d：返回的重建的3d点
        frontMask[i]：返回值，表示points3d[i]是否同时位于p1和p2两个相机之前
        frontCount：返回值，表示同时位于p1和p2两个相机之前的点数量
     */
    void triangleatePoints(const Mat&p1,const Mat&p2,const Mat&cameraMatrix,const vector<Point2d>&points1,const vector<Point2d>&points2,vector<Point3d>&points3d,vector<bool>&frontMask,int&frontCount);
    /*
        根据matches返回特征点points1和points2
     */
    void getMatchPoints(const vector<Point2d>& keypoints1,const vector<Point2d>& keypoints2,const vector<DMatch>&matches,vector<Point2d>&points1,vector<Point2d>&points2);
    /*
        用两个视图的特征点与特征描述计算基础矩阵，并用ransac算法产生的mask获取更好的特征匹配
        keypoints1：视图1的特征点
        keypoints2：视图2的特征点
        descriptor1：视图1的特征描述
        descriptor2：视图2的特征描述
        refinedMatch：返回更好的匹配
     */
    Mat computeFundamentalMatAndMatch(const vector<Point2d>& keypoints1,const vector<Point2d>& keypoints2,const Mat& descriptor1,const Mat& descriptor2,vector<DMatch>&refinedMatch);
}
#endif /* sfmtools_hpp */
