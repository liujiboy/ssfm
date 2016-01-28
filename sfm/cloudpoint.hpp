//
//  CloudPoint.h
//  sfm
//
//  Created by  刘骥 on 15/4/18.
//  Copyright (c) 2015年  刘骥. All rights reserved.
//

#ifndef __sfm__CloudPoint__
#define __sfm__CloudPoint__

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
//用于表示三维点云点，其结构如下：
//x y z (x1,y1) (x2,y2) (x3,y3)
class CloudPoint {
private:
    vector<Point2d> matchPoints;
public:
    double x;
    double y;
    double z;
    int nframes; //视图数量
    bool known;  //是否已知
    CloudPoint(double x,double y,double z,int nframes);
    CloudPoint(int nframes);
    /*
        返回frame视图对应的2d点坐标
        p返回2d点坐标
        返回值-1表示没有找到对应的2d点
     */
    bool getPointInFrame(int frame,Point2d&p) const;
    /*
        设置frame视图对应的2d点坐标
     */
    void setPointInFrame(int frame,Point2d p);
    
    //重载<<运算符
    friend ostream&operator<<(ostream&out,const CloudPoint&cp);
};

#endif /* defined(__sfm__CloudPoint__) */
