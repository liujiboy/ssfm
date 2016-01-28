//
//  quaternion.hpp
//  stereorectify
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef quaternion_hpp
#define quaternion_hpp
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#define FULLQUATSZ     4
/* unit quaternion from vector part */
#define _MK_QUAT_FRM_VEC(q, v){                                     \
(q)[1]=(v)[0]; (q)[2]=(v)[1]; (q)[3]=(v)[2];                      \
(q)[0]=sqrt(1.0 - (q)[1]*(q)[1] - (q)[2]*(q)[2]- (q)[3]*(q)[3]);  \
}
/*
    四元数乘法
 */
void quatMultFast(double q1[FULLQUATSZ], double q2[FULLQUATSZ], double p[FULLQUATSZ]);
/* convert a vector of camera parameters so that rotation is represented by
 * the vector part of the input quaternion. The function converts the
 * input quaternion into a unit one with a non-negative scalar part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 */
void quat2vec(double *inp, int nin, double *outp, int nout);
/* convert a vector of camera parameters so that rotation is represented by
 * a full unit quaternion instead of its input 3-vector part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 */
void vec2quat(double *inp, int nin, double *outp, int nout);
/*
    选择矩阵变四元数
 */
void rotmat2quat(double *R,double *r);
Mat_<double> rotmat2quat(Mat_<double> R);
/*
    四元数变旋转矩阵
 */
void quat2rotmat(double *r,double *R);
/*
    投影矩阵变四元数表示
 */
void cammat2quat(const Mat_<double>&p,double*r,double*t);
/*
    四元数表示变投影矩阵
 */
void quat2cammat(double*r,double*t,Mat_<double>&p);

#endif /* quaternion_hpp */
