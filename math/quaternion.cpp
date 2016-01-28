//
//  quaternion.cpp
//  stereorectify
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "quaternion.hpp"
void quatMultFast(double q1[FULLQUATSZ], double q2[FULLQUATSZ], double p[FULLQUATSZ])
{
    double t1, t2, t3, t4, t5, t6, t7, t8, t9;
    //double t10, t11, t12;
    
    t1=(q1[0]+q1[1])*(q2[0]+q2[1]);
    t2=(q1[3]-q1[2])*(q2[2]-q2[3]);
    t3=(q1[1]-q1[0])*(q2[2]+q2[3]);
    t4=(q1[2]+q1[3])*(q2[1]-q2[0]);
    t5=(q1[1]+q1[3])*(q2[1]+q2[2]);
    t6=(q1[1]-q1[3])*(q2[1]-q2[2]);
    t7=(q1[0]+q1[2])*(q2[0]-q2[3]);
    t8=(q1[0]-q1[2])*(q2[0]+q2[3]);
    
#if 0
    t9 =t5+t6;
    t10=t7+t8;
    t11=t5-t6;
    t12=t7-t8;
    
    p[0]= t2 + 0.5*(-t9+t10);
    p[1]= t1 - 0.5*(t9+t10);
    p[2]=-t3 + 0.5*(t11+t12);
    p[3]=-t4 + 0.5*(t11-t12);
#endif
    
    /* following fragment it equivalent to the one above */
    t9=0.5*(t5-t6+t7+t8);
    p[0]= t2 + t9-t5;
    p[1]= t1 - t9-t6;
    p[2]=-t3 + t9-t8;
    p[3]=-t4 + t9-t7;
}
/* convert a vector of camera parameters so that rotation is represented by
 * the vector part of the input quaternion. The function converts the
 * input quaternion into a unit one with a non-negative scalar part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 */
void quat2vec(double *inp, int nin, double *outp, int nout)
{
    double mag, sg;
    int i;
    
    /* intrinsics & distortion */
    if(nin>7) // are they present?
        for(i=0; i<nin-7; ++i)
            outp[i]=inp[i];
    else
        i=0;
    
    /* rotation */
    /* normalize and ensure that the quaternion's scalar component is non-negative;
     * if not, negate the quaternion since two quaternions q and -q represent the
     * same rotation
     */
    mag=sqrt(inp[i]*inp[i] + inp[i+1]*inp[i+1] + inp[i+2]*inp[i+2] + inp[i+3]*inp[i+3]);
    sg=(inp[i]>=0.0)? 1.0 : -1.0;
    mag=sg/mag;
    outp[i]  =inp[i+1]*mag;
    outp[i+1]=inp[i+2]*mag;
    outp[i+2]=inp[i+3]*mag;
    i+=3;
    
    /* translation*/
    for( ; i<nout; ++i)
        outp[i]=inp[i+1];
}

/* convert a vector of camera parameters so that rotation is represented by
 * a full unit quaternion instead of its input 3-vector part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 */
void vec2quat(double *inp, int nin, double *outp, int nout)
{
    double *v, q[FULLQUATSZ];
    int i;
    
    /* intrinsics & distortion */
    if(nin>7-1) // are they present?
        for(i=0; i<nin-(7-1); ++i)
            outp[i]=inp[i];
    else
        i=0;
    
    /* rotation */
    /* recover the quaternion from the vector */
    v=inp+i;
    _MK_QUAT_FRM_VEC(q, v);
    outp[i]  =q[0];
    outp[i+1]=q[1];
    outp[i+2]=q[2];
    outp[i+3]=q[3];
    i+=FULLQUATSZ;
    
    /* translation */
    for( ; i<nout; ++i)
        outp[i]=inp[i-1];
}
//r表示四元数的后三位
void rotmat2quat(double *R,double *r){
    double q[4];
    q[0]=sqrt(1.0 + R[0] + R[4] + R[8])*0.5;
    q[1]=(R[7] - R[5])/(4.0*q[0]);
    q[2]=(R[2] - R[6])/(4.0*q[0]);
    q[3]=(R[3] - R[1])/(4.0*q[0]);
    
    r[0]=q[0];
    r[1]=q[1];
    r[2]=q[2];
    r[3]=q[3];
}
Mat_<double> rotmat2quat(Mat_<double> R)
{
    Mat_<double> q(4,1);
    R=R.reshape(0,9);
    q(0)=sqrt(1.0 + R(0) + R(4) + R(8))*0.5;
    q(1)=(R(7) - R(5))/(4.0*q(0));
    q(2)=(R(2) - R(6))/(4.0*q(0));
    q(3)=(R(3) - R(1))/(4.0*q(0));
    return q;
}
//r表示四元数的后三位
void quat2rotmat(double *r,double *R)
{   double q1=r[0];
    double q2=r[1];
    double q3=r[2];
    double q0=sqrt(1-q1*q1-q2*q2-q3*q3);
    R[0]=q0*q0+q1*q1-q2*q2-q3*q3;
    R[1]=2*(q1*q2-q0*q3);
    R[2]=2*(q1*q3+q0*q2);
    
    R[3]=2*(q1*q2+q0*q3);
    R[4]=q0*q0+q2*q2-q1*q1-q3*q3;
    R[5]=2*(q2*q3-q0*q1);
    
    R[6]=2*(q1*q3-q0*q2);
    R[7]=2*(q2*q3+q0*q1);
    R[8]=q0*q0+q3*q3-q1*q1-q2*q2;
}
void cammat2quat(const Mat_<double>&p,double*r,double*t)
{
    double R[9]={p(0,0),p(0,1),p(0,2),p(1,0),p(1,1),p(1,2),p(2,0),p(2,1),p(2,2)};
    rotmat2quat(R, r);
    t[0]=p(0,3);
    t[1]=p(1,3);
    t[2]=p(2,3);
}
void quat2cammat(double*r,double*t,Mat_<double>&p)
{
    double R[9];
    quat2rotmat(r,R);
    p(0,0)=R[0];
    p(0,1)=R[1];
    p(0,2)=R[2];
    p(1,0)=R[3];
    p(1,1)=R[4];
    p(1,2)=R[5];
    p(2,0)=R[6];
    p(2,1)=R[7];
    p(2,2)=R[8];
    p(0,3)=t[0];
    p(1,3)=t[1];
    p(2,3)=t[2];
    
}
