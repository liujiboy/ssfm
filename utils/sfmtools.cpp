//
//  sfmtools.cpp
//  matching
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "sfmtools.hpp"
namespace utils{
    void triangleatePoints(const Mat&p1,const Mat&p2,const Mat&cameraMatrix,const vector<Point2d>&points1,const vector<Point2d>&points2,vector<Point3d>&points3d,vector<bool>&frontMask,int&frontCount)
    {
        Mat_<double> pointsMat;
        //三角化
        triangulatePoints(cameraMatrix*p1, cameraMatrix*p2, points1, points2, pointsMat);
        //非齐次坐标化
        for(int col=0;col<pointsMat.cols;col++)
        {
            double x=pointsMat(0,col)/pointsMat(3,col);
            double y=pointsMat(1,col)/pointsMat(3,col);
            double z=pointsMat(2,col)/pointsMat(3,col);
            pointsMat(0,col)=x;
            pointsMat(1,col)=y;
            pointsMat(2,col)=z;
            pointsMat(3,col)=1;
            points3d.push_back(Point3d(x,y,z));
        }
        //重新投影到2d
       // cout<<pointsMat<<endl;
        Mat_<double> points3d_projected1=p1*pointsMat;
        Mat_<double> points3d_projected2=p2*pointsMat;
        //统计在P1和P2两个相机前面的点数量
        frontCount=0;
        for(int colIdx=0;colIdx<points3d_projected1.cols;colIdx++)
        {

            if(points3d_projected1(2,colIdx)>0&&points3d_projected2(2,colIdx)>0)
            {
                
                frontMask[colIdx]=true;
                frontCount++;
            }else{
                frontMask[colIdx]=false;
            }
        }
        /*Point3d center(0,0,0); //中心
        int psize=0;
        for (int i=0; i<points3d.size(); i++) {
            //计算有效点的中心
            if (frontMask[i]) {
                center.x+=points3d[i].x;
                center.y+=points3d[i].y;
                center.z+=points3d[i].z;
                psize++;
            }
            
        }
        //计算中心
        center.x/=psize;
        center.y/=psize;
        center.z/=psize;
        
        //统计有效点距离中心的平均距离
        double avgd=0;
        vector<double> distances(points3d.size(),-1);
        for (int i=0; i<points3d.size();i++) {
            if(frontMask[i])
            {
                Point3d p=points3d[i];
                double dx=center.x-p.x;
                double dy=center.y-p.y;
                double dz=center.z-p.z;
                distances[i]=sqrt(dx*dx+dy*dy+dz*dz);
                avgd+=distances[i];
            }
        }
        avgd/=psize;
        //移除距离中心太远的点
        for (int i=0; i<points3d.size();i++) {
            if(frontMask[i])
            {
                if(distances[i]/avgd>1)
                    frontMask[i]=false;
            }
        }*/
        

    }
    void trianglulateAndFindPMatrix(const vector<Point2d>&points1,const vector<Point2d>&points2,const Mat&P1,const Mat&F,const Mat&cameraMatrix,Mat&P2,vector<Point3d>&points3d,vector<bool>&frontMask)
    {
        Mat E=cameraMatrix.t()*F*cameraMatrix;
        Mat pvec[4];
        computePMatFromE(E,pvec);
        int maxCount=0;
        int bestIdx=0;
        vector<Point3d> points3dVector[4];
        //vector<int> frontIdxvecarr[4];
        vector<vector<bool> > frontMaskarr(4,vector<bool>(frontMask.size(),false));
        for(int i=0;i<4;i++)
        {
            int frontCount=0;
            triangleatePoints(P1, pvec[i], cameraMatrix, points1, points2, points3dVector[i], frontMaskarr[i],frontCount);
            if(frontCount>maxCount)
            {
                maxCount=frontCount;
                bestIdx=i;
            }
        }
        //拷贝返回值
        pvec[bestIdx].copyTo(P2);
        frontMask=frontMaskarr[bestIdx];
        points3d=points3dVector[bestIdx];
        
        /*Point3d center(0,0,0); //中心
        int psize=0;
        for (int i=0; i<points3d.size(); i++) {
            //计算有效点的中心
            if (frontMask[i]) {
                center.x+=points3d[i].x;
                center.y+=points3d[i].y;
                center.z+=points3d[i].z;
                psize++;
            }
           
        }
        //计算中心
        center.x/=psize;
        center.y/=psize;
        center.z/=psize;
        
        //统计有效点距离中心的平均距离
        double avgd=0;
        vector<double> distances(points3d.size(),-1);
        for (int i=0; i<points3d.size();i++) {
            if(frontMask[i])
            {
                Point3d p=points3d[i];
                double dx=center.x-p.x;
                double dy=center.y-p.y;
                double dz=center.z-p.z;
                distances[i]=sqrt(dx*dx+dy*dy+dz*dz);
                avgd+=distances[i];
            }
        }
        avgd/=psize;
        //移除距离中心太远的点
        for (int i=0; i<points3d.size();i++) {
            if(frontMask[i])
            {
                if(distances[i]/avgd>1)
                    frontMask[i]=false;
            }
        }*/

    }

void computePMatFromE(const Mat&E,Mat *pvec)
    {
        
        Mat u,s,vt;
        SVD::compute(E, s, u, vt);
       // SVD::compute(u*Mat(Matx33d(1,0,0,0,1,0,0,0,0))*vt,s, u, vt);
        if(determinant(u*vt)<0)
        {
            vt=-vt;
        }
        
        Mat w( Matx33d(0,-1,0,1,0,0,0,0,1));
        Mat u3=u.col(2);
        Mat P1,P2,P3,P4;
        hconcat(u*w*vt, u3,P1);
        hconcat(u*w*vt, -u3,P2);
        hconcat(u*w.t()*vt, u3,P3);
        hconcat(u*w.t()*vt, -u3,P4);
        pvec[0]=P1;
        pvec[1]=P2;
        pvec[2]=P3;
        pvec[3]=P4;
    }
    //用两个视图的特征点与特征描述计算基础矩阵，并用ransac算法产生的mask获取更好的特征匹配
    //keypoints1：视图1的特征点
    //keypoints2：视图2的特征点
    //descriptor1：视图1的特征描述
    //descriptor2：视图2的特征描述
    //refinedMatch：返回更好的匹配
    Mat computeFundamentalMatAndMatch(const vector<Point2d>& keypoints1,const vector<Point2d>& keypoints2,const Mat& descriptor1,const Mat& descriptor2,vector<DMatch>&refinedMatch)
    {
        //求得初始匹配(ratioTest）
        BFMatcher bfMatcher;
        vector<vector<DMatch> > matches;
        vector<DMatch> initialMatch;
        bfMatcher.knnMatch(descriptor1, descriptor2, matches, 2);
        for(vector<vector<DMatch> >::iterator it=matches.begin(); it!=matches.end();it++)
        {
            DMatch m1=(*it)[0];
            DMatch m2=(*it)[1];
            //两个最近匹配的distance比值小于0.8
            if(m1.distance/m2.distance<0.8)
                initialMatch.push_back(m1);
        }
        
        //计算基础矩阵
        vector<Point2d> points1;
        vector<Point2d> points2;
        for(vector<DMatch>::iterator it=initialMatch.begin();it!=initialMatch.end();it++)
        {
            points1.push_back(keypoints1[it->queryIdx]);
            points2.push_back(keypoints2[it->trainIdx]);
        }
        vector<uchar> mask;
        Mat F=cv::findFundamentalMat(points1,points2,FM_RANSAC,1,0.99,mask);
        //用ransac算法产生的mask获取更好的特征匹配
        for(vector<DMatch>::size_type i=0;i<initialMatch.size();i++)
        {
            if(mask[i])
                refinedMatch.push_back(initialMatch[i]);
        }
        return F;
    }
    void getMatchPoints(const vector<Point2d>& keypoints1,const vector<Point2d>& keypoints2,const vector<DMatch>&matches,vector<Point2d>&points1,vector<Point2d>&points2)
    {
        
        for(vector<DMatch>::size_type i=0;i<matches.size();i++)
        {
            DMatch match=matches[i];
            Point2d point1=keypoints1[match.queryIdx];
            Point2d point2=keypoints2[match.trainIdx];
            points1.push_back(point1);
            points2.push_back(point2);
        }
    }
}