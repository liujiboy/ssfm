//
//  Cloud.cpp
//  sfm
//
//  Created by  刘骥 on 15/4/20.
//  Copyright (c) 2015年  刘骥. All rights reserved.
//

#include "cloud.hpp"
#include <algorithm>
using namespace std;
Cloud::Cloud(const vector<vector<Point2d> >& keypoints,vector<vector<vector<DMatch> > > &matches,int nframes)
{
    
    //创建初始点集合
    for (int i=0; i<nframes; i++) {
        for (int j=i+1; j<nframes; j++) {
            for (vector<DMatch>::iterator it=matches[i][j].begin(); it!=matches[i][j].end(); it++) {
                int a=it->queryIdx;
                int b=it->trainIdx;
                CloudPoint cp(nframes);
                cp.setPointInFrame(i, keypoints[i][a]);
                cp.setPointInFrame(j, keypoints[j][b]);
                addPoint(cp);
            }
        }
    }
    //合并点集合
    int m=0;
    for (vector<CloudPoint>::iterator it1=cloudPoints.begin(); it1!=cloudPoints.end();it1++,m++) {
    
        int n=0;
        for (vector<CloudPoint>::iterator it2=it1+1; it2!=cloudPoints.end();) {
            
            //测试it1和it2能否合并
            bool merged=false;
            for (int i=0; i<nframes; i++) {
                Point2d p1,p2;
                if (it1->getPointInFrame(i, p1)&&it2->getPointInFrame(i, p2)&&p1==p2) {
                    //cout<<m<<","<<n<<endl;
                    //可以合并，合并
                    for(int j=0;j<nframes;j++)
                    {
                        if (!it1->getPointInFrame(j, p1)&&it2->getPointInFrame(j, p2)) {
                            it1->setPointInFrame(j, p2);
                        }
                    }
                    //合并完毕删除it2
                    cloudPoints.erase(it2);
                    merged=true;
                    break;
                }
            }
            if (!merged) {
                it2++;
                n++;
                
            }
        }
    }
}
void Cloud::addPoint(const CloudPoint &cp)
{
    this->cloudPoints.push_back(cp);
}
void Cloud::printReprojectionError(const vector<int>&knowFramevec,const Mat&cameraMatrix,const vector<Mat> &pmatrices)
{
    for (int i=0; i<knowFramevec.size(); i++) {
        int frame=knowFramevec[i];
        vector<Point3d> objectPoints;
        vector<Point2d> imagePoints;
        vector<Point2d> projectedPoints;
        vector<int> idxvec;
        vector<double> errors;
        double error=reprojectionError(frame, cameraMatrix, pmatrices[frame], objectPoints,imagePoints, projectedPoints,idxvec, errors);
        cout<<"视图"<<frame<<"投影点数为："<<objectPoints.size()<<" 平均投影误差为："<<error<<endl;
    }
}
void Cloud::plotCloudPoints(const vector<int>&knowFramevec,const Mat&cameraMatrix,const vector<Mat> &pmatrices,const vector<Mat>&images)
{
    for (int i=0; i<knowFramevec.size(); i++) {
        int frame=knowFramevec[i];
        vector<Point3d> objectPoints;
        vector<Point2d> imagePoints;
        vector<Point2d> projectedPoints;
        vector<int> idxvec;
        vector<double> errors;
        reprojectionError(frame, cameraMatrix, pmatrices[frame], objectPoints,imagePoints,projectedPoints, idxvec, errors);
        Mat image=images[frame].clone();
        for (int j=0; j<imagePoints.size(); j++) {
            circle(image, imagePoints[j], 3, Scalar(0,0,255),-1);
            circle(image, projectedPoints[j], 3, Scalar(255,0,0),-1);
        }
        showImage("view:"+to_string(frame), image);
        waitKey();
    }
}
void Cloud::saveCloudPoints(const char *fileName)
{
    ofstream out(fileName);
   /* Mat_<double> m((int)cloudPoints.size(),3);
    for(int i=0;i<cloudPoints.size();i++)
    {
        m(i,0)=cloudPoints[i].x;
        m(i,1)=cloudPoints[i].y;
        m(i,2)=cloudPoints[i].z;
    }
    out<<cv::format(m, "csv");*/
    for(int i=0;i<cloudPoints.size();i++)
    {
        CloudPoint cp=cloudPoints[i];
        out<<i<<";"<<cp.x<<","<<cp.y<<","<<cp.z<<";";
        for (int j=0; j <cp.nframes; j++) {
            Point2d p;
            if (cp.getPointInFrame(j, p)) {
                out<<p.x<<","<<p.y<<";";
            }
            else
            {
                out<<"-1"<<";";
            }
        }
        out<<endl;
    }
    out.close();
}
void Cloud::saveCloudPointsToPly(const vector<Mat>&images,const char* fileName)
{
    ofstream out(fileName);
    out<<"ply"<<endl;
    out<<"format ascii 1.0"<<endl;
    out<<"element vertex "<<cloudPoints.size()<<endl;
    out<<"property float x"<<endl;
    out<<"property float y"<<endl;
    out<<"property float z"<<endl;
    out<<"property uchar diffuse_red"<<endl;
    out<<"property uchar diffuse_green"<<endl;
    out<<"property uchar diffuse_blue"<<endl;
    out<<"end_header"<<endl;
    for(int i=0;i<cloudPoints.size();i++)
    {
        const CloudPoint& cp=cloudPoints[i];
        if(cp.known)
        {
        out<<cp.x<<" "<<cp.y<<" "<<cp.z<<" ";
        for(int frame=0;frame<cp.nframes;frame++)
        {
            Point2d p;
            if(cp.getPointInFrame(frame,p))
            {
                Vec3b color=images[frame].at<Vec3b>(p.y,p.x);
                out<<(int)color[2]<<" "<<(int)color[1]<<" "<<(int)color[0]<<endl;
                break;
            }
        }
        }
    }
    out.close();
}
void Cloud::addFrame(int frame,const vector<vector<Point2d> >& keypoints,vector<vector<vector<DMatch> > > &matches)
{
    int nframes=(int)keypoints.size();
    for (int prevFrame=frame-1; prevFrame>=0; prevFrame--) {
        vector<DMatch> match=matches[prevFrame][frame];
        for (int i=0; i<match.size(); i++) {
            int a=match[i].queryIdx;
            int b=match[i].trainIdx;
            bool found=false;
            for(vector<CloudPoint>::iterator it=cloudPoints.begin();it!=cloudPoints.end();it++)
            {
                Point2d point2d;
                it->getPointInFrame(prevFrame,point2d);
                if (point2d==keypoints[prevFrame][a]) {
                    it->setPointInFrame(frame, keypoints[frame][b]);
                    found=true;
                    break;
                }
            }
            if (!found) {
//                if (keypoints[prevFrame][a].x==795.147) {
//                    cout<<a<<","<<b<<","<<keypoints[prevFrame][a]<<endl;
//                    cin.get();
//                }
                CloudPoint cp(nframes);
                cp.setPointInFrame(prevFrame,keypoints[prevFrame][a]);
                cp.setPointInFrame(frame,keypoints[frame][b]);
                this->addPoint(cp);
            }
        }
    }
}
void Cloud::findUnKnowPoints(int frame1,int frame2,vector<Point2d>& points1,vector<Point2d>&points2,vector<int>&idxvec)
{
    int idx=0;
    for(vector<CloudPoint>::iterator it=cloudPoints.begin();it!=cloudPoints.end();it++,idx++)
    {
        if (!it->known) {
            Point2d p1,p2;
            if(it->getPointInFrame(frame1,p1)&&it->getPointInFrame(frame2,p2))
            {
                points1.push_back(p1);
                points2.push_back(p2);
                idxvec.push_back(idx);
            }
        }
    }
}
void Cloud::updatePoints(const vector<Point3d>& points,const vector<int>& idxvec,const vector<bool>& mask)
{
    for (int i=0;i<idxvec.size();i++) {
        if(mask[i])
        {
            cloudPoints[idxvec[i]].x=points[i].x;
            cloudPoints[idxvec[i]].y=points[i].y;
            cloudPoints[idxvec[i]].z=points[i].z;
            cloudPoints[idxvec[i]].known=true;
        }
    }
}
double Cloud::reprojectionError(int frame,const Mat&cameraMatrix,const Mat&pmat,vector<Point3d>&objectPoints,vector<Point2d>&imagePoints,vector<Point2d>&projectedPoints,vector<int>&idxvec,vector<double>&errors)
{
    for (int i=0;i<cloudPoints.size();i++) {
        CloudPoint& cp=cloudPoints[i];
        if(cp.known)
        {
            Point2d p2d;
            if(cp.getPointInFrame(frame, p2d))
            {
                objectPoints.push_back(Point3d(cp.x,cp.y,cp.z));
                imagePoints.push_back(p2d);
                idxvec.push_back(i);
            }
        }
    }
    Mat rvec;
    Rodrigues(pmat(Rect(0,0,3,3)), rvec);
    Mat tvec(3,1,CV_64F);
    pmat(Rect(3,0,1,3)).copyTo(tvec);
    projectPoints(objectPoints, rvec, tvec, cameraMatrix, noArray(), projectedPoints);
    errors.resize(objectPoints.size());
    double avgError=0;
    for (int i=0; i<imagePoints.size(); i++) {
        double dx=imagePoints[i].x-projectedPoints[i].x;
        double dy=imagePoints[i].y-projectedPoints[i].y;
        errors[i]=sqrt(dx*dx+dy*dy);
        avgError+=errors[i];
    }
    avgError/=errors.size();
    return avgError;
}
void Cloud::filter(int frame,const vector<int>& knowFramevec,const Mat&cameraMatrix,const Mat&pmat)
{
    vector<Point3d> objectPoints;
    vector<Point2d> imagePoints;
    vector<Point2d> projectedPoints;
    vector<int> idxvec;
    vector<double> errors;
    double avgError=reprojectionError(frame, cameraMatrix, pmat, objectPoints, imagePoints,projectedPoints, idxvec, errors);
    cout<<"视图"<<frame<<"平均误差是"<<avgError<<endl;
    vector<bool> mask(objectPoints.size(),false);
    for (int i=0; i<errors.size(); i++) {
        if(errors[i]>2*avgError)
            mask[i]=true;
    }
    for (int i=0; i<idxvec.size(); i++) {
        if(mask[i])
        {
            CloudPoint& cp=cloudPoints[idxvec[i]];
            cp.setPointInFrame(frame, Point2d(-1,-1));
            bool unknown=true;
            int count=0;
            for (vector<int>::const_iterator it=knowFramevec.begin(); it!=knowFramevec.end(); it++) {
                Point2d p;
                if (cp.getPointInFrame(*it, p)) {
                    count++;
                }
            }
            if (count>=2) {
                unknown=false;
            }
            if(unknown)
            {
                cp.known=false;
            }
        }
    }
    
}
void Cloud::deleteOutliers()
{
    vector<bool> mask(cloudPoints.size(),false);
    Point3d center(0,0,0);
    int count=0;
    for(int i=0;i<cloudPoints.size();i++)
    {
        CloudPoint&cp =cloudPoints[i];
        if(cp.known)
        {
            center.x+=cp.x;
            center.y+=cp.y;
            center.z+=cp.z;
            count++;
        }
    }
    center.x/=count;
    center.y/=count;
    center.z/=count;
    vector<double> distances(cloudPoints.size(),0);
    double avgdistance=0;
    for(int i=0;i<cloudPoints.size();i++)
    {
        CloudPoint&cp =cloudPoints[i];
        if(cp.known)
        {
            double dx=cp.x-center.x;
            double dy=cp.y-center.y;
            double dz=cp.z-center.z;
            distances[i]=sqrt(dx*dx+dy*dy+dz*dz);
            avgdistance+=distances[i];
        }
    }
    avgdistance/=count;
    cout<<"平均距离"<<avgdistance<<endl;
    for(int i=0;i<cloudPoints.size();i++)
    {
        CloudPoint&cp =cloudPoints[i];
        if(cp.known)
        {
            if(distances[i]>2*avgdistance)
                mask[i]=true;
        }else{
            mask[i]=true;
        }
    }
    vector<CloudPoint> newCloudPoints;
    for(int i=0;i<cloudPoints.size();i++)
    {
        if (!mask[i]) {
            newCloudPoints.push_back(cloudPoints[i]);
        }
    }
    this->cloudPoints=newCloudPoints;
}
int Cloud::getPointSize()
{
    return (int)cloudPoints.size();
}
CloudPoint&Cloud::getPoint(int i)
{
    return cloudPoints[i];
}
void Cloud::getKnownPoints(vector<int>& knownIdxvec)
{
    for (int i=0; i<cloudPoints.size(); i++) {
        if (cloudPoints[i].known) {
            knownIdxvec.push_back(i);
        }
    }
}
int Cloud:: chooseNextFrame(const vector<int>&knowFramevec,int nframes)
{
    int nextFrame=0;
    int maxCount=0;
    for (int frame=0; frame<nframes; frame++) {
        
        vector<int>::const_iterator pos=find(knowFramevec.begin(),knowFramevec.end(),frame);
        if (pos==knowFramevec.end()) {
            int count=0;
            for (int i=0; i<cloudPoints.size(); i++) {
                CloudPoint cp=cloudPoints[i];
                Point2d p;
                if(cp.known&&cp.getPointInFrame(frame, p))
                {
                    count++;
                }
            }
            cout<<"视图"<<frame<<"已知点数为"<<count<<endl;
            if (count>maxCount) {
                maxCount=count;
                nextFrame=frame;
            }
            
        }
        
    }
    return nextFrame;
}
void Cloud::findPmatrixByKnown(int frame,const Mat&cameraMatrix,Mat&p)
{
    //根据已知的三位点确定投影矩阵
    //solvePnPRansac必须用float，这可能是一个bug
    vector<Point3f> objectPoints;
    vector<Point2f> imagePoints;
    Mat_<double> rvec;
    Mat_<double> tvec;
    for (int i=0; i<cloudPoints.size(); i++) {
        CloudPoint cp=cloudPoints[i];
        Point2d p;
        if(cp.known&&cp.getPointInFrame(frame, p))
        {
            objectPoints.push_back(Point3d(cp.x,cp.y,cp.z));
            imagePoints.push_back(p);
        }
    }
    solvePnPRansac(objectPoints, imagePoints, cameraMatrix, noArray(), rvec, tvec);
    Mat_<double> rmat;
    Rodrigues(rvec, rmat);
    p.create(3, 4,CV_64F);
    rmat.copyTo(p(Rect(0,0,3,3)));
    p.at<double>(0,3)=tvec(0);
    p.at<double>(1,3)=tvec(1);
    p.at<double>(2,3)=tvec(2);
}
ostream&operator<<(ostream&out,const Cloud&cloud)
{
    for (int i=0; i<cloud.cloudPoints.size(); i++) {
        out<<cloud.cloudPoints[i];
    }
    return out;
}
