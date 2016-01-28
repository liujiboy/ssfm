//
//  SFM.cpp
//  sfm
//
//  Created by  刘骥 on 15/4/21.
//  Copyright (c) 2015年  刘骥. All rights reserved.
//

#include "sfm.hpp"
#include "quaternion.hpp"
#include "sfmtools.hpp"
struct globs_{
    double *rot0params; /* initial rotation parameters, combined with a local rotation parameterization */
    double *intrcalib; /* the 5 intrinsic calibration parameters in the order [fu, u0, v0, ar, skew],
                        * where ar is the aspect ratio fv/fu.
                        * Used only when calibration is fixed for all cameras;
                        * otherwise, it is null and the intrinsic parameters are
                        * included in the set of motion parameters for each camera
                        */
    int nccalib; /* number of calibration parameters that must be kept constant.
                  * 0: all parameters are free
                  * 1: skew is fixed to its initial value, all other parameters vary (i.e. fu, u0, v0, ar)
                  * 2: skew and aspect ratio are fixed to their initial values, all other parameters vary (i.e. fu, u0, v0)
                  * 3: meaningless
                  * 4: skew, aspect ratio and principal point are fixed to their initial values, only the focal length varies (i.e. fu)
                  * 5: all intrinsics are kept fixed to their initial values
                  * >5: meaningless
                  * Used only when calibration varies among cameras
                  */
    
    int ncdist; /* number of distortion parameters in Bouguet's model that must be kept constant.
                 * 0: all parameters are free
                 * 1: 6th order radial distortion term (kc[4]) is fixed
                 * 2: 6th order radial distortion and one of the tangential distortion terms (kc[3]) are fixed
                 * 3: 6th order radial distortion and both tangential distortion terms (kc[3], kc[2]) are fixed [i.e., only 2nd & 4th order radial dist.]
                 * 4: 4th & 6th order radial distortion terms and both tangential distortion ones are fixed [i.e., only 2nd order radial dist.]
                 * 5: all distortion parameters are kept fixed to their initial values
                 * >5: meaningless
                 * Used only when calibration varies among cameras and distortion is to be estimated
                 */
    int cnp, pnp, mnp; /* dimensions */
    
    double *ptparams; /* needed only when bundle adjusting for camera parameters only */
    double *camparams; /* needed only when bundle adjusting for structure parameters only */
} ;
/*** MEASUREMENT VECTOR AND JACOBIAN COMPUTATION FOR THE EXPERT DRIVERS ***/

/* FULL BUNDLE ADJUSTMENT, I.E. SIMULTANEOUS ESTIMATION OF CAMERA AND STRUCTURE PARAMETERS */

/* Given a parameter vector p made up of the 3D coordinates of n points and the parameters of m cameras, compute in
 * hx the prediction of the measurements, i.e. the projections of 3D points in the m images. The measurements
 * are returned in the order (hx_11^T, .. hx_1m^T, ..., hx_n1^T, .. hx_nm^T)^T, where hx_ij is the predicted
 * projection of the i-th point on the j-th camera.
 * Notice that depending on idxij, some of the hx_ij might be missing
 *
 */
static void img_projsRTS_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata)
{
    int i, j;
    int cnp, pnp, mnp;
    double *pa, *pb, *pqr, *pt, *ppt, *pmeas, *Kparms, *pr0, lrot[FULLQUATSZ], trot[FULLQUATSZ];
    //int n;
    int m, nnz;
    struct globs_ *gl;
    
    gl=(struct globs_ *)adata;
    cnp=gl->cnp; pnp=gl->pnp; mnp=gl->mnp;
    Kparms=gl->intrcalib;
    
    //n=idxij->nr;
    m=idxij->nc;
    pa=p; pb=p+m*cnp;
    
    for(j=0; j<m; ++j){
        /* j-th camera parameters */
        pqr=pa+j*cnp;
        pt=pqr+3; // quaternion vector part has 3 elements
        pr0=gl->rot0params+j*FULLQUATSZ; // full quat for initial rotation estimate
        _MK_QUAT_FRM_VEC(lrot, pqr);
        quatMultFast(lrot, pr0, trot); // trot=lrot*pr0
        
        nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */
        
        for(i=0; i<nnz; ++i){
            ppt=pb + rcsubs[i]*pnp;
            pmeas=hx + idxij->val[rcidxs[i]]*mnp; // set pmeas to point to hx_ij
            
            calcImgProjFullR(Kparms, trot, pt, ppt, pmeas); // evaluate Q in pmeas
            //calcImgProj(Kparms, pr0, pqr, pt, ppt, pmeas); // evaluate Q in pmeas
        }
    }
}

/* Given a parameter vector p made up of the 3D coordinates of n points and the parameters of m cameras, compute in
 * jac the jacobian of the predicted measurements, i.e. the jacobian of the projections of 3D points in the m images.
 * The jacobian is returned in the order (A_11, ..., A_1m, ..., A_n1, ..., A_nm, B_11, ..., B_1m, ..., B_n1, ..., B_nm),
 * where A_ij=dx_ij/db_j and B_ij=dx_ij/db_i (see HZ).
 * Notice that depending on idxij, some of the A_ij, B_ij might be missing
 *
 */
static void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata)
{
    int i, j;
    int cnp, pnp, mnp;
    double *pa, *pb, *pqr, *pt, *ppt, *pA, *pB, *Kparms, *pr0;
    //int n;
    int m, nnz, Asz, Bsz, ABsz;
    struct globs_ *gl;
    
    gl=(struct globs_ *)adata;
    cnp=gl->cnp; pnp=gl->pnp; mnp=gl->mnp;
    Kparms=gl->intrcalib;
    
    //n=idxij->nr;
    m=idxij->nc;
    pa=p; pb=p+m*cnp;
    Asz=mnp*cnp; Bsz=mnp*pnp; ABsz=Asz+Bsz;
    
    for(j=0; j<m; ++j){
        /* j-th camera parameters */
        pqr=pa+j*cnp;
        pt=pqr+3; // quaternion vector part has 3 elements
        pr0=gl->rot0params+j*FULLQUATSZ; // full quat for initial rotation estimate
        
        nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */
        
        for(i=0; i<nnz; ++i){
            ppt=pb + rcsubs[i]*pnp;
            pA=jac + idxij->val[rcidxs[i]]*ABsz; // set pA to point to A_ij
            pB=pA  + Asz; // set pB to point to B_ij
            
            calcImgProjJacRTS(Kparms, pr0, pqr, pt, ppt, (double (*)[6])pA, (double (*)[3])pB); // evaluate dQ/da, dQ/db in pA, pB
        }
    }
}


void SFM::computeFeatures(const vector<Mat>&images)
{
    SIFT sift(0, 3, 0.01, 100, 1.6);
    //SIFT sift;
    for(vector<Mat>::const_iterator it=images.begin();it!=images.end();it++)
    {
        vector<KeyPoint> kp;
        Mat des;
        sift(*it,Mat(),kp,des);
        vector<Point2d> points;
        for (vector<KeyPoint>::iterator it=kp.begin(); it!=kp.end(); it++) {
            points.push_back(it->pt);
        }
        keypoints.push_back(points);
        descriptors.push_back(des);
    }
}

void SFM::pairwiseMatch()
{
    for (int i=0; i<nframes; i++) {
        for (int j=i+1; j<nframes; j++) {
            
            vector<DMatch> match;
            Mat f=computeFundamentalMatAndMatch(keypoints[i],keypoints[j],descriptors[i],descriptors[j],match);
            fmatrices[i][j]=f;
            matches[i][j]=match;
            cout<<"计算图像"<<i<<"和图像"<<j<<"之间的匹配以及基础矩阵,匹配点数为"<<match.size()<<endl;
        }
    }
}
SFM::SFM(Mat cameraMatrix,const vector<Mat>& images)
{
    nframes=(int)images.size();
    fmatrices.resize(nframes, vector<Mat>(nframes));
    matches.resize(nframes,vector<vector<DMatch> >(nframes));
    pmatrices.resize(nframes);
    this->cameraMatrix=cameraMatrix;
    computeFeatures(images);
    pairwiseMatch();
    cloud=new Cloud(keypoints,matches,nframes);
    initialReconstruct();
}
void SFM::initialReconstruct()
{
    //查找匹配最多的两个视图作为初始视图
    int maxMatchSize=0;
    int idx1=0,idx2=0;
    for(int i=0;i<nframes;i++)
        for (int j=i+1; j<nframes; j++) {
            int matchSize=(int)matches[i][j].size();
            if(matchSize>maxMatchSize)
            {
                idx1=i;
                idx2=j;
                maxMatchSize=matchSize;
            }
        }
   // idx1=0;
    //idx2=1;
    cout<<"初始匹配是："<<idx1<<","<<idx2<<endl;
    //查找idx1和idx2对应视图的未知点
    knowFramevec.push_back(idx1);
    knowFramevec.push_back(idx2);
    vector<Point2d>points1,points2;
    vector<int> idxvec;
    cloud->findUnKnowPoints(idx1, idx2, points1, points2, idxvec);
    //初始重建
    cout<<"对视图"<<idx1<<"和视图"<<idx2<<"的匹配点进行三角化"<<endl;
    Mat P1=Mat(Matx34d(1,0,0,0,0,1,0,0,0,0,1,0));
    Mat_<double> P2;
    vector<Point3d> points3d;
    vector<bool> frontMask(points1.size(),false);
    trianglulateAndFindPMatrix(points1,points2,P1,fmatrices[idx1][idx2],cameraMatrix,P2,points3d,frontMask);
    
    cloud->updatePoints(points3d,idxvec,frontMask);
    pmatrices[idx1]=P1;
    pmatrices[idx2]=P2;
    cloud->printReprojectionError(knowFramevec, cameraMatrix, pmatrices);
    cloud->filter(idx1, knowFramevec, cameraMatrix, pmatrices[idx1]);
    cloud->filter(idx2, knowFramevec, cameraMatrix, pmatrices[idx2]);
    cloud->printReprojectionError(knowFramevec, cameraMatrix, pmatrices);
    cout<<"三角化完成，开始bundle adjustment"<<endl;
    sba();
    cloud->printReprojectionError(knowFramevec, cameraMatrix, pmatrices);
   // cloud->saveCloudPoints("/Users/liuji/cloud.txt");
    //
    //nviewSba(2);
    //cloud->reprojectError(2);
}

void SFM::reconstruct()
{
   
    while (knowFramevec.size()!=nframes) {
        //查找下一个重建视图
        int nextFrame=cloud->chooseNextFrame(knowFramevec,nframes);
        knowFramevec.push_back(nextFrame);
        cout<<"正在计算视图"<<nextFrame<<endl;
        //计算视图的投影矩阵
        cloud->findPmatrixByKnown(nextFrame, cameraMatrix, pmatrices[nextFrame]);
        cout<<"计算投影矩阵后********"<<endl;
        cloud->printReprojectionError(knowFramevec, cameraMatrix, pmatrices);
        //重建视图的未知点
        vector<Point2d>points1,points2;
        vector<int> idxvec;
        for (int i=0; i<knowFramevec.size(); i++) {
            if (nextFrame!=knowFramevec[i]) {
                int idx1,idx2;
                if (nextFrame>knowFramevec[i]) {
                    idx1=knowFramevec[i];
                    idx2=nextFrame;
                }else
                {
                    idx2=knowFramevec[i];
                    idx1=nextFrame;
                }
                cloud->findUnKnowPoints(idx1, idx2, points1, points2, idxvec);
                if(idxvec.size()>0)
                {
                vector<Point3d> points3d;
                vector<bool> frontMask(points1.size(),false);
                int frontCount;
                triangleatePoints(pmatrices[idx1], pmatrices[idx2], cameraMatrix, points1, points2, points3d, frontMask,frontCount);
                cloud->updatePoints(points3d,idxvec,frontMask);
                    
                }
               // cloud->reprojectError();
            }
            
        }
        cout<<"重建三维点后********"<<endl;
        cloud->printReprojectionError(knowFramevec, cameraMatrix, pmatrices);
        for (int i=0; i<knowFramevec.size(); i++)
            cloud->filter(knowFramevec[i], knowFramevec, cameraMatrix, pmatrices[knowFramevec[i]]);
        cout<<"过滤后********"<<endl;
        cloud->printReprojectionError(knowFramevec, cameraMatrix, pmatrices);
       //sba(n++);
        //cout<<"优化之前"<<endl;
        //cloud->reprojectError();
        sba();
        //cout<<"优化之后"<<endl;
        cout<<"优化之后*******"<<endl;
        cloud->printReprojectionError(knowFramevec, cameraMatrix, pmatrices);
        cout<<"*************"<<endl;
       // cin.get();

    }
    cloud->deleteOutliers();
   
}

void SFM::plotCloudPoints(const vector<Mat>&images)
{
    cloud->plotCloudPoints(knowFramevec, cameraMatrix, pmatrices, images);
}
void SFM::saveCloudPointsToPly(const vector<Mat>&images,const char* fileName)
{
    cloud->saveCloudPointsToPly(images, fileName);
}
Mat SFM::getPMatrix(int i)
{
    return pmatrices[i];
}
void SFM::savePmatrix(const string& fileName,int i)
{
    ofstream out(fileName.c_str());
    Mat mat=this->pmatrices[i];
    for (int i=0; i<mat.rows; i++) {
        for(int j=0;j<mat.cols;j++)
        {
            if(j!=mat.cols-1)
                out<<mat.at<double>(i,j)<<" ";
            else
                out<<mat.at<double>(i,j)<<endl;
        }
    }
    out.close();
    
}
SFM::~SFM()
{
    delete cloud;
}
void SFM::sba(int nconstframes,int nconstpts3D,int maxiter,int verbose)
{
    int frameNum=(int)knowFramevec.size();
    int cnp=6;  //相机矩阵用3位表示旋转，3位表示位移
    int pnp=3;  //三维点的坐标数
    int mnp=2;  //二维点的坐标数
    double f=cameraMatrix.at<double>(0,0);
    double cx=cameraMatrix.at<double>(0,2);
    double cy=cameraMatrix.at<double>(1,2);
    double ar=cameraMatrix.at<double>(1,1)/cameraMatrix.at<double>(0,0);
    double ical[5]={f,cx,cy,ar,0};//f cx cy ar s;
    double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
    struct globs_ globs;
    //设置globs
    globs.cnp=cnp;
    globs.pnp=pnp;
    globs.mnp=mnp;
    globs.rot0params=new double[FULLQUATSZ*frameNum]();
    globs.intrcalib=ical;
    globs.ptparams=NULL;
    globs.camparams=NULL;
    
    //设置优化选项
    opts[0]=SBA_INIT_MU;
    opts[1]=SBA_STOP_THRESH;
    opts[2]=SBA_STOP_THRESH;
    opts[3]=SBA_STOP_THRESH;
    opts[4]=0.0;
    
    vector<int> knownIdxvec;
    cloud->getKnownPoints(knownIdxvec);
    int numpts3D=(int)knownIdxvec.size();   //三维点的数量
    int numprojs=0; //在所有相机下，三维点共计有多少个二维投影
    //vmask[i,j]表示第i个点在第j个镜头下是否可见，此处填充为全1，因为点在两个镜头下均可见
    char *vmask=new char[numpts3D*frameNum]();
    for(int i=0;i<numpts3D;i++)
    {
        CloudPoint cp=cloud->getPoint(knownIdxvec[i]);
        for (int j=0; j<frameNum; j++) {
            int index=i*frameNum+j;
            Point2d p;
            if(cp.getPointInFrame(knowFramevec[j], p))
            {
                vmask[index]=1;
                numprojs++;
            }
        }
    }
    //motstruct是待优化的相机矩阵和三维点，其结构为(r1,t1,r2,t2,X[1],X[2]...X[n])
    int motstruct_size=frameNum*cnp+numpts3D*pnp;
    double *motstruct=new double[motstruct_size]();
    for(int i=0;i<frameNum;i++)
    {
        Mat_<double> p=pmatrices[knowFramevec[i]];
        double r[4],t[3];
        cammat2quat(p, r, t);
        copy(r+1, r+4, motstruct+i*cnp);
        copy(t,t+3,motstruct+i*cnp+3);
        copy(r,r+4, globs.rot0params+i*FULLQUATSZ);
    }
    //拷贝三维点
    int pstart=frameNum*cnp; //三维点的开始位置
    for(int i=0;i<numpts3D;i++)
    {
        CloudPoint cp=cloud->getPoint(knownIdxvec[i]);
        motstruct[pstart+i*pnp]=cp.x;
        motstruct[pstart+i*pnp+1]=cp.y;
        motstruct[pstart+i*pnp+2]=cp.z;
    }
    //如果要对相机旋转矩阵和三维点的位置同时优化，必须将相机矩阵的旋转初始化为0，即四元数表示的(1,0,0,0)
    //并用globs.rot0params保存了相机旋转矩阵
    //若只对三维点的位置进行优化，此步不做
    for(int i=0; i<frameNum; ++i){
        int j=(i+1)*cnp; // 跳过位移向量
        motstruct[j-4]=motstruct[j-5]=motstruct[j-6]=0.0; // 设置为(1,0,0,0)
    }
    //imgpts保存三维点在每个相机下的投影，即二维点
    double *imgpts=new double[numprojs*mnp]();
    for(int i=0,n=0;i<numpts3D;i++)
    {
        CloudPoint cp=cloud->getPoint(knownIdxvec[i]);
        for (int j=0; j<frameNum; j++) {
            Point2d point;
            if(cp.getPointInFrame(knowFramevec[j],point))
            {
                imgpts[n*mnp]=point.x;
                imgpts[n*mnp+1]=point.y;
                n++;
            }
        }
    }
    
    double *covimgpts=NULL;
    //cout<<"优化前："<<endl;
    //printSBAData(stdout, motstruct, cnp, pnp, mnp, vec2quat, cnp+1, frameNum, numpts3D, imgpts, numprojs, vmask);
   
    
    //优化
    int n=sba_motstr_levmar_x(numpts3D, nconstpts3D, frameNum, nconstframes, vmask, motstruct, cnp, pnp, imgpts, covimgpts, mnp,img_projsRTS_x,img_projsRTS_jac_x,(void*)(&globs),maxiter, verbose, opts, info);
    if(n!=SBA_ERROR)
    {
        
        /* combine the local rotation estimates with the initial ones */
        for(int i=0; i<frameNum; ++i){
            double *v, qs[FULLQUATSZ], *q0, prd[FULLQUATSZ];
            
            /* retrieve the vector part */
            v=motstruct + (i+1)*cnp - 6; // note the +1, we access the motion parameters from the right, assuming 3 for translation!
            _MK_QUAT_FRM_VEC(qs, v);
            
            q0=globs.rot0params+i*FULLQUATSZ;
            quatMultFast(qs, q0, prd); // prd=qs*q0
            
            /* copy back vector part making sure that the scalar part is non-negative */
            if(prd[0]>=0.0){
                v[0]=prd[1];
                v[1]=prd[2];
                v[2]=prd[3];
            }
            else{ // negate since two quaternions q and -q represent the same rotation
                v[0]=-prd[1];
                v[1]=-prd[2];
                v[2]=-prd[3];
            }
        }
        //cout<<"优化后："<<endl;
        //printSBAData(stdout, motstruct, cnp, pnp, mnp, vec2quat, cnp+1, frameNum, numpts3D, imgpts, numprojs, vmask);
        for(int i=0;i<frameNum;i++)
        {
            Mat_<double> p(3,4);
            double r[3],t[3];
            copy(motstruct+i*cnp, motstruct+3+i*cnp, r);
            copy(motstruct+3+i*cnp, motstruct+6+i*cnp, t);
            quat2cammat(r,t,p);
            //cout<<p<<endl;
            pmatrices[knowFramevec[i]]=p;
           
        }
        for (int i=0; i<numpts3D; i++) {
            CloudPoint& cp=cloud->getPoint(knownIdxvec[i]);
            cp.x=motstruct[frameNum*cnp+i*pnp];
            cp.y=motstruct[frameNum*cnp+i*pnp+1];
            cp.z=motstruct[frameNum*cnp+i*pnp+2];
        }
    }

    
}

