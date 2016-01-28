//
//  CloudPoint.cpp
//  sfm
//
//  Created by  刘骥 on 15/4/18.
//  Copyright (c) 2015年  刘骥. All rights reserved.
//

#include "cloudpoint.hpp"
CloudPoint::CloudPoint(double x,double y,double z,int nframes):matchPoints(nframes,Point2d(-1,-1)){
    this->x=x;
    this->y=y;
    this->z=z;
    this->nframes=nframes;
    this->known=true;
}
CloudPoint::CloudPoint(int nframes):matchPoints(nframes,Point2d(-1,-1)){
    this->x=0;
    this->y=0;
    this->z=0;
    this->nframes=nframes;
    this->known=false;
}

bool  CloudPoint::getPointInFrame(int frame,Point2d&p)const 
{
    Point2d mp=matchPoints[frame];
    if(mp.x==-1)
        return false;
    else
    {
        p.x=mp.x;
        p.y=mp.y;
        return true;
    }

}
void CloudPoint::setPointInFrame(int frame, Point2d p)
{
    matchPoints[frame]=p;
}

ostream&operator<<(ostream&out,const CloudPoint&cp)
{
    if (cp.known) {
    out<<cp.x<<" "<<cp.y<<" "<<cp.z<<" ";
    for (int i =0; i<cp.nframes;i++) {
        Point2d p;
        if(cp.getPointInFrame(i,p))
        {
            out<<i<<" "<<p.x<<" "<<p.y<<" ";
        }
    }
        out<<endl;
    }
    return out;
}