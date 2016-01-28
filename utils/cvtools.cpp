//
//  OpencvTools.cpp
//  sfm
//
//  Created by  刘骥 on 15/4/20.
//  Copyright (c) 2015年  刘骥. All rights reserved.
//

#include "cvtools.hpp"
namespace utils{
    void showImage(const string&name,const Mat&img,int flags)
    {
        namedWindow(name,flags);
        imshow(name, img);
    }


}