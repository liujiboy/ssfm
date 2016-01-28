//
//  stringtools.cpp
//  matching
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#include "stringtools.hpp"
namespace  utils {
    bool endWith(const string& str,const string& strEnd)
    {
        if(str.empty() || strEnd.empty())
        {
            return false;
        }
        return str.compare(str.size()-strEnd.size(),strEnd.size(),strEnd)==0?true:false;
    }
    string toUpperCase(string&str)
    {
        string result=str;
        transform(result.begin(), result.end(), result.begin(), ::toupper);
        return result;
    }
}