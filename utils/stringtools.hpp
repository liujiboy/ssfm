//
//  stringtools.hpp
//  matching
//
//  Created by  刘骥 on 16/1/24.
//  Copyright © 2016年  刘骥. All rights reserved.
//

#ifndef stringtools_hpp
#define stringtools_hpp
#include <string>
using namespace std;
namespace  utils {
    /*
        判断字符串结尾是否是strEnd
     */
    bool endWith(const string& str,const string& strEnd);
    /*
        字符串转换为大写
     */
    string toUpperCase(string&str);
}
#endif /* stringtools_hpp */
