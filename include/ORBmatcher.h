/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"map_point.h"
#include"frame.h"


namespace my_slam
{

class ORBmatcher {    
public:

 ORBmatcher(float nnratio=0.6, bool checkOri=true);
 static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
 
public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

protected:
 float mfNNratio;
 bool mbCheckOrientation;

    
};

}// namespace my_slam

#endif // ORBMATCHER_H
