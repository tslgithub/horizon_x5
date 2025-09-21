//
// Created by acer on 25-7-26.
//

#include  <iostream>
#include "stereo_depth.h"


int main(){

    std::unique_ptr<STEREO_DEPTH>stereo_depth_pytr = std::make_unique<STEREO_DEPTH>();
    stereo_depth_pytr->StereDepthRun();
    return 0;
}