//
// Created by acer on 25-9-13.
//

#include "model.h"

int main(){
    Model model;
    model.LoadModel("/root/T-Robot/keypoints_match/model/dfeat.bin");
    model.FreeModel();
    return 0;
}