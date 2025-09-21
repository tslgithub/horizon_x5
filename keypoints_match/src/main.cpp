//
// Created by acer on 25-9-7.
//

#include "keypoints_match.h"

int main() {
    std::unique_ptr<KEYPOINTS_MATCH> keypoints_match_ptr = std::make_unique<KEYPOINTS_MATCH>();
    keypoints_match_ptr->KeypointsMatchRun();

    return 0;
}