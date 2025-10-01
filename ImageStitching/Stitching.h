#ifndef _STITCHING_H_
#define _STITCHING_H_

#include "Feature.h"
#include "Match.h"
#include "Warping.h"
#include "Blend.h"

#define MAX_STITCHING_NUM 20

CImg<unsigned char> get_gray_image(const CImg<unsigned char> &srcImg);
CImg<unsigned char> stitching(vector<CImg<unsigned char>> &src_imgs);
int getMiddleIndex(vector<vector<int>> matching_index);
void updateFeaturesByHomography(map<vector<float>, VlSiftKeypoint> &feature, Parameters H, float offset_x, float offset_y);
void updateFeaturesByOffset(map<vector<float>, VlSiftKeypoint> &feature, int offset_x, int offset_y);

#endif