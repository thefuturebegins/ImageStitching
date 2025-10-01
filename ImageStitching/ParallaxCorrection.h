#ifndef _PARALLAX_CORRECTION_H_
#define _PARALLAX_CORRECTION_H_

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include "CImg.h"
#include "Feature.h"
#include "Match.h"

using namespace cimg_library;
using namespace std;

// Forward declaration - will be defined in main.cpp
struct ImageProfile {
    int index;
    string fileNamePrefix;
    string fileName;
    double radialAngle;
    double rotation;
    string description;
};

// Structure to represent a seam line between two images
struct SeamLine {
    int image1_index;
    int image2_index;
    vector<pair<int, int>> seam_points;  // Points along the seam line
    double confidence;  // Confidence in the seam line detection
};

// Structure for local warping control points
struct WarpControlPoint {
    int x, y;  // Original position
    int target_x, target_y;  // Target position after warping
    double weight;  // Weight for this control point
};

// Structure for local warping region
struct WarpRegion {
    int center_x, center_y;
    int radius;
    vector<WarpControlPoint> control_points;
    double max_displacement;
};

// Parallax correction class
class ParallaxCorrection {
private:
    // Parameters for seam detection
    static const int SEAM_DETECTION_WINDOW = 50;  // Window size for seam detection
    static constexpr double SEAM_CONFIDENCE_THRESHOLD = 0.7;
    static const int MIN_FEATURES_NEAR_SEAM = 10;

    // Parameters for local warping
    static const int WARP_REGION_RADIUS = 100;  // Radius of influence for local warping
    static constexpr double MAX_DISPLACEMENT_RATIO = 0.1;  // Maximum displacement as ratio of image size
    static const int WARP_GRID_SIZE = 20;  // Grid size for local warping

        public:
            // Detect seam lines between adjacent images
            static vector<SeamLine> detectSeamLines(
                const vector<CImg<unsigned char>>& images,
                const vector<ImageProfile>& image_profiles,
                const vector<map<vector<float>, VlSiftKeypoint>>& features
            );

            // Generate seam lines from stitching profile (radial angles)
            static vector<SeamLine> generateSeamLinesFromProfile(
                const vector<ImageProfile>& image_profiles,
                int image_width,
                int image_height
            );

    // Find features near seam lines for parallax correction
    static vector<point_pair> findSeamFeatures(
        const CImg<unsigned char>& img1,
        const CImg<unsigned char>& img2,
        const map<vector<float>, VlSiftKeypoint>& features1,
        const map<vector<float>, VlSiftKeypoint>& features2,
        const SeamLine& seam_line
    );

    // Find features only within the exact overlap region between adjacent images
    static vector<point_pair> findSeamFeaturesInOverlap(
        const CImg<unsigned char>& img1,
        const CImg<unsigned char>& img2,
        const map<vector<float>, VlSiftKeypoint>& features1,
        const map<vector<float>, VlSiftKeypoint>& features2,
        const SeamLine& seam_line
    );

    // Calculate parallax displacement for features near seam
    static vector<WarpControlPoint> calculateParallaxDisplacement(
        const vector<point_pair>& seam_features,
        const SeamLine& seam_line,
        int image_width,
        int image_height
    );

    // Apply local warping to correct parallax
    static CImg<unsigned char> applyLocalWarping(
        const CImg<unsigned char>& image,
        const vector<WarpControlPoint>& control_points,
        int image_width,
        int image_height
    );

    // Main function to correct parallax for all images
    static vector<CImg<unsigned char>> correctParallax(
        const vector<CImg<unsigned char>>& images,
        const vector<ImageProfile>& image_profiles,
        const vector<map<vector<float>, VlSiftKeypoint>>& features
    );

    // Main function to correct parallax using pre-computed seam lines
    static vector<CImg<unsigned char>> correctParallaxWithProfile(
        const vector<CImg<unsigned char>>& src_imgs,
        const vector<ImageProfile>& image_profiles,
        const vector<map<vector<float>, VlSiftKeypoint>>& features
    );

    // Create feature visualization image
    static CImg<unsigned char> createFeatureVisualization(
        const vector<CImg<unsigned char>>& src_imgs,
        const vector<ImageProfile>& image_profiles,
        const vector<SeamLine>& seam_lines,
        const vector<map<vector<float>, VlSiftKeypoint>>& features
    );

private:
    // Helper function to calculate distance from point to seam line
    static double distanceToSeamLine(int x, int y, const SeamLine& seam_line);

    // Helper function to find the closest point on seam line
    static pair<int, int> findClosestSeamPoint(int x, int y, const SeamLine& seam_line);

    // Helper function for bilinear interpolation in warping
    static unsigned char bilinearInterpolate(
        const CImg<unsigned char>& image,
        double x, double y, int channel
    );

    // Helper function to calculate warping weights
    static double calculateWarpingWeight(
        int x, int y,
        const WarpControlPoint& control_point,
        int radius
    );
};

#endif
