#ifndef _COMMON_TYPES_H_
#define _COMMON_TYPES_H_

#include <string>
#include <vector>

// Forward declaration for ImageProfile (defined in ParallaxCorrection.h)
struct ImageProfile;

// Structure for stitching profile
struct StitchingProfile {
    std::string mode;
    std::string projection;
    double hFOV;
    int totalImages;
    double angularSpacing;
    bool enableFeatureMatching;
    bool enableFeatureMatchingCache;
    bool enableFeatureVisualization;
    bool enableWarpingVisualization;
    std::vector<ImageProfile> images;
};

#endif
