#ifndef _FEATURE_H_
#define _FEATURE_H_

#include <iostream>
#include <map>
#include <vector>
#include "CImg.h"

#define RESIZE_SIZE 500.0

using namespace cimg_library;
using namespace std;

// Feature matching algorithm types
enum class FeatureAlgorithm {
    SIFT,
    SURF,
    ORB
};

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
}

// SURF keypoint structure (compatible with SIFT keypoint for unified interface)
struct SurfKeypoint {
    float x, y;           // Keypoint coordinates
    float scale;          // Scale of the keypoint
    float angle;          // Orientation angle
    int ix, iy;           // Integer coordinates
    int octave;           // Octave level
    int class_id;         // Class ID for SURF
    float response;       // Response strength
};

// ORB keypoint structure (compatible with SIFT keypoint for unified interface)
struct OrbKeypoint {
    float x, y;           // Keypoint coordinates
    float scale;          // Scale of the keypoint
    float angle;          // Orientation angle
    int ix, iy;           // Integer coordinates
    int octave;           // Octave level
    int class_id;         // Class ID for ORB
    float response;       // Response strength
};

// Unified feature structure that can hold either SIFT or ORB features
struct UnifiedKeypoint {
    float x, y;
    float scale;
    float angle;
    int ix, iy;
    int octave;
    int class_id;
    float response;
    
    // Constructor from SIFT keypoint
    UnifiedKeypoint(const VlSiftKeypoint& sift_kp) {
        x = sift_kp.x;
        y = sift_kp.y;
        scale = sift_kp.sigma;
        angle = 0.0f;  // SIFT doesn't store angle in keypoint structure
        ix = sift_kp.ix;
        iy = sift_kp.iy;
        octave = sift_kp.o;
        class_id = 0;  // SIFT doesn't have class_id
        response = 0.0f;  // SIFT doesn't store peak response in keypoint
    }
    
    // Constructor from SURF keypoint
    UnifiedKeypoint(const SurfKeypoint& surf_kp) {
        x = surf_kp.x;
        y = surf_kp.y;
        scale = surf_kp.scale;
        angle = surf_kp.angle;
        ix = surf_kp.ix;
        iy = surf_kp.iy;
        octave = surf_kp.octave;
        class_id = surf_kp.class_id;
        response = surf_kp.response;
    }
    
    // Constructor from ORB keypoint
    UnifiedKeypoint(const OrbKeypoint& orb_kp) {
        x = orb_kp.x;
        y = orb_kp.y;
        scale = orb_kp.scale;
        angle = orb_kp.angle;
        ix = orb_kp.ix;
        iy = orb_kp.iy;
        octave = orb_kp.octave;
        class_id = orb_kp.class_id;
        response = orb_kp.response;
    }
    
    // Default constructor
    UnifiedKeypoint() : x(0), y(0), scale(0), angle(0), ix(0), iy(0), octave(0), class_id(0), response(0) {}
};

// Legacy function for backward compatibility
map<vector<float>, VlSiftKeypoint> getFeatureFromImage(const CImg<unsigned char> &src);

// New unified feature extraction functions
map<vector<float>, UnifiedKeypoint> getFeatureFromImageUnified(const CImg<unsigned char> &src, FeatureAlgorithm algorithm = FeatureAlgorithm::SIFT);
map<vector<float>, UnifiedKeypoint> getSiftFeatures(const CImg<unsigned char> &src);
map<vector<float>, UnifiedKeypoint> getSurfFeatures(const CImg<unsigned char> &src);
map<vector<float>, UnifiedKeypoint> getOrbFeatures(const CImg<unsigned char> &src);

#endif