#ifndef _FEATURE_CACHE_H_
#define _FEATURE_CACHE_H_

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include "CImg.h"
#include "Match.h"
#include "CommonTypes.h"

using namespace cimg_library;
using namespace std;

extern "C" {
    #include "vl/generic.h"
    #include "vl/sift.h"
}

// Structure to hold cached feature data
struct CachedFeatureData {
    vector<vector<float>> descriptors;
    vector<VlSiftKeypoint> keypoints;
    int imageIndex;
    string fileName;
    int width;
    int height;
};

// Structure to hold cached feature match data
struct CachedMatchData {
    vector<point_pair> pairs;
    int image1Index;
    int image2Index;
    int image1Width;
    int image1Height;
    int image2Width;
    int image2Height;
};

// Structure to hold all cached data
struct FeatureCacheData {
    vector<CachedFeatureData> features;
    vector<CachedMatchData> matches;
    string cacheVersion;
    string datasetName;
    int totalImages;
    double hFOV;
    double angularSpacing;
};

class FeatureCache {
public:
    // Save feature data to cache file
    static bool saveFeatureCache(const string& cacheFilePath,
                                const vector<map<vector<float>, VlSiftKeypoint>>& features,
                                const vector<string>& imageFiles,
                                const vector<ImageProfile>& imageProfiles,
                                const StitchingProfile& profile);

    // Load feature data from cache file
    static bool loadFeatureCache(const string& cacheFilePath,
                                vector<map<vector<float>, VlSiftKeypoint>>& features,
                                vector<string>& imageFiles,
                                vector<ImageProfile>& imageProfiles,
                                StitchingProfile& profile);

    // Check if cache file exists and is valid
    static bool isCacheValid(const string& cacheFilePath,
                            const vector<string>& imageFiles,
                            const StitchingProfile& profile);

    // Generate cache file path based on output filename
    static string generateCacheFilePath(const string& outputFilename);

    // Convert features map to cached format
    static CachedFeatureData convertToCachedFormat(const map<vector<float>, VlSiftKeypoint>& features,
                                                  int imageIndex, const string& fileName, int width, int height);

    // Convert cached format back to features map
    static map<vector<float>, VlSiftKeypoint> convertFromCachedFormat(const CachedFeatureData& cachedData);

};

#endif
