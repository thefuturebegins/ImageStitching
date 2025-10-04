#include "stdafx.h"
#include "FeatureCache.h"
#include "Match.h"
#include "ParallaxCorrection.h"
#include "CommonTypes.h"
#include <sstream>
#include <iomanip>
#include <cmath>

bool FeatureCache::saveFeatureCache(const string& cacheFilePath,
                                   const vector<map<vector<float>, VlSiftKeypoint>>& features,
                                   const vector<string>& imageFiles,
                                   const vector<ImageProfile>& imageProfiles,
                                   const StitchingProfile& profile) {
    ofstream cacheFile(cacheFilePath);
    if (!cacheFile.is_open()) {
        cout << "Error: Could not create cache file " << cacheFilePath << endl;
        return false;
    }

    cout << "Saving feature cache to: " << cacheFilePath << endl;

    // Create cache data structure
    FeatureCacheData cacheData;
    cacheData.cacheVersion = "1.0";
    cacheData.datasetName = "dataset3"; // Could be made configurable
    cacheData.totalImages = features.size();
    cacheData.hFOV = profile.hFOV;
    cacheData.angularSpacing = profile.angularSpacing;

    // Convert features to cached format
    for (int i = 0; i < features.size(); i++) {
        string fileName = imageFiles[i];
        // Extract just the filename from the full path
        size_t lastSlash = fileName.find_last_of("/\\");
        string justFilename = (lastSlash != string::npos) ? fileName.substr(lastSlash + 1) : fileName;

        // Get image dimensions (assuming standard dimensions for now)
        int width = 3840;  // Could be made configurable
        int height = 2160; // Could be made configurable

        CachedFeatureData cachedFeature = convertToCachedFormat(features[i], i, justFilename, width, height);
        cacheData.features.push_back(cachedFeature);
    }

    // Write JSON format cache file
    cacheFile << "{\n";
    cacheFile << "  \"cacheVersion\": \"" << cacheData.cacheVersion << "\",\n";
    cacheFile << "  \"datasetName\": \"" << cacheData.datasetName << "\",\n";
    cacheFile << "  \"totalImages\": " << cacheData.totalImages << ",\n";
    cacheFile << "  \"hFOV\": " << cacheData.hFOV << ",\n";
    cacheFile << "  \"angularSpacing\": " << cacheData.angularSpacing << ",\n";
    cacheFile << "  \"features\": [\n";

    for (int i = 0; i < cacheData.features.size(); i++) {
        const CachedFeatureData& feature = cacheData.features[i];
        cacheFile << "    {\n";
        cacheFile << "      \"imageIndex\": " << feature.imageIndex << ",\n";
        cacheFile << "      \"fileName\": \"" << feature.fileName << "\",\n";
        cacheFile << "      \"width\": " << feature.width << ",\n";
        cacheFile << "      \"height\": " << feature.height << ",\n";
        cacheFile << "      \"keypointCount\": " << feature.keypoints.size() << ",\n";
        cacheFile << "      \"keypoints\": [\n";

        for (int j = 0; j < feature.keypoints.size(); j++) {
            const VlSiftKeypoint& kp = feature.keypoints[j];
            cacheFile << "        {\n";
            cacheFile << "          \"x\": " << kp.x << ",\n";
            cacheFile << "          \"y\": " << kp.y << ",\n";
            cacheFile << "          \"ix\": " << kp.ix << ",\n";
            cacheFile << "          \"iy\": " << kp.iy << ",\n";
            cacheFile << "          \"sigma\": " << kp.sigma << ",\n";
            cacheFile << "          \"o\": " << kp.o << ",\n";
            cacheFile << "          \"s\": " << kp.s << "\n";
            cacheFile << "        }";
            if (j < feature.keypoints.size() - 1) cacheFile << ",";
            cacheFile << "\n";
        }

        cacheFile << "      ]\n";
        cacheFile << "    }";
        if (i < cacheData.features.size() - 1) cacheFile << ",";
        cacheFile << "\n";
    }

    cacheFile << "  ]\n";
    cacheFile << "}\n";

    cacheFile.close();
    cout << "Feature cache saved successfully with " << cacheData.features.size() << " images" << endl;
    return true;
}

bool FeatureCache::loadFeatureCache(const string& cacheFilePath,
                                   vector<map<vector<float>, VlSiftKeypoint>>& features,
                                   vector<string>& imageFiles,
                                   vector<ImageProfile>& imageProfiles,
                                   StitchingProfile& profile) {
    ifstream cacheFile(cacheFilePath);
    if (!cacheFile.is_open()) {
        cout << "Cache file not found: " << cacheFilePath << endl;
        return false;
    }

    cout << "Loading feature cache from: " << cacheFilePath << endl;

    // Read the entire JSON file
    stringstream buffer;
    buffer << cacheFile.rdbuf();
    string json = buffer.str();
    cacheFile.close();

    // Simple JSON parsing for cache file
    // Parse totalImages
    size_t totalImagesPos = json.find("\"totalImages\":");
    if (totalImagesPos != string::npos) {
        totalImagesPos = json.find(":", totalImagesPos);
        if (totalImagesPos != string::npos) {
            totalImagesPos = json.find_first_not_of(" \t\n\r", totalImagesPos + 1);
            if (totalImagesPos != string::npos) {
                size_t endPos = totalImagesPos;
                while (endPos < json.length() && isdigit(json[endPos])) {
                    endPos++;
                }
                string totalStr = json.substr(totalImagesPos, endPos - totalImagesPos);
                int totalImages = atoi(totalStr.c_str());
                features.resize(totalImages);
            }
        }
    }

    // Parse features array
    size_t featuresStart = json.find("\"features\":");
    if (featuresStart == string::npos) {
        cout << "Error: No features array found in cache file" << endl;
        return false;
    }

    featuresStart = json.find("[", featuresStart);
    if (featuresStart == string::npos) {
        cout << "Error: Malformed features array in cache file" << endl;
        return false;
    }

    // Find the matching closing bracket by counting nested brackets
    size_t featuresEnd = featuresStart;
    int bracketCount = 0;
    bool inString = false;
    bool escaped = false;

    for (size_t i = featuresStart; i < json.length(); i++) {
        char c = json[i];

        if (escaped) {
            escaped = false;
            continue;
        }

        if (c == '\\') {
            escaped = true;
            continue;
        }

        if (c == '"') {
            inString = !inString;
            continue;
        }

        if (!inString) {
            if (c == '[') {
                bracketCount++;
            } else if (c == ']') {
                bracketCount--;
                if (bracketCount == 0) {
                    featuresEnd = i;
                    break;
                }
            }
        }
    }

    if (bracketCount != 0) {
        cout << "Error: Unclosed features array in cache file" << endl;
        return false;
    }

    string featuresArray = json.substr(featuresStart, featuresEnd - featuresStart + 1);
    cout << "Features array length: " << featuresArray.length() << endl;
    cout << "First 200 chars of features array: " << featuresArray.substr(0, 200) << endl;
    cout << "Last 200 chars of features array: " << featuresArray.substr(featuresArray.length() - 200) << endl;
    cout << "featuresStart: " << featuresStart << ", featuresEnd: " << featuresEnd << endl;

    // Parse each feature object using a simpler approach
    size_t pos = 0;
    int featureIndex = 0;
    cout << "Starting to parse features array, looking for feature objects..." << endl;

    // Look for feature objects by finding "imageIndex" patterns
    size_t firstImageIndex = featuresArray.find("\"imageIndex\":");
    cout << "First imageIndex found at position: " << firstImageIndex << endl;
    while ((pos = featuresArray.find("\"imageIndex\":", pos)) != string::npos && featureIndex < features.size()) {
        cout << "Found imageIndex at position " << pos << ", featureIndex=" << featureIndex << endl;
        // Find the start of this feature object (go backwards to find the opening brace)
        size_t objStart = pos;
        while (objStart > 0 && featuresArray[objStart] != '{') {
            objStart--;
        }

        cout << "Found opening brace at position: " << objStart << endl;

        if (objStart == 0) {
            cout << "No opening brace found, skipping..." << endl;
            pos += 13; // Skip "imageIndex"
            continue;
        }

        // Find the end of this feature object by looking for the next "imageIndex" or end of array
        size_t objEnd = featuresArray.find("\"imageIndex\":", pos + 13);
        cout << "Next imageIndex found at position: " << objEnd << endl;
        if (objEnd == string::npos) {
            // This is the last feature object, find the closing brace
            objEnd = featuresArray.find_last_of('}');
            cout << "Last feature object, closing brace at: " << objEnd << endl;
        } else {
            // Find the closing brace before the next "imageIndex"
            objEnd = featuresArray.find_last_of('}', objEnd);
            cout << "Closing brace before next imageIndex at: " << objEnd << endl;
        }

        if (objEnd == string::npos) {
            cout << "No closing brace found, breaking..." << endl;
            break;
        }

        string featureObj = featuresArray.substr(objStart, objEnd - objStart + 1);

        // Parse imageIndex
        size_t indexPos = featureObj.find("\"imageIndex\":");
        if (indexPos == string::npos) {
            pos = objEnd + 1;
            continue;
        }

        indexPos = featureObj.find(":", indexPos);
        if (indexPos == string::npos) {
            pos = objEnd + 1;
            continue;
        }

        indexPos = featureObj.find_first_not_of(" \t\n\r", indexPos + 1);
        if (indexPos == string::npos) {
            pos = objEnd + 1;
            continue;
        }

        size_t endPos = indexPos;
        while (endPos < featureObj.length() && isdigit(featureObj[endPos])) {
            endPos++;
        }
        string indexStr = featureObj.substr(indexPos, endPos - indexPos);
        int imageIndex = atoi(indexStr.c_str());

        if (imageIndex >= features.size()) {
            pos = objEnd + 1;
            continue;
        }

        // Parse keypoints array
        size_t keypointsStart = featureObj.find("\"keypoints\":");
        cout << "Looking for keypoints in feature object " << featureIndex << endl;
        if (keypointsStart == string::npos) {
            cout << "No keypoints found in feature object " << featureIndex << endl;
            pos = objEnd + 1;
            continue;
        }

        cout << "Found keypoints at position " << keypointsStart << endl;
        keypointsStart = featureObj.find("[", keypointsStart);
        if (keypointsStart == string::npos) {
            cout << "No keypoints array found in feature object " << featureIndex << endl;
            pos = objEnd + 1;
            continue;
        }

        cout << "Found keypoints array at position " << keypointsStart << endl;
        size_t keypointsEnd = featureObj.find("]", keypointsStart);
        if (keypointsEnd == string::npos) {
            cout << "No keypoints array end found in feature object " << featureIndex << endl;
            pos = objEnd + 1;
            continue;
        }

        cout << "Found keypoints array end at position " << keypointsEnd << endl;
        string keypointsArray = featureObj.substr(keypointsStart, keypointsEnd - keypointsStart + 1);
        cout << "Keypoints array length: " << keypointsArray.length() << endl;

        // Parse each keypoint
        size_t kpPos = 0;
        int keypointCount = 0;
        cout << "Starting to parse keypoints in feature object " << featureIndex << endl;
        while ((kpPos = keypointsArray.find("{", kpPos)) != string::npos) {
            cout << "Found keypoint at position " << kpPos << " in feature object " << featureIndex << endl;

            // Find the matching closing brace by counting nested braces
            size_t kpEnd = kpPos;
            int braceCount = 0;
            bool inString = false;
            bool escaped = false;
            bool inScientificNotation = false;

            for (size_t i = kpPos; i < keypointsArray.length(); i++) {
                char c = keypointsArray[i];

                if (escaped) {
                    escaped = false;
                    continue;
                }

                if (c == '\\') {
                    escaped = true;
                    continue;
                }

                if (c == '"') {
                    inString = !inString;
                    continue;
                }

                // Handle scientific notation (e.g., 1.23e-05)
                // Only consider 'e' as scientific notation if preceded by a digit
                if (!inString && (c == 'e' || c == 'E') && i > 0 && isdigit(keypointsArray[i-1])) {
                    inScientificNotation = true;
                    continue;
                }

                if (inScientificNotation && (c == ',' || c == ']' || c == '}' || c == ' ' || c == '\t' || c == '\n')) {
                    inScientificNotation = false;
                }

                if (!inString && !inScientificNotation) {
                    if (c == '{') {
                        braceCount++;
                        if (braceCount == 1) cout << "Found opening brace at " << i << endl;
                    } else if (c == '}') {
                        braceCount--;
                        cout << "Found closing brace at " << i << ", count=" << braceCount << endl;
                        if (braceCount == 0) {
                            kpEnd = i;
                            cout << "Found matching closing brace at " << i << endl;
                            break;
                        }
                    }
                }

                // Debug: show progress every 100 characters
                if (i % 100 == 0 && i > kpPos) {
                    cout << "Progress: i=" << i << ", braceCount=" << braceCount << ", inString=" << inString << ", inScientificNotation=" << inScientificNotation << endl;
                }
            }

            cout << "Brace counting finished. Final braceCount=" << braceCount << ", kpEnd=" << kpEnd << endl;
            if (braceCount != 0) {
                cout << "No matching closing brace found for keypoint in feature object " << featureIndex << endl;
                break;
            }

            string keypointObj = keypointsArray.substr(kpPos, kpEnd - kpPos + 1);
            cout << "Processing keypoint object: " << keypointObj.substr(0, 100) << "..." << endl;
            cout << "Keypoint object length: " << keypointObj.length() << endl;
            cout << "Keypoint object ends with: " << keypointObj.substr(keypointObj.length() - 20) << endl;

            // Parse keypoint data
            VlSiftKeypoint kp;

            // Parse x
            size_t xPos = keypointObj.find("\"x\":");
            if (xPos != string::npos) {
                xPos = keypointObj.find(":", xPos);
                if (xPos != string::npos) {
                    xPos = keypointObj.find_first_not_of(" \t\n\r", xPos + 1);
                    if (xPos != string::npos) {
                        size_t xEnd = xPos;
                        while (xEnd < keypointObj.length() && (isdigit(keypointObj[xEnd]) || keypointObj[xEnd] == '.' || keypointObj[xEnd] == '-')) {
                            xEnd++;
                        }
                        string xStr = keypointObj.substr(xPos, xEnd - xPos);
                        kp.x = atof(xStr.c_str());
                    }
                }
            }

            // Parse y
            size_t yPos = keypointObj.find("\"y\":");
            if (yPos != string::npos) {
                yPos = keypointObj.find(":", yPos);
                if (yPos != string::npos) {
                    yPos = keypointObj.find_first_not_of(" \t\n\r", yPos + 1);
                    if (yPos != string::npos) {
                        size_t yEnd = yPos;
                        while (yEnd < keypointObj.length() && (isdigit(keypointObj[yEnd]) || keypointObj[yEnd] == '.' || keypointObj[yEnd] == '-')) {
                            yEnd++;
                        }
                        string yStr = keypointObj.substr(yPos, yEnd - yPos);
                        kp.y = atof(yStr.c_str());
                    }
                }
            }

            // Parse ix
            size_t ixPos = keypointObj.find("\"ix\":");
            if (ixPos != string::npos) {
                ixPos = keypointObj.find(":", ixPos);
                if (ixPos != string::npos) {
                    ixPos = keypointObj.find_first_not_of(" \t\n\r", ixPos + 1);
                    if (ixPos != string::npos) {
                        size_t ixEnd = ixPos;
                        while (ixEnd < keypointObj.length() && isdigit(keypointObj[ixEnd])) {
                            ixEnd++;
                        }
                        string ixStr = keypointObj.substr(ixPos, ixEnd - ixPos);
                        kp.ix = atoi(ixStr.c_str());
                    }
                }
            }

            // Parse iy
            size_t iyPos = keypointObj.find("\"iy\":");
            if (iyPos != string::npos) {
                iyPos = keypointObj.find(":", iyPos);
                if (iyPos != string::npos) {
                    iyPos = keypointObj.find_first_not_of(" \t\n\r", iyPos + 1);
                    if (iyPos != string::npos) {
                        size_t iyEnd = iyPos;
                        while (iyEnd < keypointObj.length() && isdigit(keypointObj[iyEnd])) {
                            iyEnd++;
                        }
                        string iyStr = keypointObj.substr(iyPos, iyEnd - iyPos);
                        kp.iy = atoi(iyStr.c_str());
                    }
                }
            }

            // Parse sigma
            size_t sigmaPos = keypointObj.find("\"sigma\":");
            if (sigmaPos != string::npos) {
                sigmaPos = keypointObj.find(":", sigmaPos);
                if (sigmaPos != string::npos) {
                    sigmaPos = keypointObj.find_first_not_of(" \t\n\r", sigmaPos + 1);
                    if (sigmaPos != string::npos) {
                        size_t sigmaEnd = sigmaPos;
                        while (sigmaEnd < keypointObj.length() && (isdigit(keypointObj[sigmaEnd]) || keypointObj[sigmaEnd] == '.' || keypointObj[sigmaEnd] == '-')) {
                            sigmaEnd++;
                        }
                        string sigmaStr = keypointObj.substr(sigmaPos, sigmaEnd - sigmaPos);
                        kp.sigma = atof(sigmaStr.c_str());
                    }
                }
            }

            // Parse o
            size_t oPos = keypointObj.find("\"o\":");
            if (oPos != string::npos) {
                oPos = keypointObj.find(":", oPos);
                if (oPos != string::npos) {
                    oPos = keypointObj.find_first_not_of(" \t\n\r", oPos + 1);
                    if (oPos != string::npos) {
                        size_t oEnd = oPos;
                        while (oEnd < keypointObj.length() && isdigit(keypointObj[oEnd])) {
                            oEnd++;
                        }
                        string oStr = keypointObj.substr(oPos, oEnd - oPos);
                        kp.o = atoi(oStr.c_str());
                    }
                }
            }

            // Parse s
            size_t sPos = keypointObj.find("\"s\":");
            if (sPos != string::npos) {
                sPos = keypointObj.find(":", sPos);
                if (sPos != string::npos) {
                    sPos = keypointObj.find_first_not_of(" \t\n\r", sPos + 1);
                    if (sPos != string::npos) {
                        size_t sEnd = sPos;
                        while (sEnd < keypointObj.length() && isdigit(keypointObj[sEnd])) {
                            sEnd++;
                        }
                        string sStr = keypointObj.substr(sPos, sEnd - sPos);
                        kp.s = atoi(sStr.c_str());
                    }
                }
            }

            // Descriptors are no longer stored in cache
            // Create a unique descriptor vector for each keypoint using position
            vector<float> uniqueDescriptor(128, 0.0f);
            uniqueDescriptor[0] = kp.x;
            uniqueDescriptor[1] = kp.y;
            uniqueDescriptor[2] = keypointCount; // Use keypoint count as unique identifier

            // Add to features map
            features[imageIndex][uniqueDescriptor] = kp;
            keypointCount++;
            cout << "Added keypoint " << keypointCount << " to image " << imageIndex << endl;

            kpPos = kpEnd + 1;
        }
        cout << "Finished parsing keypoints for feature object " << featureIndex << ". Total keypoints: " << keypointCount << endl;

        cout << "Loaded " << features[imageIndex].size() << " features for image " << imageIndex << endl;
        featureIndex++;
        pos = objEnd + 1;
    }

    cout << "Finished parsing features array. Processed " << featureIndex << " feature objects." << endl;

    cout << "Feature cache loaded successfully" << endl;
    return true;
}

bool FeatureCache::isCacheValid(const string& cacheFilePath,
                               const vector<string>& imageFiles,
                               const StitchingProfile& profile) {
    ifstream cacheFile(cacheFilePath);
    if (!cacheFile.is_open()) {
        return false;
    }

    // Read the entire JSON file
    stringstream buffer;
    buffer << cacheFile.rdbuf();
    string json = buffer.str();
    cacheFile.close();

    // Check if cache has the expected number of images
    size_t totalImagesPos = json.find("\"totalImages\":");
    if (totalImagesPos != string::npos) {
        totalImagesPos = json.find(":", totalImagesPos);
        if (totalImagesPos != string::npos) {
            totalImagesPos = json.find_first_not_of(" \t\n\r", totalImagesPos + 1);
            if (totalImagesPos != string::npos) {
                size_t endPos = totalImagesPos;
                while (endPos < json.length() && isdigit(json[endPos])) {
                    endPos++;
                }
                string totalStr = json.substr(totalImagesPos, endPos - totalImagesPos);
                int totalImages = atoi(totalStr.c_str());

                if (totalImages != imageFiles.size()) {
                    cout << "Cache invalid: image count mismatch (cache: " << totalImages << ", expected: " << imageFiles.size() << ")" << endl;
                    return false;
                }
            }
        }
    }

    // Check if cache has the expected hFOV
    size_t hfovPos = json.find("\"hFOV\":");
    if (hfovPos != string::npos) {
        hfovPos = json.find(":", hfovPos);
        if (hfovPos != string::npos) {
            hfovPos = json.find_first_not_of(" \t\n\r", hfovPos + 1);
            if (hfovPos != string::npos) {
                size_t endPos = hfovPos;
                while (endPos < json.length() && (isdigit(json[endPos]) || json[endPos] == '.' || json[endPos] == '-')) {
                    endPos++;
                }
                string hfovStr = json.substr(hfovPos, endPos - hfovPos);
                double hfov = atof(hfovStr.c_str());

                if (abs(hfov - profile.hFOV) > 0.01) {
                    cout << "Cache invalid: hFOV mismatch (cache: " << hfov << ", expected: " << profile.hFOV << ")" << endl;
                    return false;
                }
            }
        }
    }

    return true;
}

string FeatureCache::generateCacheFilePath(const string& outputFilename) {
    // Generate cache file path based on output filename
    // For example: "ImageStitching/res/pano3.jpg" -> "ImageStitching/res/pano3_feature_match_cache.json"
    string cacheFilePath = outputFilename;

    // Replace .jpg extension with _feature_match_cache.json
    size_t lastDot = cacheFilePath.find_last_of('.');
    if (lastDot != string::npos) {
        cacheFilePath = cacheFilePath.substr(0, lastDot);
    }
    cacheFilePath += "_feature_match_cache.json";

    return cacheFilePath;
}

CachedFeatureData FeatureCache::convertToCachedFormat(const map<vector<float>, VlSiftKeypoint>& features,
                                                     int imageIndex, const string& fileName, int width, int height) {
    CachedFeatureData cachedData;
    cachedData.imageIndex = imageIndex;
    cachedData.fileName = fileName;
    cachedData.width = width;
    cachedData.height = height;

    for (const auto& pair : features) {
        // Descriptors are no longer stored in cache
        cachedData.keypoints.push_back(pair.second);
    }

    return cachedData;
}

map<vector<float>, VlSiftKeypoint> FeatureCache::convertFromCachedFormat(const CachedFeatureData& cachedData) {
    map<vector<float>, VlSiftKeypoint> features;

    for (int i = 0; i < cachedData.keypoints.size(); i++) {
        // Create unique descriptor for each keypoint using position
        vector<float> uniqueDescriptor(128, 0.0f);
        uniqueDescriptor[0] = cachedData.keypoints[i].x;
        uniqueDescriptor[1] = cachedData.keypoints[i].y;
        uniqueDescriptor[2] = i; // Use index as unique identifier
        features[uniqueDescriptor] = cachedData.keypoints[i];
    }

    return features;
}

