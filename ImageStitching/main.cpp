// ImageStitching.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"

#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <queue>
#include <algorithm>
#include "CImg.h"
#include "Projection.h"
#include "Feature.h"
#include "Match.h"
#include "Warping.h"
#include "FileReading.h"
#include "Stitching.h"
#include "ParallaxCorrection.h"

#define FILE_FOLDER "ImageStitching/dataset3/"

using namespace cimg_library;
using namespace std;

// Simple JSON parsing structures for stitching profile
// ImageProfile is now defined in ParallaxCorrection.h

struct StitchingProfile {
    string mode;
    string projection;
    double hFOV;
    int totalImages;
    double angularSpacing;
    bool enableFeatureMatching;
    bool enableFeatureVisualization;
    bool enableWarpingVisualization;
    vector<ImageProfile> images;
};

// Forward declarations
vector<vector<int>> getAdjacentImagesFromProfile(const vector<ImageProfile>& images);
struct StitchingResult {
    CImg<unsigned char> panorama;
    vector<SeamLine> seam_lines;
    vector<int> left_seam_positions;
    vector<int> right_seam_positions;
};
StitchingResult stitchingWithProfile(vector<CImg<unsigned char>> &src_imgs, const vector<ImageProfile>& images, const StitchingProfile& profile);
void generateStitchingReport(const CImg<unsigned char>& result, const vector<ImageProfile>& images, const StitchingProfile& profile, const vector<map<vector<float>, VlSiftKeypoint>>& features, const vector<SeamLine>& seam_lines, const vector<int>& left_seam_positions, const vector<int>& right_seam_positions);

// Simple JSON parser for stitching profile
class SimpleJSONParser {
public:
    static string extractValue(const string& json, const string& key) {
        string searchKey = "\"" + key + "\"";
        size_t pos = json.find(searchKey);
        if (pos == string::npos) return "";

        pos = json.find(":", pos);
        if (pos == string::npos) return "";

        pos = json.find_first_not_of(" \t\n\r", pos);
        if (pos == string::npos) return "";

        if (json[pos] == '"') {
            pos++; // Skip the opening quote
            size_t endPos = json.find("\"", pos);
            if (endPos == string::npos) return "";
            return json.substr(pos, endPos - pos);
        } else {
            // Handle non-string values
            size_t endPos = pos;
            while (endPos < json.length() && json[endPos] != ',' && json[endPos] != '}' && json[endPos] != ']') {
                endPos++;
            }
            return json.substr(pos, endPos - pos);
        }
    }

    static double extractDoubleValue(const string& json, const string& key) {
        string searchKey = "\"" + key + "\"";
        size_t pos = json.find(searchKey);
        if (pos == string::npos) return 0.0;

        pos = json.find(":", pos);
        if (pos == string::npos) return 0.0;

        pos = json.find_first_not_of(" \t\n\r", pos);
        if (pos == string::npos) return 0.0;

        size_t endPos = pos;
        while (endPos < json.length() && (isdigit(json[endPos]) || json[endPos] == '.' || json[endPos] == '-')) {
            endPos++;
        }

        string valueStr = json.substr(pos, endPos - pos);
        double result = atof(valueStr.c_str());

        return result;
    }

    static int extractIntValue(const string& json, const string& key) {
        return (int)extractDoubleValue(json, key);
    }

    static vector<ImageProfile> parseImages(const string& json) {
        vector<ImageProfile> images;

        // Find the images array
        size_t arrayStart = json.find("\"images\"");
        if (arrayStart == string::npos) return images;

        arrayStart = json.find("[", arrayStart);
        if (arrayStart == string::npos) return images;

        size_t arrayEnd = json.find("]", arrayStart);
        if (arrayEnd == string::npos) return images;

        string imagesArray = json.substr(arrayStart, arrayEnd - arrayStart + 1);

        // Parse each image object using direct string search
        size_t pos = 0;
        while ((pos = imagesArray.find("{", pos)) != string::npos) {
            size_t objEnd = imagesArray.find("}", pos);
            if (objEnd == string::npos) break;

            string imageObj = imagesArray.substr(pos, objEnd - pos + 1);

            ImageProfile img;

            // Parse index
            size_t indexPos = imageObj.find("\"index\"");
            if (indexPos != string::npos) {
                indexPos = imageObj.find(":", indexPos);
                if (indexPos != string::npos) {
                    indexPos = imageObj.find_first_not_of(" \t\n\r", indexPos + 1);
                    if (indexPos != string::npos) {
                        size_t endPos = indexPos;
                        while (endPos < imageObj.length() && isdigit(imageObj[endPos])) {
                            endPos++;
                        }
                        string indexStr = imageObj.substr(indexPos, endPos - indexPos);
                        img.index = atoi(indexStr.c_str());
                    }
                }
            }

            // Parse fileName
            size_t namePos = imageObj.find("\"fileName\":");
            if (namePos != string::npos) {
                namePos = imageObj.find("\"", namePos + 11);
                if (namePos != string::npos) {
                    size_t endPos = imageObj.find("\"", namePos + 1);
                    if (endPos != string::npos) {
                        img.fileName = imageObj.substr(namePos + 1, endPos - namePos - 1);
                    }
                }
            }

            // Parse fileNamePrefix
            size_t prefixPos = imageObj.find("\"fileNamePrefix\":");
            if (prefixPos != string::npos) {
                prefixPos = imageObj.find("\"", prefixPos + 17);
                if (prefixPos != string::npos) {
                    size_t endPos = imageObj.find("\"", prefixPos + 1);
                    if (endPos != string::npos) {
                        img.fileNamePrefix = imageObj.substr(prefixPos + 1, endPos - prefixPos - 1);
                    }
                }
            }

            // Parse radialAngle
            size_t anglePos = imageObj.find("\"radialAngle\"");
            if (anglePos != string::npos) {
                anglePos = imageObj.find(":", anglePos);
                if (anglePos != string::npos) {
                    anglePos = imageObj.find_first_not_of(" \t\n\r", anglePos + 1);
                    if (anglePos != string::npos) {
                        size_t endPos = anglePos;
                        while (endPos < imageObj.length() && (isdigit(imageObj[endPos]) || imageObj[endPos] == '.' || imageObj[endPos] == '-')) {
                            endPos++;
                        }
                        string angleStr = imageObj.substr(anglePos, endPos - anglePos);
                        img.radialAngle = atof(angleStr.c_str());
                    }
                }
            }

            // Parse rotation
            size_t rotPos = imageObj.find("\"rotation\":");
            if (rotPos != string::npos) {
                rotPos += 11;
                size_t endPos = rotPos;
                while (endPos < imageObj.length() && (isdigit(imageObj[endPos]) || imageObj[endPos] == '.' || imageObj[endPos] == '-')) {
                    endPos++;
                }
                string rotStr = imageObj.substr(rotPos, endPos - rotPos);
                img.rotation = atof(rotStr.c_str());
            }

            // Parse description
            size_t descPos = imageObj.find("\"description\":");
            if (descPos != string::npos) {
                descPos = imageObj.find("\"", descPos + 14);
                if (descPos != string::npos) {
                    size_t endPos = imageObj.find("\"", descPos + 1);
                    if (endPos != string::npos) {
                        img.description = imageObj.substr(descPos + 1, endPos - descPos - 1);
                    }
                }
            }

            images.push_back(img);
            pos = objEnd + 1;
        }

        return images;
    }

    static StitchingProfile parseProfile(const string& json) {
        StitchingProfile profile;

        // Set default values
        profile.enableFeatureMatching = true;
        profile.enableFeatureVisualization = true;
        profile.enableWarpingVisualization = true;

        // Simple direct parsing using string search
        // Parse mode
        size_t modePos = json.find("\"mode\":");
        if (modePos != string::npos) {
            modePos = json.find("\"", modePos + 7);
            if (modePos != string::npos) {
                size_t endPos = json.find("\"", modePos + 1);
                if (endPos != string::npos) {
                    profile.mode = json.substr(modePos + 1, endPos - modePos - 1);
                }
            }
        }

        // Parse projection
        size_t projPos = json.find("\"projection\":");
        if (projPos != string::npos) {
            projPos = json.find("\"", projPos + 13);
            if (projPos != string::npos) {
                size_t endPos = json.find("\"", projPos + 1);
                if (endPos != string::npos) {
                    profile.projection = json.substr(projPos + 1, endPos - projPos - 1);
                }
            }
        }

        // Parse hFOV
        size_t hfovPos = json.find("\"hFOV\"");
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
                    profile.hFOV = atof(hfovStr.c_str());
                }
            }
        }

        // Parse totalImages
        size_t totalPos = json.find("\"totalImages\"");
        if (totalPos != string::npos) {
            totalPos = json.find(":", totalPos);
            if (totalPos != string::npos) {
                totalPos = json.find_first_not_of(" \t\n\r", totalPos + 1);
                if (totalPos != string::npos) {
                    size_t endPos = totalPos;
                    while (endPos < json.length() && isdigit(json[endPos])) {
                        endPos++;
                    }
                    string totalStr = json.substr(totalPos, endPos - totalPos);
                    profile.totalImages = atoi(totalStr.c_str());
                }
            }
        }

        // Parse angular spacing from cameraLayout
        size_t spacingPos = json.find("\"spacing\"");
        if (spacingPos != string::npos) {
            spacingPos = json.find(":", spacingPos);
            if (spacingPos != string::npos) {
                spacingPos = json.find_first_not_of(" \t\n\r", spacingPos + 1);
                if (spacingPos != string::npos) {
                    size_t endPos = spacingPos;
                    while (endPos < json.length() && (isdigit(json[endPos]) || json[endPos] == '.' || json[endPos] == '-')) {
                        endPos++;
                    }
                    string spacingStr = json.substr(spacingPos, endPos - spacingPos);
                    profile.angularSpacing = atof(spacingStr.c_str());
                }
            }
        } else {
            profile.angularSpacing = 45.0; // Default fallback
        }

        // Parse enableFeatureMatching
        size_t featureMatchingPos = json.find("\"enableFeatureMatching\":");
        if (featureMatchingPos != string::npos) {
            featureMatchingPos = json.find(":", featureMatchingPos);
            if (featureMatchingPos != string::npos) {
                featureMatchingPos = json.find_first_not_of(" \t\n\r", featureMatchingPos + 1);
                if (featureMatchingPos != string::npos) {
                    string valueStr = json.substr(featureMatchingPos, 4); // "true" or "false"
                    profile.enableFeatureMatching = (valueStr == "true");
                }
            }
        }

        // Parse enableFeatureVisualization
        size_t featureVizPos = json.find("\"enableFeatureVisualization\":");
        if (featureVizPos != string::npos) {
            featureVizPos = json.find(":", featureVizPos);
            if (featureVizPos != string::npos) {
                featureVizPos = json.find_first_not_of(" \t\n\r", featureVizPos + 1);
                if (featureVizPos != string::npos) {
                    string valueStr = json.substr(featureVizPos, 4); // "true" or "false"
                    profile.enableFeatureVisualization = (valueStr == "true");
                }
            }
        }

        // Parse enableWarpingVisualization
        size_t warpingVizPos = json.find("\"enableWarpingVisualization\":");
        if (warpingVizPos != string::npos) {
            warpingVizPos = json.find(":", warpingVizPos);
            if (warpingVizPos != string::npos) {
                warpingVizPos = json.find_first_not_of(" \t\n\r", warpingVizPos + 1);
                if (warpingVizPos != string::npos) {
                    string valueStr = json.substr(warpingVizPos, 4); // "true" or "false"
                    profile.enableWarpingVisualization = (valueStr == "true");
                }
            }
        }

        // Parse images array
        profile.images = parseImages(json);

        // Debug output to verify parsing
        cout << "Parsed profile - Mode: '" << profile.mode << "', Projection: '" << profile.projection
             << "', HFOV: " << profile.hFOV << ", AngularSpacing: " << profile.angularSpacing
             << ", TotalImages: " << profile.totalImages << ", Images count: " << profile.images.size()
             << ", FeatureMatching: " << (profile.enableFeatureMatching ? "enabled" : "disabled")
             << ", FeatureVisualization: " << (profile.enableFeatureVisualization ? "enabled" : "disabled")
             << ", WarpingVisualization: " << (profile.enableWarpingVisualization ? "enabled" : "disabled") << endl;

        for (int i = 0; i < profile.images.size() && i < 3; i++) {
            cout << "  Image " << i << ": " << profile.images[i].fileName
                 << " (angle: " << profile.images[i].radialAngle << "°)" << endl;
        }

        return profile;
    }
};

// Function to load stitching profile
StitchingProfile loadStitchingProfile(const string& folderPath) {
    string profilePath = folderPath + "stitching-profile.json";
    ifstream file(profilePath);

    if (!file.is_open()) {
        cout << "No stitching-profile.json found, using default settings" << endl;
        return StitchingProfile();
    }

    cout << "Loading stitching profile from: " << profilePath << endl;

    // Read the entire JSON file
    stringstream buffer;
    buffer << file.rdbuf();
    string json = buffer.str();
    file.close();

    // Parse the JSON using our simple parser
    StitchingProfile profile = SimpleJSONParser::parseProfile(json);

    cout << "Profile loaded - Mode: " << profile.mode
         << ", Projection: " << profile.projection
         << ", HFOV: " << profile.hFOV
         << ", Images: " << profile.images.size() << endl;

    return profile;
}

// Function to determine adjacency based on radial angles
vector<vector<int>> getAdjacentImagesFromProfile(const vector<ImageProfile>& images) {
    vector<vector<int>> adjacent_images(images.size());

    for (int i = 0; i < images.size(); i++) {
        for (int j = 0; j < images.size(); j++) {
            if (i == j) continue;

            // Calculate angular difference
            double angle1 = images[i].radialAngle;
            double angle2 = images[j].radialAngle;

            // Normalize angles to 0-360 range
            while (angle1 < 0) angle1 += 360;
            while (angle1 >= 360) angle1 -= 360;
            while (angle2 < 0) angle2 += 360;
            while (angle2 >= 360) angle2 -= 360;

            // Calculate the smaller angular difference
            double diff = abs(angle1 - angle2);
            if (diff > 180) diff = 360 - diff;

            // Consider images adjacent if they are within 50 degrees of each other
            // This ensures only truly adjacent images are considered for stitching
            if (diff <= 50.0) {
                adjacent_images[i].push_back(j);
                cout << "Image " << i << " (angle: " << angle1 << "°) and "
                     << j << " (angle: " << angle2 << "°) are adjacent (diff: " << diff << "°)" << endl;
            }
        }
    }

    return adjacent_images;
}

// Geometric stitching function for 360-degree panoramas
StitchingResult stitchingWithProfile(vector<CImg<unsigned char>> &src_imgs, const vector<ImageProfile>& images, const StitchingProfile& profile) {
    cout << "Using geometric 360-degree panoramic stitching with parallax correction..." << endl;

    int num_images = src_imgs.size();
    if (num_images == 0) {
        cout << "No images to stitch!" << endl;
        StitchingResult result;
        result.panorama = CImg<unsigned char>();
        result.seam_lines = vector<SeamLine>();
        result.left_seam_positions = vector<int>();
        result.right_seam_positions = vector<int>();
        return result;
    }

    // Get values from profile instead of hardcoding
    double hFOV_degrees = profile.hFOV;
    double angular_spacing = profile.angularSpacing;
    double overlap_degrees = hFOV_degrees - angular_spacing;

    cout << "hFOV: " << hFOV_degrees << "°, Angular spacing: " << angular_spacing << "°" << endl;
    cout << "Overlap between adjacent images: " << overlap_degrees << "°" << endl;

    // Calculate output dimensions and seam positions first
    int base_width = src_imgs[0].width();
    int base_height = src_imgs[0].height();
    double effective_degrees_per_image = angular_spacing;  // Each image's unique contribution
    double scale_factor = effective_degrees_per_image / hFOV_degrees;
    int output_width = (int)(base_width * scale_factor * num_images);
    int output_height = base_height;  // Keep original height

    cout << "Effective degrees per image (unique): " << effective_degrees_per_image << "°" << endl;
    cout << "Scale factor: " << scale_factor << endl;
    cout << "Output dimensions: " << output_width << " x " << output_height << endl;

    // Calculate seam positions based on angular overlap
    vector<SeamLine> seam_lines;

    // Calculate seam positions once at the beginning
    double seam_offset_degrees = overlap_degrees / 2.0; // 9.52° from center
    double seam_offset_ratio = seam_offset_degrees / hFOV_degrees; // 9.52° / 64.04° = 0.1486
    int seam_pixel_offset = (int)(base_width * seam_offset_ratio); // Position from center

    cout << "Overlap: " << overlap_degrees << "°, Seam offset: " << seam_offset_degrees
         << "°, Pixel offset: " << seam_pixel_offset << " pixels" << endl;

    // Store seam positions for each image
    vector<int> left_seam_positions(num_images);
    vector<int> right_seam_positions(num_images);

    for (int i = 0; i < num_images; i++) {
        left_seam_positions[i] = seam_pixel_offset; // Left seam at 570
        right_seam_positions[i] = base_width - seam_pixel_offset; // Right seam at 3270
    }

    // Create seam lines using the calculated positions
    for (int i = 0; i < num_images; i++) {
        int next_i = (i + 1) % num_images;
        int prev_i = (i - 1 + num_images) % num_images;

        // Create seam line for right seam (current image meets next image)
        SeamLine right_seam;
        right_seam.image1_index = i;
        right_seam.image2_index = next_i;
        right_seam.confidence = 1.0;
        for (int y = 0; y < base_height; y += 10) {
            right_seam.seam_points.push_back(make_pair(right_seam_positions[i], y));
        }
        seam_lines.push_back(right_seam);

        // Create seam line for left seam (previous image meets current image)
        SeamLine left_seam;
        left_seam.image1_index = prev_i;
        left_seam.image2_index = i;
        left_seam.confidence = 1.0;
        for (int y = 0; y < base_height; y += 10) {
            left_seam.seam_points.push_back(make_pair(left_seam_positions[i], y));
        }
        seam_lines.push_back(left_seam);

        cout << "Image " << i << " - Right seam at pixel " << right_seam_positions[i]
             << " (meets image " << next_i << "), Left seam at pixel " << left_seam_positions[i]
             << " (meets image " << prev_i << ")" << endl;
    }

    // Apply parallax correction using the calculated seam lines
    cout << "Applying parallax correction with calculated seam lines..." << endl;

    vector<map<vector<float>, VlSiftKeypoint>> features_for_parallax(num_images);
    vector<CImg<unsigned char>> corrected_imgs;

    if (profile.enableFeatureMatching) {
        // Extract features for parallax correction
        for (int i = 0; i < num_images; i++) {
            CImg<unsigned char> gray = get_gray_image(src_imgs[i]);
            features_for_parallax[i] = getFeatureFromImage(gray);
            cout << "Extracted " << features_for_parallax[i].size() << " features from image " << i
                 << " (" << src_imgs[i].width() << "x" << src_imgs[i].height() << ")" << endl;
        }

        // Test feature matching between adjacent images with distance constraint
        cout << "\nTesting feature matching between adjacent images with distance constraint:" << endl;
        double overlap_ratio = (profile.hFOV - profile.angularSpacing) / profile.hFOV;
        cout << "Overlap ratio: " << overlap_ratio << " (max distance constraint applied)" << endl;

        for (int i = 0; i < num_images; i++) {
            int next_i = (i + 1) % num_images;
            vector<point_pair> test_pairs = getPointPairsFromFeatureWithDistanceConstraint(
                features_for_parallax[i],
                features_for_parallax[next_i],
                src_imgs[i].width(),
                src_imgs[i].height(),
                overlap_ratio
            );
            cout << "Images " << i << " <-> " << next_i << ": " << test_pairs.size() << " feature pairs" << endl;
        }

        // Apply parallax correction using calculated seam lines
        corrected_imgs = ParallaxCorrection::correctParallaxWithProfile(
            src_imgs, images, features_for_parallax
        );

        if (profile.enableWarpingVisualization) {
            cout << "Creating warping visualization..." << endl;
            CImg<unsigned char> warping_viz = ParallaxCorrection::createWarpingVisualization(
                src_imgs, images, seam_lines, features_for_parallax
            );

            // Save warping visualization
            if (!warping_viz.is_empty()) {
                warping_viz.save("ImageStitching/res/pano3_warping_visualization.jpg");
                cout << "Warping visualization saved to pano3_warping_visualization.jpg" << endl;
            }

        } else {
            cout << "Warping visualization disabled, skipping..." << endl;
        }

        // Create feature visualization if enabled
        if (profile.enableFeatureVisualization) {
            cout << "Creating feature visualization..." << endl;
            CImg<unsigned char> feature_viz = ParallaxCorrection::createFeatureVisualization(
                src_imgs, images, seam_lines, features_for_parallax
            );

            // Save feature visualization
            if (!feature_viz.is_empty()) {
                feature_viz.save("ImageStitching/res/pano3_feature_matches.jpg");
                cout << "Feature visualization saved to pano3_feature_matches.jpg" << endl;
            }
        } else {
            cout << "Feature visualization disabled, skipping..." << endl;
        }

        // Use corrected images for the rest of the stitching process
        src_imgs = corrected_imgs;
        cout << "Parallax correction completed, proceeding with geometric stitching..." << endl;
    } else {
        cout << "Feature matching disabled, skipping parallax correction..." << endl;
        // Use original images without correction
        corrected_imgs = src_imgs;
    }

    // Output dimensions already calculated above

    // Create the output image
    CImg<unsigned char> panorama(output_width, output_height, 1, src_imgs[0].spectrum(), 0);

    // Calculate the width each image should occupy in the output (for positioning only)
    int image_width = output_width / num_images;

    cout << "Each image will occupy " << image_width << " pixels width in output" << endl;

    // Position each image according to its radial angle (keeping original size)
    for (int i = 0; i < num_images; i++) {
        double angle = images[i].radialAngle;

        // Calculate the horizontal position based on angle
        int x_offset = (int)((angle / 360.0) * output_width);

        cout << "Positioning image " << i << " (angle: " << angle << "°) at x=" << x_offset << endl;

        // Copy the original image (no resizing) to the panorama
        for (int y = 0; y < output_height && y < src_imgs[i].height(); y++) {
            for (int x = 0; x < base_width && x < src_imgs[i].width(); x++) {
                int target_x = x_offset + x;
                if (target_x >= 0 && target_x < output_width) {
                    for (int c = 0; c < panorama.spectrum() && c < src_imgs[i].spectrum(); c++) {
                        panorama(target_x, y, 0, c) = src_imgs[i](x, y, 0, c);
                    }
                }
            }
        }
    }

    cout << "Geometric positioning completed!" << endl;

    // Now apply blending at the seam lines
    cout << "Applying seam blending..." << endl;

    // Blend at the boundaries between images using the pre-calculated seam lines
    for (int i = 0; i < num_images; i++) {
        int x_offset = (int)((images[i].radialAngle / 360.0) * output_width);

        // Blend with the next image (wrapping around)
        int next_i = (i + 1) % num_images;
        int next_x_offset = (int)((images[next_i].radialAngle / 360.0) * output_width);

            // Calculate the actual overlap region where both images have content
            int overlap_start = max(x_offset, next_x_offset);
            int overlap_end = min(x_offset + base_width, next_x_offset + base_width);

            // Only proceed if there's actually an overlap
            if (overlap_start < overlap_end) {
                // Use the centralized seam position from the right seam of current image
                int seam_x_in_source = right_seam_positions[i];
                int seam_x = x_offset + seam_x_in_source;

            // Use the full overlap region for blending
            int blend_width = overlap_end - overlap_start;  // Blend over the entire overlap region

            // Apply linear blending across the entire overlap region
            int blend_start = overlap_start;
            int blend_end = overlap_end;

            cout << "Blending seam between images " << i << " and " << next_i
                 << " at x=" << seam_x << " (final render image overlap: " << blend_start << "-" << blend_end
                 << ", blend_width: " << blend_width << ")" << endl;

            for (int y = 0; y < output_height; y++) {
                for (int x = blend_start; x < blend_end; x++) {
                    // Calculate blend factor based on position relative to seam
                    // blend_factor represents the next image's contribution (0.0 = only current, 1.0 = only next)
                    // For current image: fade from 100% at left edge to 50% at seam to 0% at right edge
                    // For next image: fade from 0% at left edge to 50% at seam to 100% at right edge
                    double blend_factor;
					blend_factor = 1.0 * (double)(x - blend_start) / (blend_end - blend_start);
                    blend_factor = max(0.0, min(1.0, blend_factor));

                    // Get the color from the current image at this panorama position
                    unsigned char current_color[3] = {0, 0, 0};
                    int current_x = x - x_offset;
                    if (current_x >= 0 && current_x < base_width) {
                        for (int c = 0; c < panorama.spectrum(); c++) {
                            current_color[c] = src_imgs[i](current_x, y, 0, c);
                        }
                    }

                    // Get the color from the next image at this panorama position
                    unsigned char next_color[3] = {0, 0, 0};
                    int next_x = x - next_x_offset;
                    if (next_x >= 0 && next_x < base_width) {
                        for (int c = 0; c < panorama.spectrum(); c++) {
                            next_color[c] = src_imgs[next_i](next_x, y, 0, c);
                        }
                    }

                    // Only blend if both images have valid pixels at this location
                    bool current_valid = (current_x >= 0 && current_x < base_width);
                    bool next_valid = (next_x >= 0 && next_x < base_width);

                    if (current_valid && next_valid) {
                        // Blend the colors: current image fades out, next image fades in
                        for (int c = 0; c < panorama.spectrum(); c++) {
                            unsigned char blended_color = (unsigned char)(current_color[c] * (1.0 - blend_factor) + next_color[c] * blend_factor);
                            panorama(x, y, 0, c) = blended_color;
                        }
                    } else if (current_valid) {
                        // Only current image has valid data
                        for (int c = 0; c < panorama.spectrum(); c++) {
                            panorama(x, y, 0, c) = current_color[c];
                        }
                    } else if (next_valid) {
                        // Only next image has valid data
                        for (int c = 0; c < panorama.spectrum(); c++) {
                            panorama(x, y, 0, c) = next_color[c];
                        }
                    }
                }
            }

            cout << "Blended seam between images " << i << " and " << next_i << " (blended " << (blend_end - blend_start) << " pixels)" << endl;
        } else {
            cout << "No overlap between images " << i << " and " << next_i << " - skipping blending" << endl;
        }
    }

    cout << "360-degree panoramic stitching completed!" << endl;
    StitchingResult result;
    result.panorama = panorama;
    result.seam_lines = seam_lines;
    result.left_seam_positions = left_seam_positions;
    result.right_seam_positions = right_seam_positions;
    return result;
}

int main(int argc, char **argv) {

	string file_folder(FILE_FOLDER);
	vector<string> image_files;

	cout << "Getting files from: " << file_folder << endl;
	getAllFiles(file_folder, image_files);
	cout << "Found " << image_files.size() << " files" << endl;

	// Load stitching profile if available
	StitchingProfile profile = loadStitchingProfile(file_folder);

	vector<string> image_files_filtered;
	vector<ImageProfile> ordered_images;

	if (profile.images.size() > 0) {
		// Use profile to order images
		cout << "Using stitching profile to order images..." << endl;

		// Create a map for quick lookup
		map<string, ImageProfile> profileMap;
		for (const auto& img : profile.images) {
			profileMap[img.fileName] = img;
		}

		// Filter and order images according to profile
		for (int i = 0; i < image_files.size(); i++) {
			string filename = image_files[i];
			if (filename.length() >= 4 &&
				(filename.substr(filename.length() - 4) == ".jpg" ||
				 filename.substr(filename.length() - 4) == ".JPG" ||
				 filename.substr(filename.length() - 4) == ".jpeg" ||
				 filename.substr(filename.length() - 4) == ".JPEG")) {

				// Extract just the filename from the full path
				size_t lastSlash = filename.find_last_of("/\\");
				string justFilename = (lastSlash != string::npos) ? filename.substr(lastSlash + 1) : filename;

				if (profileMap.find(justFilename) != profileMap.end()) {
					image_files_filtered.push_back(filename);
					ordered_images.push_back(profileMap[justFilename]);
					cout << "Profile image " << profileMap[justFilename].index
						 << ": " << justFilename
						 << " (angle: " << profileMap[justFilename].radialAngle << "°)" << endl;
				}
			}
		}

		// Sort by index to maintain proper order
		vector<pair<int, int>> indexed_positions;
		for (int i = 0; i < image_files_filtered.size(); i++) {
			indexed_positions.push_back(make_pair(ordered_images[i].index, i));
		}
		sort(indexed_positions.begin(), indexed_positions.end());

		// Rebuild ordered arrays
		vector<string> temp_files = image_files_filtered;
		vector<ImageProfile> temp_profiles = ordered_images;
		image_files_filtered.clear();
		ordered_images.clear();

		for (const auto& item : indexed_positions) {
			image_files_filtered.push_back(temp_files[item.second]);
			ordered_images.push_back(temp_profiles[item.second]);
		}

		cout << "Ordered " << image_files_filtered.size() << " images according to profile" << endl;
	} else {
		// Fallback to original filtering
		cout << "No profile found, using default image ordering..." << endl;
		for (int i = 0; i < image_files.size(); i++) {
			string filename = image_files[i];
			if (filename.length() >= 4 &&
				(filename.substr(filename.length() - 4) == ".jpg" ||
				 filename.substr(filename.length() - 4) == ".JPG" ||
				 filename.substr(filename.length() - 4) == ".jpeg" ||
				 filename.substr(filename.length() - 4) == ".JPEG")) {
				image_files_filtered.push_back(filename);
				cout << "Image file: " << filename << endl;
			}
		}
	}

	cout << "Filtered to " << image_files_filtered.size() << " image files" << endl;

	vector<CImg<unsigned char>> src_imgs(image_files_filtered.size());
	vector<map<vector<float>, VlSiftKeypoint>> features(image_files_filtered.size());

	cout << "Loading images..." << endl;
	for (int i = 0; i < image_files_filtered.size(); i++) {
		cout << "Loading image " << i << ": " << image_files_filtered[i] << endl;
		src_imgs[i] = CImg<unsigned char>(image_files_filtered[i].c_str());
		cout << "Image " << i << " loaded successfully";
		if (ordered_images.size() > i) {
			cout << " (angle: " << ordered_images[i].radialAngle << "°)";
		}
		cout << endl;
	}

	cout << "Starting stitching process..." << endl;
	if (profile.mode == "360-pano") {
		cout << "Using 360-degree panoramic stitching mode" << endl;
		cout << "Projection: " << profile.projection << ", HFOV: " << profile.hFOV << "°" << endl;
	}

	// Extract features for report generation (only if feature matching is enabled)
	vector<map<vector<float>, VlSiftKeypoint>> features_for_report;
	if (profile.images.size() > 0 && profile.enableFeatureMatching) {
		features_for_report.resize(src_imgs.size());
		for (int i = 0; i < src_imgs.size(); i++) {
			CImg<unsigned char> gray = get_gray_image(src_imgs[i]);
			features_for_report[i] = getFeatureFromImage(gray);
		}
		cout << "Extracted features for report generation" << endl;
	} else if (profile.images.size() > 0) {
		cout << "Feature matching disabled, skipping feature extraction for report" << endl;
	}

	CImg<unsigned char> res;
	vector<SeamLine> seam_lines;
	vector<int> left_seam_positions, right_seam_positions;
	if (profile.images.size() > 0) {
		// Use geometric profile-based stitching (no feature detection needed)
		cout << "Using geometric profile-based stitching..." << endl;
		auto result = stitchingWithProfile(src_imgs, ordered_images, profile);
		res = result.panorama;
		seam_lines = result.seam_lines;
		left_seam_positions = result.left_seam_positions;
		right_seam_positions = result.right_seam_positions;
	} else {
		// Use traditional feature-based stitching with cylinder projection
		cout << "Using traditional feature-based stitching..." << endl;
		res = stitching(src_imgs);
		// For traditional stitching, create empty seam lines
		seam_lines = vector<SeamLine>();
		left_seam_positions = vector<int>();
		right_seam_positions = vector<int>();
	}
	cout << "Stitching completed successfully!" << endl;

	// res.display(); // Display disabled for headless operation
	cout << "Saving result..." << endl;

	// Generate unique filename with incremental suffix if file exists
	string base_filename = "ImageStitching/res/pano3.jpg";
	string filename = base_filename;
	int counter = 0;

	// Check if file exists and generate unique name
	while (true) {
		ifstream file_check(filename);
		if (!file_check.good()) {
			// File doesn't exist, we can use this name
			break;
		}
		file_check.close();

		// File exists, generate new name with counter
		counter++;
		size_t dot_pos = base_filename.find_last_of(".");
		if (dot_pos != string::npos) {
			string name_part = base_filename.substr(0, dot_pos);
			string ext_part = base_filename.substr(dot_pos);
			filename = name_part + "_" + to_string(counter).insert(0, 3 - to_string(counter).length(), '0') + ext_part;
		} else {
			filename = base_filename + "_" + to_string(counter).insert(0, 3 - to_string(counter).length(), '0');
		}
	}

	// Draw control points on the final image if using profile-based stitching
	if (profile.images.size() > 0) {
		cout << "Drawing control points on final image..." << endl;
		vector<WarpControlPoint> all_control_points = ParallaxCorrection::collectAllControlPoints(
			src_imgs, profile.images, features_for_report
		);
		ParallaxCorrection::drawControlPointsOnImage(res, all_control_points);
	}

	res.save(filename.c_str());
	cout << "Result saved to " << filename << endl;

	// Generate comprehensive stitching report
	cout << "Generating stitching report..." << endl;
	generateStitchingReport(res, ordered_images, profile, features_for_report, seam_lines, left_seam_positions, right_seam_positions);

	return 0;
}

// Generate comprehensive stitching report
void generateStitchingReport(const CImg<unsigned char>& result, const vector<ImageProfile>& images, const StitchingProfile& profile, const vector<map<vector<float>, VlSiftKeypoint>>& features, const vector<SeamLine>& seam_lines, const vector<int>& left_seam_positions, const vector<int>& right_seam_positions) {
    ofstream report_file("ImageStitching/res/pano3_report.json");

    if (!report_file.is_open()) {
        cout << "Error: Could not create report file" << endl;
        return;
    }

    report_file << "{\n";
    report_file << "  \"stitchingReport\": {\n";

    // Overall image dimensions
    report_file << "    \"overallDimensions\": {\n";
    report_file << "      \"width\": " << result.width() << ",\n";
    report_file << "      \"height\": " << result.height() << ",\n";
    report_file << "      \"channels\": " << result.spectrum() << "\n";
    report_file << "    },\n";

    // Number of images and seam lines
    report_file << "    \"imageCount\": " << images.size() << ",\n";
    report_file << "    \"seamLineCount\": " << seam_lines.size() << ",\n";

    // Overlap information
    double overlap_degrees = profile.hFOV - profile.angularSpacing;
    double overlap_ratio = overlap_degrees / profile.hFOV;
    report_file << "    \"overlapInfo\": {\n";
    report_file << "      \"overlapDegrees\": " << overlap_degrees << ",\n";
    report_file << "      \"overlapRatio\": " << overlap_ratio << ",\n";
    report_file << "      \"angularSpacing\": " << profile.angularSpacing << ",\n";
    report_file << "      \"horizontalFieldOfView\": " << profile.hFOV << "\n";
    report_file << "    },\n";

    // Image-by-image seam line positions
    report_file << "    \"imageSeamPositions\": [\n";
    for (int i = 0; i < images.size(); i++) {
        report_file << "      {\n";
        report_file << "        \"imageIndex\": " << i << ",\n";
        report_file << "        \"fileName\": \"" << images[i].fileName << "\",\n";
        report_file << "        \"radialAngle\": " << images[i].radialAngle << ",\n";
        report_file << "        \"description\": \"" << images[i].description << "\",\n";

        // Use the centralized seam positions
        int left_seam_x = left_seam_positions[i];
        int right_seam_x = right_seam_positions[i];

        // Left seam line (from previous image)
        report_file << "        \"leftSeam\": {\n";
        report_file << "          \"xPosition\": " << left_seam_x << ",\n";
        report_file << "          \"confidence\": 1.0,\n";
        report_file << "          \"pointCount\": 216\n";
        report_file << "        },\n";

        // Right seam line (to next image)
        report_file << "        \"rightSeam\": {\n";
        report_file << "          \"xPosition\": " << right_seam_x << ",\n";
        report_file << "          \"confidence\": 1.0,\n";
        report_file << "          \"pointCount\": 216\n";
        report_file << "        },\n";

        // Feature counts around seam lines
        int left_seam_features = 0, right_seam_features = 0;

        // Count features around left seam (only if features are available)
        if (i > 0 && !features.empty()) {
            int prev_i = (i - 1 + images.size()) % images.size();
            // Create a dummy seam line for feature counting
            SeamLine dummy_left_seam;
            dummy_left_seam.image1_index = prev_i;
            dummy_left_seam.image2_index = i;
            dummy_left_seam.confidence = 1.0;
            for (int y = 0; y < 2160; y += 10) {
                dummy_left_seam.seam_points.push_back(make_pair(left_seam_x, y));
            }

            vector<point_pair> left_features = ParallaxCorrection::findSeamFeaturesInOverlap(
                CImg<unsigned char>(), CImg<unsigned char>(), // Dummy images for now
                features[prev_i], features[i], dummy_left_seam
            );
            left_seam_features = left_features.size();
        }

        // Count features around right seam (only if features are available)
        if (i < images.size() - 1 && !features.empty()) {
            int next_i = (i + 1) % images.size();
            // Create a dummy seam line for feature counting
            SeamLine dummy_right_seam;
            dummy_right_seam.image1_index = i;
            dummy_right_seam.image2_index = next_i;
            dummy_right_seam.confidence = 1.0;
            for (int y = 0; y < 2160; y += 10) {
                dummy_right_seam.seam_points.push_back(make_pair(right_seam_x, y));
            }

            vector<point_pair> right_features = ParallaxCorrection::findSeamFeaturesInOverlap(
                CImg<unsigned char>(), CImg<unsigned char>(), // Dummy images for now
                features[i], features[next_i], dummy_right_seam
            );
            right_seam_features = right_features.size();
        }

        // Calculate blend regions for this image
        int base_width = 3840; // Standard image width
        int x_offset = (int)((images[i].radialAngle / 360.0) * result.width());

        // Calculate overlap regions with adjacent images
        int left_blend_start = 0, left_blend_end = 0;
        int right_blend_start = 0, right_blend_end = 0;

        // Left blend region (with previous image)
        if (i > 0) {
            int prev_i = (i - 1 + images.size()) % images.size();
            int prev_x_offset = (int)((images[prev_i].radialAngle / 360.0) * result.width());

            // Calculate the overlap region between current image and previous image
            left_blend_start = max(prev_x_offset, x_offset);
            left_blend_end = min(prev_x_offset + base_width, x_offset + base_width);

            // Ensure valid blend region
            if (left_blend_start >= left_blend_end) {
                left_blend_start = 0;
                left_blend_end = 0;
            }
        }

        // Right blend region (with next image)
        if (i < images.size() - 1) {
            int next_i = (i + 1) % images.size();
            int next_x_offset = (int)((images[next_i].radialAngle / 360.0) * result.width());

            // Calculate the overlap region between current image and next image
            right_blend_start = max(x_offset, next_x_offset);
            right_blend_end = min(x_offset + base_width, next_x_offset + base_width);

            // Ensure valid blend region
            if (right_blend_start >= right_blend_end) {
                right_blend_start = 0;
                right_blend_end = 0;
            }
        }

        report_file << "        \"featureCounts\": {\n";
        report_file << "          \"leftSeamFeatures\": " << left_seam_features << ",\n";
        report_file << "          \"rightSeamFeatures\": " << right_seam_features << ",\n";
        report_file << "          \"totalFeatures\": " << (features.empty() ? 0 : features[i].size()) << "\n";
        report_file << "        },\n";

        // Add blend region information
        report_file << "        \"blendRegions\": {\n";
        report_file << "          \"leftBlend\": {\n";
        report_file << "            \"start\": " << left_blend_start << ",\n";
        report_file << "            \"end\": " << left_blend_end << ",\n";
        report_file << "            \"width\": " << (left_blend_end - left_blend_start) << "\n";
        report_file << "          },\n";
        report_file << "          \"rightBlend\": {\n";
        report_file << "            \"start\": " << right_blend_start << ",\n";
        report_file << "            \"end\": " << right_blend_end << ",\n";
        report_file << "            \"width\": " << (right_blend_end - right_blend_start) << "\n";
        report_file << "          }\n";
        report_file << "        }\n";

        report_file << "      }";
        if (i < images.size() - 1) report_file << ",";
        report_file << "\n";
    }
    report_file << "    ],\n";

    // Detailed seam line analysis
    report_file << "    \"seamLineAnalysis\": [\n";
    for (int i = 0; i < seam_lines.size(); i++) {
        const auto& seam = seam_lines[i];
        report_file << "      {\n";
        report_file << "        \"seamIndex\": " << i << ",\n";
        report_file << "        \"image1Index\": " << seam.image1_index << ",\n";
        report_file << "        \"image2Index\": " << seam.image2_index << ",\n";
        report_file << "        \"xPosition\": " << seam.seam_points[0].first << ",\n";
        report_file << "        \"confidence\": " << seam.confidence << ",\n";
        report_file << "        \"pointCount\": " << seam.seam_points.size() << ",\n";

        // Count features around this seam (only if features are available)
        int seam_features = 0;
        if (!features.empty() && seam.image1_index < features.size() && seam.image2_index < features.size()) {
            vector<point_pair> seam_features_vec = ParallaxCorrection::findSeamFeaturesInOverlap(
                CImg<unsigned char>(), CImg<unsigned char>(), // Dummy images
                features[seam.image1_index], features[seam.image2_index], seam
            );
            seam_features = seam_features_vec.size();
        }

        report_file << "        \"featureMatches\": " << seam_features << "\n";
        report_file << "      }";
        if (i < seam_lines.size() - 1) report_file << ",";
        report_file << "\n";
    }
    report_file << "    ],\n";

    // Summary statistics
    int total_features = 0;
    if (!features.empty()) {
        for (const auto& feature_map : features) {
            total_features += feature_map.size();
        }
    }

    int total_seam_features = 0;
    if (!features.empty()) {
        for (const auto& seam : seam_lines) {
            if (seam.image1_index < features.size() && seam.image2_index < features.size()) {
                vector<point_pair> seam_features_vec = ParallaxCorrection::findSeamFeaturesInOverlap(
                    CImg<unsigned char>(), CImg<unsigned char>(), // Dummy images
                    features[seam.image1_index], features[seam.image2_index], seam
                );
                total_seam_features += seam_features_vec.size();
            }
        }
    }

    report_file << "    \"summaryStatistics\": {\n";
    report_file << "      \"totalFeatures\": " << total_features << ",\n";
    report_file << "      \"averageFeaturesPerImage\": " << (total_features / images.size()) << ",\n";
    report_file << "      \"totalSeamFeatures\": " << total_seam_features << ",\n";
    report_file << "      \"averageSeamFeatures\": " << (seam_lines.empty() ? 0 : total_seam_features / seam_lines.size()) << "\n";
    report_file << "    }\n";

    report_file << "  }\n";
    report_file << "}\n";

    report_file.close();
    cout << "Stitching report saved to pano3_report.json" << endl;
}



