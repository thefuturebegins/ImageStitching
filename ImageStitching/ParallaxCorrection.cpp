#include "stdafx.h"
#include "ParallaxCorrection.h"
#include "Interpolation.h"
#include <algorithm>
#include <queue>

// Detect seam lines between adjacent images
vector<SeamLine> ParallaxCorrection::detectSeamLines(
    const vector<CImg<unsigned char>>& images,
    const vector<ImageProfile>& image_profiles,
    const vector<map<vector<float>, VlSiftKeypoint>>& features) {

    vector<SeamLine> seam_lines;

    cout << "Detecting seam lines for parallax correction..." << endl;

    for (int i = 0; i < images.size(); i++) {
        for (int j = i + 1; j < images.size(); j++) {
            // Check if images are adjacent (within 50 degrees)
            double angle1 = image_profiles[i].radialAngle;
            double angle2 = image_profiles[j].radialAngle;

            // Normalize angles
            while (angle1 < 0) angle1 += 360;
            while (angle2 < 0) angle2 += 360;
            while (angle1 >= 360) angle1 -= 360;
            while (angle2 >= 360) angle2 -= 360;

            double diff = abs(angle1 - angle2);
            if (diff > 180) diff = 360 - diff;

            if (diff <= 50.0) {  // Adjacent images
                cout << "Detecting seam between images " << i << " and " << j << endl;

                SeamLine seam;
                seam.image1_index = i;
                seam.image2_index = j;
                seam.confidence = 0.0;

                // Find features between the two images with distance constraint
                // Calculate overlap ratio for distance constraint
                double overlap_ratio = 0.297; // 19.04°/64.04° from stitching profile
                vector<point_pair> pairs = getPointPairsFromFeatureWithDistanceConstraint(features[i], features[j], images[i].width(), images[i].height(), overlap_ratio);
                cout << "Found " << pairs.size() << " feature pairs between images " << i << " and " << j << endl;

                if (pairs.size() >= 5) {  // Lower threshold for 8-camera setup
                    // Calculate seam line based on feature matches
                    // For 360-degree panoramas, the seam is typically vertical
                    int img_width = images[i].width();
                    int img_height = images[i].height();

                    // For 8-camera setup, estimate seam position based on angular difference
                    // Adjacent cameras are 45 degrees apart, so seam should be at the edge
                    int seam_x = img_width - 50;  // Start near the right edge for adjacent images

                    // Refine seam position based on feature matches
                    vector<int> seam_candidates;
                    for (const auto& pair : pairs) {
                        int x1 = (int)pair.a.x;
                        int x2 = (int)pair.b.x;
                        // For adjacent images, use the rightmost feature as seam indicator
                        int seam_candidate = max(x1, x2);
                        seam_candidates.push_back(seam_candidate);
                    }

                    if (!seam_candidates.empty()) {
                        // Use median as seam position, but ensure it's near the edge
                        sort(seam_candidates.begin(), seam_candidates.end());
                        seam_x = seam_candidates[seam_candidates.size() / 2];
                        // Ensure seam is in the overlap region (last 20% of image)
                        seam_x = max(seam_x, (int)(img_width * 0.8));
                        seam_x = min(seam_x, img_width - 10);
                    }

                    // Generate seam points (vertical line)
                    for (int y = 0; y < img_height; y += 10) {  // Sample every 10 pixels
                        seam.seam_points.push_back(make_pair(seam_x, y));
                    }

                    // Calculate confidence based on feature density near seam
                    int features_near_seam = 0;
                    for (const auto& pair : pairs) {
                        int x1 = (int)pair.a.x;
                        int x2 = (int)pair.b.x;
                        if (abs(x1 - seam_x) < SEAM_DETECTION_WINDOW ||
                            abs(x2 - seam_x) < SEAM_DETECTION_WINDOW) {
                            features_near_seam++;
                        }
                    }

                    seam.confidence = (double)features_near_seam / pairs.size();

                    // Lower confidence threshold for 8-camera setup
                    if (seam.confidence > 0.3 || pairs.size() >= 10) {
                        seam_lines.push_back(seam);
                        cout << "Seam detected between images " << i << " and " << j
                             << " at x=" << seam_x << " with confidence=" << seam.confidence << endl;
                    } else {
                        cout << "Seam rejected between images " << i << " and " << j
                             << " (confidence=" << seam.confidence << ", features=" << pairs.size() << ")" << endl;
                    }
                }
            }
        }
    }

    cout << "Detected " << seam_lines.size() << " seam lines" << endl;
    return seam_lines;
}

// Find features near seam lines for parallax correction
vector<point_pair> ParallaxCorrection::findSeamFeatures(
    const CImg<unsigned char>& img1,
    const CImg<unsigned char>& img2,
    const map<vector<float>, VlSiftKeypoint>& features1,
    const map<vector<float>, VlSiftKeypoint>& features2,
    const SeamLine& seam_line) {

    vector<point_pair> seam_features;

    // Get all feature pairs between the two images with distance constraint
    double overlap_ratio = 0.297; // 19.04°/64.04° from stitching profile
    vector<point_pair> all_pairs = getPointPairsFromFeatureWithDistanceConstraint(features1, features2, img1.width(), img1.height(), overlap_ratio);

    // For 360-degree panoramas, use ALL feature pairs in the overlap region
    // The seam line is just a reference - we want to use all features for parallax correction
    // The overlap region is defined by the distance constraint in getPointPairsFromFeatureWithDistanceConstraint
    // So all pairs in all_pairs are already in the overlap region
    seam_features = all_pairs;

    cout << "Found " << seam_features.size() << " features in overlap region" << endl;
    return seam_features;
}

// Find features only within the exact overlap region between adjacent images
vector<point_pair> ParallaxCorrection::findSeamFeaturesInOverlap(
    const CImg<unsigned char>& img1,
    const CImg<unsigned char>& img2,
    const map<vector<float>, VlSiftKeypoint>& features1,
    const map<vector<float>, VlSiftKeypoint>& features2,
    const SeamLine& seam_line) {

    vector<point_pair> seam_features;

    // Get all feature pairs between the two images with distance constraint
    double overlap_ratio = 0.297; // 19.04°/64.04° from stitching profile
    vector<point_pair> all_pairs = getPointPairsFromFeatureWithDistanceConstraint(features1, features2, img1.width(), img1.height(), overlap_ratio);
    cout << "Total feature pairs found: " << all_pairs.size() << endl;

    // Calculate overlap region for 360-degree panorama
    int img_width = img1.width();
    int img_height = img1.height();

    // For 8-camera setup with 45° spacing, overlap is approximately 19° out of 64° FOV
    // This means overlap is about 19/64 = 0.297 of the image width
    // overlap_ratio is already defined above
    int overlap_width = (int)(img_width * overlap_ratio);

    // For 360-degree panorama with 8 cameras at 45-degree intervals,
    // the overlap regions are at the edges where adjacent images meet.
    // Each image overlaps with the next image on the right edge.

    // For 360-degree panorama, we'll use the right edge of img1 and left edge of img2
    // as the overlap regions where adjacent images meet
    int overlap_half_width = overlap_width; // Use actual overlap width

    // img1 overlap region: right edge (where it meets the next image)
    int img1_overlap_start = img_width - overlap_width;
    int img1_overlap_end = img_width;

    // img2 overlap region: left edge (where it meets the previous image)
    int img2_overlap_start = 0;
    int img2_overlap_end = overlap_width;

    cout << "Image dimensions: " << img_width << "x" << img_height << endl;
    cout << "Overlap ratio: " << overlap_ratio << " (19.04°/64.04°)" << endl;
    cout << "Overlap width: " << overlap_width << " pixels" << endl;
    cout << "Overlap region - img1 (right edge): [" << img1_overlap_start << "-" << img1_overlap_end
         << "], img2 (left edge): [" << img2_overlap_start << "-" << img2_overlap_end << "]" << endl;

    // Count features in each region for debugging
    int features_in_img1_overlap = 0;
    int features_in_img2_overlap = 0;
    int features_in_both_overlaps = 0;

    // Filter features that are within the overlap region
    for (const auto& pair : all_pairs) {
        int x1 = (int)pair.a.x;
        int y1 = (int)pair.a.y;
        int x2 = (int)pair.b.x;
        int y2 = (int)pair.b.y;

        // Check if both features are within their respective overlap regions
        bool in_img1_overlap = (x1 >= img1_overlap_start && x1 < img1_overlap_end);
        bool in_img2_overlap = (x2 >= img2_overlap_start && x2 < img2_overlap_end);

        if (in_img1_overlap) features_in_img1_overlap++;
        if (in_img2_overlap) features_in_img2_overlap++;
        if (in_img1_overlap && in_img2_overlap) {
            seam_features.push_back(pair);
            features_in_both_overlaps++;
        }
    }

    cout << "Features in img1 overlap region: " << features_in_img1_overlap << endl;
    cout << "Features in img2 overlap region: " << features_in_img2_overlap << endl;
    cout << "Features in both overlap regions: " << features_in_both_overlaps << endl;
    cout << "Final seam features: " << seam_features.size() << endl;
    return seam_features;
}

// Calculate parallax displacement for features near seam
vector<WarpControlPoint> ParallaxCorrection::calculateParallaxDisplacement(
    const vector<point_pair>& seam_features,
    const SeamLine& seam_line,
    int image_width,
    int image_height) {

    vector<WarpControlPoint> control_points;

    if (seam_features.empty()) {
        return control_points;
    }

    cout << "Calculating parallax displacement for " << seam_features.size() << " features" << endl;

    // Create a control point for EVERY single feature match
    // This will provide maximum warping coverage
    for (const auto& pair : seam_features) {
        int x1 = (int)pair.a.x;
        int y1 = (int)pair.a.y;
        int x2 = (int)pair.b.x;
        int y2 = (int)pair.b.y;

        // Calculate displacement (difference between matched points)
        // For parallax correction, we need to calculate the displacement within the overlap region
        // The large displacement between features from different images is due to the circular arrangement
        // We need to focus on the actual parallax displacement within the overlap region

        // Calculate the displacement within the overlap region
        // For 360-degree panoramas, features at the right edge of one image should appear
        // at the left edge of the next image, but the parallax displacement should be small
        double dx = x2 - x1;
        double dy = y2 - y1;

        // If the displacement is very large in x-direction, it's due to circular arrangement
        // For parallax correction, we should use a much smaller displacement
        if (abs(dx) > image_width * 0.3) {
            // This is a circular wrap-around - scale down the x-displacement significantly
            // and keep the y-displacement as it represents the actual parallax
            dx = dx * 0.01; // Scale down x-displacement by 100x
            // Keep the y-displacement as it represents the actual parallax
        }

        // Only consider reasonable displacements
        if (abs(dx) < image_width * MAX_DISPLACEMENT_RATIO &&
            abs(dy) < image_height * MAX_DISPLACEMENT_RATIO) {

            // Create control point for this individual feature match
            WarpControlPoint cp;
            cp.x = x1;  // Use the first image's feature position
            cp.y = y1;
            cp.target_x = x1 + (int)dx;  // Apply the displacement
            cp.target_y = y1 + (int)dy;
            cp.weight = 1.0;  // Full weight for individual features

            control_points.push_back(cp);

            cout << "Control point at (" << cp.x << "," << cp.y << ") -> ("
                 << cp.target_x << "," << cp.target_y << ") weight=" << cp.weight
                 << " displacement=(" << (cp.target_x - cp.x) << "," << (cp.target_y - cp.y) << ")" << endl;
        }
    }

    cout << "Created " << control_points.size() << " control points for local warping" << endl;
    return control_points;
}

// Apply local warping to correct parallax
CImg<unsigned char> ParallaxCorrection::applyLocalWarping(
    const CImg<unsigned char>& image,
    const vector<WarpControlPoint>& control_points,
    int image_width,
    int image_height) {

    if (control_points.empty()) {
        return image;  // No warping needed
    }

    cout << "Applying local warping with " << control_points.size() << " control points" << endl;

    CImg<unsigned char> warped_image(image.width(), image.height(), image.depth(), image.spectrum(), 0);

    // Create displacement field
    CImg<double> dx_field(image.width(), image.height(), 1, 1, 0.0);
    CImg<double> dy_field(image.width(), image.height(), 1, 1, 0.0);

    // Calculate displacement field using control points
    for (int y = 0; y < image.height(); y++) {
        for (int x = 0; x < image.width(); x++) {
            double total_dx = 0.0, total_dy = 0.0;
            double total_weight = 0.0;

            for (const auto& cp : control_points) {
                double dist = sqrt((x - cp.x) * (x - cp.x) + (y - cp.y) * (y - cp.y));
                double weight = calculateWarpingWeight(x, y, cp, WARP_REGION_RADIUS);

                if (weight > 0.0) {
                    double dx = (cp.target_x - cp.x) * weight;
                    double dy = (cp.target_y - cp.y) * weight;

                    total_dx += dx;
                    total_dy += dy;
                    total_weight += weight;
                }
            }

            if (total_weight > 0.0) {
                dx_field(x, y) = total_dx / total_weight;
                dy_field(x, y) = total_dy / total_weight;
            }
        }
    }

    // Apply warping using displacement field
    for (int y = 0; y < image.height(); y++) {
        for (int x = 0; x < image.width(); x++) {
            double src_x = x - dx_field(x, y);
            double src_y = y - dy_field(x, y);

            // Check bounds
            if (src_x >= 0 && src_x < image.width() - 1 &&
                src_y >= 0 && src_y < image.height() - 1) {

                for (int c = 0; c < image.spectrum(); c++) {
                    warped_image(x, y, 0, c) = bilinearInterpolate(image, src_x, src_y, c);
                }
            }
        }
    }

    cout << "Local warping completed" << endl;
    return warped_image;
}

// Main function to correct parallax for all images
vector<CImg<unsigned char>> ParallaxCorrection::correctParallax(
    const vector<CImg<unsigned char>>& images,
    const vector<ImageProfile>& image_profiles,
    const vector<map<vector<float>, VlSiftKeypoint>>& features) {

    cout << "Starting parallax correction for " << images.size() << " images..." << endl;

    vector<CImg<unsigned char>> corrected_images = images;

    // Detect seam lines
    vector<SeamLine> seam_lines = detectSeamLines(images, image_profiles, features);

    if (seam_lines.empty()) {
        cout << "No seam lines detected, skipping parallax correction" << endl;
        return corrected_images;
    }

    // Process each seam line
    for (const auto& seam : seam_lines) {
        cout << "Processing seam between images " << seam.image1_index << " and " << seam.image2_index << endl;

        // Find features near this seam
        vector<point_pair> seam_features = findSeamFeatures(
            images[seam.image1_index],
            images[seam.image2_index],
            features[seam.image1_index],
            features[seam.image2_index],
            seam
        );

        if (seam_features.size() < 3) {  // Lower threshold for 8-camera setup
            cout << "Not enough features near seam (" << seam_features.size() << "), skipping" << endl;
            continue;
        }

        // Calculate parallax displacement for both images
        vector<WarpControlPoint> control_points1 = calculateParallaxDisplacement(
            seam_features, seam, images[seam.image1_index].width(), images[seam.image1_index].height()
        );

        // Create reverse feature pairs for second image
        vector<point_pair> reverse_seam_features;
        for (const auto& pair : seam_features) {
            reverse_seam_features.push_back(point_pair(pair.b, pair.a));
        }

        vector<WarpControlPoint> control_points2 = calculateParallaxDisplacement(
            reverse_seam_features, seam, images[seam.image2_index].width(), images[seam.image2_index].height()
        );

        // Apply local warping to both images
        if (!control_points1.empty()) {
            corrected_images[seam.image1_index] = applyLocalWarping(
                corrected_images[seam.image1_index], control_points1,
                images[seam.image1_index].width(), images[seam.image1_index].height()
            );
        }

        if (!control_points2.empty()) {
            corrected_images[seam.image2_index] = applyLocalWarping(
                corrected_images[seam.image2_index], control_points2,
                images[seam.image2_index].width(), images[seam.image2_index].height()
            );
        }
    }

    cout << "Parallax correction completed" << endl;
    return corrected_images;
}

// Helper function to calculate distance from point to seam line
double ParallaxCorrection::distanceToSeamLine(int x, int y, const SeamLine& seam_line) {
    double min_dist = 1e9;

    for (const auto& seam_point : seam_line.seam_points) {
        double dist = sqrt((x - seam_point.first) * (x - seam_point.first) +
                          (y - seam_point.second) * (y - seam_point.second));
        min_dist = min(min_dist, dist);
    }

    return min_dist;
}

// Helper function to find the closest point on seam line
pair<int, int> ParallaxCorrection::findClosestSeamPoint(int x, int y, const SeamLine& seam_line) {
    double min_dist = 1e9;
    pair<int, int> closest_point = seam_line.seam_points[0];

    for (const auto& seam_point : seam_line.seam_points) {
        double dist = sqrt((x - seam_point.first) * (x - seam_point.first) +
                          (y - seam_point.second) * (y - seam_point.second));
        if (dist < min_dist) {
            min_dist = dist;
            closest_point = seam_point;
        }
    }

    return closest_point;
}

// Helper function for bilinear interpolation in warping
unsigned char ParallaxCorrection::bilinearInterpolate(
    const CImg<unsigned char>& image,
    double x, double y, int channel) {

    int x1 = (int)floor(x);
    int y1 = (int)floor(y);
    int x2 = min(x1 + 1, image.width() - 1);
    int y2 = min(y1 + 1, image.height() - 1);

    double fx = x - x1;
    double fy = y - y1;

    double val = (1 - fx) * (1 - fy) * image(x1, y1, 0, channel) +
                 fx * (1 - fy) * image(x2, y1, 0, channel) +
                 (1 - fx) * fy * image(x1, y2, 0, channel) +
                 fx * fy * image(x2, y2, 0, channel);

    return (unsigned char)max(0.0, min(255.0, val));
}

// Helper function to calculate warping weights
double ParallaxCorrection::calculateWarpingWeight(
    int x, int y,
    const WarpControlPoint& control_point,
    int radius) {

    double dist = sqrt((x - control_point.x) * (x - control_point.x) +
                      (y - control_point.y) * (y - control_point.y));

    if (dist > radius) {
        return 0.0;
    }

    // Use Gaussian-like weight function
    double normalized_dist = dist / radius;
    double weight = exp(-2.0 * normalized_dist * normalized_dist);

    return weight * control_point.weight;
}

// Generate seam lines from stitching profile (radial angles)
vector<SeamLine> ParallaxCorrection::generateSeamLinesFromProfile(
    const vector<ImageProfile>& image_profiles,
    int image_width,
    int image_height) {

    vector<SeamLine> seam_lines;

    cout << "Generating seam lines from stitching profile..." << endl;

    // Create a map of angle to index for easier lookup
    map<double, int> angle_to_index;
    for (int i = 0; i < image_profiles.size(); i++) {
        angle_to_index[image_profiles[i].radialAngle] = i;
    }

    // Find adjacent images and create seam lines
    for (int i = 0; i < image_profiles.size(); i++) {
        double current_angle = image_profiles[i].radialAngle;
        double next_angle = current_angle + 45.0; // 45 degree spacing
        if (next_angle >= 360.0) next_angle -= 360.0;

        if (angle_to_index.find(next_angle) != angle_to_index.end()) {
            int next_index = angle_to_index[next_angle];

            SeamLine seam;
            seam.image1_index = i;
            seam.image2_index = next_index;
            seam.confidence = 1.0; // High confidence for profile-based seams

            // Create vertical seam line at the right edge of the first image
            // This represents the boundary where the two images overlap
            int seam_x = image_width - 50; // 50 pixels from right edge

            for (int y = 0; y < image_height; y += 10) {
                seam.seam_points.push_back(make_pair(seam_x, y));
            }

            seam_lines.push_back(seam);
            cout << "Created seam between images " << i << " (angle: " << current_angle
                 << "°) and " << next_index << " (angle: " << next_angle << "°) at x=" << seam_x << endl;
        }
    }

    cout << "Generated " << seam_lines.size() << " seam lines from profile" << endl;
    return seam_lines;
}

// Main function to correct parallax using pre-computed seam lines
vector<CImg<unsigned char>> ParallaxCorrection::correctParallaxWithProfile(
    const vector<CImg<unsigned char>>& src_imgs,
    const vector<ImageProfile>& image_profiles,
    const vector<map<vector<float>, VlSiftKeypoint>>& features) {

    cout << "Starting parallax correction with profile-based seam lines..." << endl;

    if (src_imgs.empty() || image_profiles.empty()) {
        cout << "No images or profiles provided for parallax correction" << endl;
        return src_imgs;
    }

    int img_width = src_imgs[0].width();
    int img_height = src_imgs[0].height();

    // Generate seam lines from profile
    vector<SeamLine> seam_lines = generateSeamLinesFromProfile(image_profiles, img_width, img_height);

    if (seam_lines.empty()) {
        cout << "No seam lines generated from profile" << endl;
        return src_imgs;
    }

    vector<CImg<unsigned char>> corrected_imgs = src_imgs;

    // Process each seam line
    for (const auto& seam : seam_lines) {
        cout << "Processing seam between images " << seam.image1_index << " and " << seam.image2_index << endl;

        // Find features near this seam
        vector<point_pair> seam_features = findSeamFeatures(
            src_imgs[seam.image1_index],
            src_imgs[seam.image2_index],
            features[seam.image1_index],
            features[seam.image2_index],
            seam
        );

        cout << "Found " << seam_features.size() << " features near seam" << endl;

        if (seam_features.size() < 3) {
            cout << "Not enough features near seam, skipping" << endl;
            continue;
        }

        // Calculate displacement for both images
        vector<WarpControlPoint> control_points1 = calculateParallaxDisplacement(
            seam_features, seam, img_width, img_height
        );

        vector<WarpControlPoint> control_points2 = calculateParallaxDisplacement(
            seam_features, seam, img_width, img_height
        );

        cout << "Created " << control_points1.size() << " control points for image " << seam.image1_index << endl;
        cout << "Created " << control_points2.size() << " control points for image " << seam.image2_index << endl;

        // Apply local warping if we have control points
        if (!control_points1.empty()) {
            corrected_imgs[seam.image1_index] = applyLocalWarping(
                corrected_imgs[seam.image1_index], control_points1, img_width, img_height
            );
        }

        if (!control_points2.empty()) {
            corrected_imgs[seam.image2_index] = applyLocalWarping(
                corrected_imgs[seam.image2_index], control_points2, img_width, img_height
            );
        }
    }

    cout << "Parallax correction with profile completed" << endl;
    return corrected_imgs;
}

// Create feature visualization image
CImg<unsigned char> ParallaxCorrection::createFeatureVisualization(
    const vector<CImg<unsigned char>>& src_imgs,
    const vector<ImageProfile>& image_profiles,
    const vector<SeamLine>& seam_lines,
    const vector<map<vector<float>, VlSiftKeypoint>>& features) {

    if (src_imgs.empty()) {
        return CImg<unsigned char>();
    }

    int img_width = src_imgs[0].width();
    int img_height = src_imgs[0].height();

    // Create ultra-wide resolution image: 30720x2160 with 8 images + separators
    int target_width = 30720; // Ultra-wide width
    int target_height = 2160; // 4K height
    int separator_width = 5;  // 5 pixel wide vertical separators
    int image_width = (target_width - (separator_width * 7)) / 8; // 7 separators between 8 images
    int image_height = target_height;

    CImg<unsigned char> visualization(target_width, target_height, 1, 3, 0);

    // Sort images by radial angle to ensure correct left-to-right order
    vector<pair<double, int>> angle_index_pairs;
    for (int i = 0; i < image_profiles.size(); i++) {
        angle_index_pairs.push_back(make_pair(image_profiles[i].radialAngle, i));
    }
    sort(angle_index_pairs.begin(), angle_index_pairs.end());

    // Copy all images into the composite in correct order
    for (int i = 0; i < angle_index_pairs.size(); i++) {
        int img_index = angle_index_pairs[i].second;
        int start_x = i * (image_width + separator_width);

        // Resize and copy image
        CImg<unsigned char> resized_img = src_imgs[img_index];
        resized_img.resize(image_width, image_height);
        for (int y = 0; y < image_height; y++) {
            for (int x = 0; x < image_width; x++) {
                for (int c = 0; c < 3; c++) {
                    visualization(start_x + x, y, 0, c) = resized_img(x, y, 0, c);
                }
            }
        }

        // Draw vertical separator line (except after last image)
        if (i < angle_index_pairs.size() - 1) {
            for (int y = 0; y < image_height; y++) {
                for (int sep = 0; sep < separator_width; sep++) {
                    int sep_x = start_x + image_width + sep;
                    visualization(sep_x, y, 0, 0) = 255; // White separator
                    visualization(sep_x, y, 0, 1) = 255;
                    visualization(sep_x, y, 0, 2) = 255;
                }
            }
        }

        // Draw seam lines using the exact seam positions passed from main.cpp
        // Each image has two seam lines: left and right
        for (const auto& seam : seam_lines) {
            if (seam.image1_index == img_index) {
                // This is the right seam of the current image
                int seam_x_in_image = seam.seam_points[0].first;
                int seam_x_in_viz = start_x + (seam_x_in_image * image_width) / src_imgs[0].width();

                // Draw right seam line (5 pixels wide, green)
                if (seam_x_in_viz >= start_x && seam_x_in_viz < start_x + image_width) {
                    for (int y = 0; y < image_height; y++) {
                        for (int seam_width = 0; seam_width < 5; seam_width++) {
                            int seam_x_pos = seam_x_in_viz - 2 + seam_width;
                            if (seam_x_pos >= start_x && seam_x_pos < start_x + image_width) {
                                visualization(seam_x_pos, y, 0, 0) = 0;   // No red
                                visualization(seam_x_pos, y, 0, 1) = 255; // Full green
                                visualization(seam_x_pos, y, 0, 2) = 0;   // No blue
                            }
                        }
                    }
                }
            }
            if (seam.image2_index == img_index) {
                // This is the left seam of the current image (from previous image's perspective)
                int seam_x_in_image = seam.seam_points[0].first;
                int seam_x_in_viz = start_x + (seam_x_in_image * image_width) / src_imgs[0].width();

                // Draw left seam line (5 pixels wide, green)
                if (seam_x_in_viz >= start_x && seam_x_in_viz < start_x + image_width) {
                    for (int y = 0; y < image_height; y++) {
                        for (int seam_width = 0; seam_width < 5; seam_width++) {
                            int seam_x_pos = seam_x_in_viz - 2 + seam_width;
                            if (seam_x_pos >= start_x && seam_x_pos < start_x + image_width) {
                                visualization(seam_x_pos, y, 0, 0) = 0;   // No red
                                visualization(seam_x_pos, y, 0, 1) = 255; // Full green
                                visualization(seam_x_pos, y, 0, 2) = 0;   // No blue
                            }
                        }
                    }
                }
            }
        }
    }

    // Create a map from image index to position in the sorted order
    map<int, int> img_index_to_position;
    for (int i = 0; i < angle_index_pairs.size(); i++) {
        img_index_to_position[angle_index_pairs[i].second] = i;
    }

    // Draw features near seams with colored bounding boxes
    for (const auto& seam : seam_lines) {
        cout << "Visualizing features for seam between images " << seam.image1_index << " and " << seam.image2_index << endl;

        // First, get ALL feature pairs between these images with distance constraint
        double overlap_ratio = 0.297; // 19.04°/64.04° from stitching profile
        vector<point_pair> all_features = getPointPairsFromFeatureWithDistanceConstraint(
            features[seam.image1_index],
            features[seam.image2_index],
            src_imgs[seam.image1_index].width(),
            src_imgs[seam.image1_index].height(),
            overlap_ratio
        );
        cout << "Total feature pairs between images " << seam.image1_index << " and " << seam.image2_index << ": " << all_features.size() << endl;

        // Also get features in overlap region for comparison
        vector<point_pair> seam_features = findSeamFeaturesInOverlap(
            src_imgs[seam.image1_index],
            src_imgs[seam.image2_index],
            features[seam.image1_index],
            features[seam.image2_index],
            seam
        );

        cout << "Features in overlap region: " << seam_features.size() << endl;

        // Debug: Show positions of first few matched features
        cout << "First few matched feature positions:" << endl;
        int count = 0;
        for (const auto& pair : all_features) {
            if (count >= 5) break; // Show only first 5
            int x1 = (int)pair.a.x;
            int y1 = (int)pair.a.y;
            int x2 = (int)pair.b.x;
            int y2 = (int)pair.b.y;
            cout << "  Feature " << count << ": img1(" << x1 << "," << y1 << ") -> img2(" << x2 << "," << y2 << ")" << endl;
            count++;
        }

        // Visualize ALL features first (with smaller, less prominent markers)
        for (const auto& pair : all_features) {
            int x1 = (int)pair.a.x;
            int y1 = (int)pair.a.y;
            int x2 = (int)pair.b.x;
            int y2 = (int)pair.b.y;

            // Find positions in the sorted order
            int pos1 = img_index_to_position[seam.image1_index];
            int pos2 = img_index_to_position[seam.image2_index];

            // Calculate coordinates in the new layout
            int global_x1 = pos1 * (image_width + separator_width) + (x1 * image_width) / img_width;
            int global_x2 = pos2 * (image_width + separator_width) + (x2 * image_width) / img_width;
            int global_y1 = (y1 * image_height) / img_height;
            int global_y2 = (y2 * image_height) / img_height;

            // Draw small dots for all features (blue)
            int dot_size = 8;
            for (int dy = -dot_size/2; dy <= dot_size/2; dy++) {
                for (int dx = -dot_size/2; dx <= dot_size/2; dx++) {
                    int px1 = global_x1 + dx;
                    int py1 = global_y1 + dy;
                    int px2 = global_x2 + dx;
                    int py2 = global_y2 + dy;

                    if (px1 >= 0 && px1 < target_width && py1 >= 0 && py1 < target_height) {
                        visualization(px1, py1, 0, 0) = 0;   // No red
                        visualization(px1, py1, 0, 1) = 0;   // No green
                        visualization(px1, py1, 0, 2) = 255; // Full blue
                    }
                    if (px2 >= 0 && px2 < target_width && py2 >= 0 && py2 < target_height) {
                        visualization(px2, py2, 0, 0) = 0;   // No red
                        visualization(px2, py2, 0, 1) = 0;   // No green
                        visualization(px2, py2, 0, 2) = 255; // Full blue
                    }
                }
            }

            // Also draw lines connecting matched features (cyan)
            if (all_features.size() <= 50) { // Only draw lines if not too many features
                // Simple line drawing between matched features
                int steps = max(abs(global_x2 - global_x1), abs(global_y2 - global_y1));
                for (int i = 0; i <= steps; i++) {
                    int px = global_x1 + (global_x2 - global_x1) * i / steps;
                    int py = global_y1 + (global_y2 - global_y1) * i / steps;
                    if (px >= 0 && px < target_width && py >= 0 && py < target_height) {
                        visualization(px, py, 0, 0) = 0;   // No red
                        visualization(px, py, 0, 1) = 255; // Full green
                        visualization(px, py, 0, 2) = 255; // Full blue (cyan)
                    }
                }
            }
        }

        // Now highlight features in overlap region with larger, colored boxes
        for (const auto& pair : seam_features) {
            int x1 = (int)pair.a.x;
            int y1 = (int)pair.a.y;
            int x2 = (int)pair.b.x;
            int y2 = (int)pair.b.y;

            // Find positions in the sorted order
            int pos1 = img_index_to_position[seam.image1_index];
            int pos2 = img_index_to_position[seam.image2_index];

            // Calculate coordinates in the new layout
            int global_x1 = pos1 * (image_width + separator_width) + (x1 * image_width) / img_width;
            int global_x2 = pos2 * (image_width + separator_width) + (x2 * image_width) / img_width;
            int global_y1 = (y1 * image_height) / img_height;
            int global_y2 = (y2 * image_height) / img_height;

            cout << "Overlap feature pair: (" << x1 << "," << y1 << ") -> (" << x2 << "," << y2 << ")" << endl;
            cout << "Global coords: (" << global_x1 << "," << global_y1 << ") -> (" << global_x2 << "," << global_y2 << ")" << endl;

            // Draw bounding box for feature in first image (red)
            int box_size = 30; // Larger boxes for overlap features
            for (int dy = -box_size/2; dy <= box_size/2; dy++) {
                for (int dx = -box_size/2; dx <= box_size/2; dx++) {
                    int px = global_x1 + dx;
                    int py = global_y1 + dy;
                    if (px >= 0 && px < target_width && py >= 0 && py < target_height) {
                        // Red border (thick and bright)
                        if (abs(dx) >= box_size/2 - 2 || abs(dy) >= box_size/2 - 2) {
                            visualization(px, py, 0, 0) = 255; // Full red
                            visualization(px, py, 0, 1) = 0;   // No green
                            visualization(px, py, 0, 2) = 0;   // No blue
                        }
                    }
                }
            }

            // Draw bounding box for feature in second image (yellow)
            for (int dy = -box_size/2; dy <= box_size/2; dy++) {
                for (int dx = -box_size/2; dx <= box_size/2; dx++) {
                    int px = global_x2 + dx;
                    int py = global_y2 + dy;
                    if (px >= 0 && px < target_width && py >= 0 && py < target_height) {
                        // Yellow border (thick and bright)
                        if (abs(dx) >= box_size/2 - 2 || abs(dy) >= box_size/2 - 2) {
                            visualization(px, py, 0, 0) = 255; // Full red
                            visualization(px, py, 0, 1) = 255; // Full green
                            visualization(px, py, 0, 2) = 0;   // No blue
                        }
                    }
                }
            }
        }
    }

    cout << "Feature visualization created with dimensions " << target_width << "x" << target_height << endl;
    return visualization;
}

// Draw control points on the final stitched image
void ParallaxCorrection::drawControlPointsOnImage(
    CImg<unsigned char>& image,
    const vector<WarpControlPoint>& control_points) {

    cout << "Drawing " << control_points.size() << " control points on final image" << endl;

    for (const auto& cp : control_points) {
        int x = cp.x;
        int y = cp.y;
        int target_x = cp.target_x;
        int target_y = cp.target_y;

        // Draw control point as a red circle
        int radius = 8;
        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                if (dx*dx + dy*dy <= radius*radius) {
                    int px = x + dx;
                    int py = y + dy;
                    if (px >= 0 && px < image.width() && py >= 0 && py < image.height()) {
                        image(px, py, 0, 0) = 255; // Red
                        image(px, py, 0, 1) = 0;   // No green
                        image(px, py, 0, 2) = 0;   // No blue
                    }
                }
            }
        }

        // Draw target point as a green circle
        radius = 6;
        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                if (dx*dx + dy*dy <= radius*radius) {
                    int px = target_x + dx;
                    int py = target_y + dy;
                    if (px >= 0 && px < image.width() && py >= 0 && py < image.height()) {
                        image(px, py, 0, 0) = 0;   // No red
                        image(px, py, 0, 1) = 255; // Green
                        image(px, py, 0, 2) = 0;   // No blue
                    }
                }
            }
        }

        // Draw arrow from control point to target point
        int steps = max(abs(target_x - x), abs(target_y - y));
        for (int i = 0; i <= steps; i++) {
            int px = x + (target_x - x) * i / steps;
            int py = y + (target_y - y) * i / steps;
            if (px >= 0 && px < image.width() && py >= 0 && py < image.height()) {
                image(px, py, 0, 0) = 255; // Red
                image(px, py, 0, 1) = 255; // Green (yellow line)
                image(px, py, 0, 2) = 0;   // No blue
            }
        }
    }
}

// Collect all control points from parallax correction
vector<WarpControlPoint> ParallaxCorrection::collectAllControlPoints(
    const vector<CImg<unsigned char>>& src_imgs,
    const vector<ImageProfile>& image_profiles,
    const vector<map<vector<float>, VlSiftKeypoint>>& features) {

    vector<WarpControlPoint> all_control_points;

    cout << "Collecting all control points for visualization..." << endl;

    if (src_imgs.empty() || image_profiles.empty()) {
        return all_control_points;
    }

    // Generate seam lines from profile
    vector<SeamLine> seam_lines = generateSeamLinesFromProfile(
        image_profiles, src_imgs[0].width(), src_imgs[0].height()
    );

    // Process each seam line to collect control points
    for (const auto& seam : seam_lines) {
        if (seam.image1_index >= 0 && seam.image1_index < src_imgs.size() &&
            seam.image2_index >= 0 && seam.image2_index < src_imgs.size()) {

            // Find features near this seam
            vector<point_pair> seam_features = findSeamFeatures(
                src_imgs[seam.image1_index], src_imgs[seam.image2_index],
                features[seam.image1_index], features[seam.image2_index], seam
            );

            if (!seam_features.empty()) {
                // Calculate control points for this seam
                vector<WarpControlPoint> seam_control_points = calculateParallaxDisplacement(
                    seam_features, seam, src_imgs[0].width(), src_imgs[0].height()
                );

                // Add to the global collection
                all_control_points.insert(all_control_points.end(),
                    seam_control_points.begin(), seam_control_points.end());
            }
        }
    }

    cout << "Collected " << all_control_points.size() << " total control points" << endl;
    return all_control_points;
}
