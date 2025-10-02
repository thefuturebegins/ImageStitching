#include "stdafx.h"

#ifdef OPENCV_AVAILABLE
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif

#include "Feature.h"

map<vector<float>, VlSiftKeypoint>  getFeatureFromImage(const CImg<unsigned char> &input) {
	assert(input.spectrum() == 1);

	CImg<unsigned char> src(input);

	float resize_factor;
	if (input.width() < input.height()) {
		resize_factor = RESIZE_SIZE / input.width();
	}
	else {
		resize_factor = RESIZE_SIZE / input.height();
	}

	if (resize_factor >= 1) {
		resize_factor = 1;
	}
	else {
		src.resize(src.width() * resize_factor, src.height() * resize_factor, src.depth(), src.spectrum(), 3);
	}

	vl_sift_pix *imageData = new vl_sift_pix[src.width()*src.height()];

	for (int i = 0; i < src.width(); i++) {
		for (int j = 0; j < src.height(); j++) {
			imageData[j*src.width() + i] = src(i, j, 0);
		}
	}

	// Initialize a SIFT filter object.
	int noctaves = 4, nlevels = 2, o_min = 0;
	VlSiftFilt *siftFilt = NULL;
	siftFilt = vl_sift_new(src.width(), src.height(), noctaves, nlevels, o_min);

	map<vector<float>, VlSiftKeypoint> features;

	// Compute the first octave of the DOG scale space.
	if (vl_sift_process_first_octave(siftFilt, imageData) != VL_ERR_EOF) {
		while (true) {
			// Run the SIFT detector to get the keypoints.
			vl_sift_detect(siftFilt);

			VlSiftKeypoint *pKeyPoint = siftFilt->keys;
			// For each keypoint:
			for (int i = 0; i < siftFilt->nkeys; i++) {
				VlSiftKeypoint tempKeyPoint = *pKeyPoint;

				// Get the keypoint orientation(s).
				double angles[4];
				int angleCount = vl_sift_calc_keypoint_orientations(siftFilt, angles, &tempKeyPoint);

				// For each orientation:
				for (int j = 0; j < angleCount; j++) {
					double tempAngle = angles[j];
					vl_sift_pix descriptors[128];

					// Get the keypoint descriptor
					vl_sift_calc_keypoint_descriptor(siftFilt, descriptors, &tempKeyPoint, tempAngle);

					vector<float> des;
					int k = 0;
					while (k < 128) {
						des.push_back(descriptors[k]);
						k++;
					}

					tempKeyPoint.x /= resize_factor;
					tempKeyPoint.y /= resize_factor;
					tempKeyPoint.ix = tempKeyPoint.x;
					tempKeyPoint.iy = tempKeyPoint.y;

					features.insert(pair<vector<float>, VlSiftKeypoint>(des, tempKeyPoint));
				}

				pKeyPoint++;
			}
			if (vl_sift_process_next_octave(siftFilt) == VL_ERR_EOF) {
				break;
			}
		}
	}

	vl_sift_delete(siftFilt);

	delete[] imageData;
	imageData = NULL;

	return features;
}

#ifdef OPENCV_AVAILABLE
// Helper function to convert CImg to OpenCV Mat
cv::Mat cimgToMat(const CImg<unsigned char>& cimg) {
    cv::Mat mat(cimg.height(), cimg.width(), CV_8UC1);
    for (int y = 0; y < cimg.height(); y++) {
        for (int x = 0; x < cimg.width(); x++) {
            mat.at<uchar>(y, x) = cimg(x, y, 0);
        }
    }
    return mat;
}

// Helper function to convert OpenCV Mat to CImg
CImg<unsigned char> matToCimg(const cv::Mat& mat) {
    CImg<unsigned char> cimg(mat.cols, mat.rows, 1, 1);
    for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
            cimg(x, y, 0) = mat.at<uchar>(y, x);
        }
    }
    return cimg;
}
#endif

// Unified feature extraction function
map<vector<float>, UnifiedKeypoint> getFeatureFromImageUnified(const CImg<unsigned char> &input, FeatureAlgorithm algorithm) {
    assert(input.spectrum() == 1);
    
    if (algorithm == FeatureAlgorithm::SIFT) {
        return getSiftFeatures(input);
    } else if (algorithm == FeatureAlgorithm::SURF) {
        return getSurfFeatures(input);
    } else if (algorithm == FeatureAlgorithm::ORB) {
        cout << "DEBUG: Using ORB algorithm for feature extraction" << endl;
        return getOrbFeatures(input);
    } else {
        // Default to SIFT
        cout << "DEBUG: Using default SIFT algorithm for feature extraction" << endl;
        return getSiftFeatures(input);
    }
}

// SIFT feature extraction (converted to unified format)
map<vector<float>, UnifiedKeypoint> getSiftFeatures(const CImg<unsigned char> &input) {
    assert(input.spectrum() == 1);

    CImg<unsigned char> src(input);

    float resize_factor;
    if (input.width() < input.height()) {
        resize_factor = RESIZE_SIZE / input.width();
    }
    else {
        resize_factor = RESIZE_SIZE / input.height();
    }

    if (resize_factor >= 1) {
        resize_factor = 1;
    }
    else {
        src.resize(src.width() * resize_factor, src.height() * resize_factor, src.depth(), src.spectrum(), 3);
    }

    vl_sift_pix *imageData = new vl_sift_pix[src.width()*src.height()];

    for (int i = 0; i < src.width(); i++) {
        for (int j = 0; j < src.height(); j++) {
            imageData[j*src.width() + i] = src(i, j, 0);
        }
    }

    // Initialize a SIFT filter object.
    int noctaves = 4, nlevels = 2, o_min = 0;
    VlSiftFilt *siftFilt = NULL;
    siftFilt = vl_sift_new(src.width(), src.height(), noctaves, nlevels, o_min);

    map<vector<float>, UnifiedKeypoint> features;

    // Compute the first octave of the DOG scale space.
    if (vl_sift_process_first_octave(siftFilt, imageData) != VL_ERR_EOF) {
        while (true) {
            // Run the SIFT detector to get the keypoints.
            vl_sift_detect(siftFilt);

            VlSiftKeypoint *pKeyPoint = siftFilt->keys;
            // For each keypoint:
            for (int i = 0; i < siftFilt->nkeys; i++) {
                VlSiftKeypoint tempKeyPoint = *pKeyPoint;

                // Get the keypoint orientation(s).
                double angles[4];
                int angleCount = vl_sift_calc_keypoint_orientations(siftFilt, angles, &tempKeyPoint);

                // For each orientation:
                for (int j = 0; j < angleCount; j++) {
                    double tempAngle = angles[j];
                    vl_sift_pix descriptors[128];

                    // Get the keypoint descriptor
                    vl_sift_calc_keypoint_descriptor(siftFilt, descriptors, &tempKeyPoint, tempAngle);

                    vector<float> des;
                    int k = 0;
                    while (k < 128) {
                        des.push_back(descriptors[k]);
                        k++;
                    }

                    tempKeyPoint.x /= resize_factor;
                    tempKeyPoint.y /= resize_factor;
                    tempKeyPoint.ix = tempKeyPoint.x;
                    tempKeyPoint.iy = tempKeyPoint.y;

                    UnifiedKeypoint unified_kp(tempKeyPoint);
                    features.insert(pair<vector<float>, UnifiedKeypoint>(des, unified_kp));
                }

                pKeyPoint++;
            }
            if (vl_sift_process_next_octave(siftFilt) == VL_ERR_EOF) {
                break;
            }
        }
    }

    vl_sift_delete(siftFilt);
    delete[] imageData;
    imageData = NULL;

    return features;
}

// SURF feature extraction
map<vector<float>, UnifiedKeypoint> getSurfFeatures(const CImg<unsigned char> &input) {
    assert(input.spectrum() == 1);

#ifdef OPENCV_AVAILABLE
    CImg<unsigned char> src(input);

    float resize_factor;
    if (input.width() < input.height()) {
        resize_factor = RESIZE_SIZE / input.width();
    }
    else {
        resize_factor = RESIZE_SIZE / input.height();
    }

    if (resize_factor >= 1) {
        resize_factor = 1;
    }
    else {
        src.resize(src.width() * resize_factor, src.height() * resize_factor, src.depth(), src.spectrum(), 3);
    }

    // Convert CImg to OpenCV Mat
    cv::Mat cv_image = cimgToMat(src);

    // Create SURF detector
    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(
        400,  // hessianThreshold - threshold for hessian keypoint detector
        4,    // nOctaves - number of octaves
        2,    // nOctaveLayers - number of layers in each octave
        true, // extended - extended descriptor (128 elements vs 64)
        true  // upright - rotation invariant
    );

    // Detect keypoints and compute descriptors
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    surf->detectAndCompute(cv_image, cv::Mat(), keypoints, descriptors);

    map<vector<float>, UnifiedKeypoint> features;

    // Convert OpenCV keypoints to our unified format
    for (size_t i = 0; i < keypoints.size(); i++) {
        const cv::KeyPoint& kp = keypoints[i];
        
        // Convert descriptor to vector<float>
        vector<float> des;
        if (descriptors.type() == CV_32F) {
            // SURF descriptors are float
            for (int j = 0; j < descriptors.cols; j++) {
                des.push_back(descriptors.at<float>(i, j));
            }
        } else {
            // If not float, convert
            for (int j = 0; j < descriptors.cols; j++) {
                des.push_back(static_cast<float>(descriptors.at<uchar>(i, j)));
            }
        }

        // Create SURF keypoint
        SurfKeypoint surf_kp;
        surf_kp.x = kp.pt.x / resize_factor;  // Scale back to original coordinates
        surf_kp.y = kp.pt.y / resize_factor;
        surf_kp.scale = kp.size;
        surf_kp.angle = kp.angle;
        surf_kp.ix = static_cast<int>(surf_kp.x);
        surf_kp.iy = static_cast<int>(surf_kp.y);
        surf_kp.octave = kp.octave;
        surf_kp.class_id = kp.class_id;
        surf_kp.response = kp.response;

        UnifiedKeypoint unified_kp(surf_kp);
        features.insert(pair<vector<float>, UnifiedKeypoint>(des, unified_kp));
    }

    cout << "SURF detected " << features.size() << " features" << endl;
    return features;
#else
    // OpenCV not available, return empty features
    cout << "ERROR: SURF features require OpenCV. Please install OpenCV or use SIFT instead." << endl;
    map<vector<float>, UnifiedKeypoint> features;
    return features;
#endif
}

// ORB feature extraction
map<vector<float>, UnifiedKeypoint> getOrbFeatures(const CImg<unsigned char> &input) {
    assert(input.spectrum() == 1);

#ifdef OPENCV_AVAILABLE
    CImg<unsigned char> src(input);

    float resize_factor;
    if (input.width() < input.height()) {
        resize_factor = RESIZE_SIZE / input.width();
    }
    else {
        resize_factor = RESIZE_SIZE / input.height();
    }

    if (resize_factor >= 1) {
        resize_factor = 1;
    }
    else {
        src.resize(src.width() * resize_factor, src.height() * resize_factor, src.depth(), src.spectrum(), 3);
    }

    // Convert CImg to OpenCV Mat
    cv::Mat cv_image = cimgToMat(src);

    // Create ORB detector
    cv::Ptr<cv::ORB> orb = cv::ORB::create(
        1000,  // nfeatures - maximum number of features
        1.2f,  // scaleFactor - pyramid decimation ratio
        8,     // nlevels - number of pyramid levels
        31,    // edgeThreshold - size of the border where features are not detected
        0,     // firstLevel - level of pyramid to put source image
        2,     // WTA_K - number of points that produce each element of the descriptor
        cv::ORB::HARRIS_SCORE,  // scoreType - algorithm to rank features
        31,    // patchSize - size of the patch used by the oriented BRIEF descriptor
        20     // fastThreshold - fast threshold
    );

    // Detect keypoints and compute descriptors
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb->detectAndCompute(cv_image, cv::Mat(), keypoints, descriptors);

    map<vector<float>, UnifiedKeypoint> features;

    // Convert OpenCV keypoints to our unified format
    for (size_t i = 0; i < keypoints.size(); i++) {
        const cv::KeyPoint& kp = keypoints[i];
        
        // Convert descriptor to vector<float>
        vector<float> des;
        if (descriptors.type() == CV_8U) {
            // ORB descriptors are binary (CV_8U), convert to float
            for (int j = 0; j < descriptors.cols; j++) {
                des.push_back(static_cast<float>(descriptors.at<uchar>(i, j)));
            }
        } else {
            // If already float
            for (int j = 0; j < descriptors.cols; j++) {
                des.push_back(descriptors.at<float>(i, j));
            }
        }

        // Create ORB keypoint
        OrbKeypoint orb_kp;
        orb_kp.x = kp.pt.x / resize_factor;  // Scale back to original coordinates
        orb_kp.y = kp.pt.y / resize_factor;
        orb_kp.scale = kp.size;
        orb_kp.angle = kp.angle;
        orb_kp.ix = static_cast<int>(orb_kp.x);
        orb_kp.iy = static_cast<int>(orb_kp.y);
        orb_kp.octave = kp.octave;
        orb_kp.class_id = kp.class_id;
        orb_kp.response = kp.response;

        UnifiedKeypoint unified_kp(orb_kp);
        features.insert(pair<vector<float>, UnifiedKeypoint>(des, unified_kp));
    }

    cout << "ORB detected " << features.size() << " features" << endl;
    cout << "DEBUG: ORB feature extraction completed successfully" << endl;
    return features;
#else
    // OpenCV not available, return empty features
    cout << "ERROR: ORB features require OpenCV. Please install OpenCV or use SIFT instead." << endl;
    cout << "DEBUG: ORB feature extraction failed - OpenCV not available" << endl;
    map<vector<float>, UnifiedKeypoint> features;
    return features;
#endif
}