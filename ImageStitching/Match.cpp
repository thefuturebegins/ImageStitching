#include "stdafx.h"

#ifdef OPENCV_AVAILABLE
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif

#include "Match.h"
#include "Feature.h"
#include <set>
#include <cmath>

float getXAfterWarping(float x, float y, Parameters H) {
	return H.c1 * x + H.c2 * y + H.c3 * x * y + H.c4;
}

float getYAfterWarping(float x, float y, Parameters H) {
	return H.c5 * x + H.c6 * y + H.c7 * x * y + H.c8;
}

vector<point_pair> getPointPairsFromFeature(const map<vector<float>, VlSiftKeypoint> &feature_a, const map<vector<float>, VlSiftKeypoint> &feature_b) {
	VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);

	float *data = new float[128 * feature_a.size()];
	int k = 0;
	for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
		const vector<float> &descriptors = it->first;
		assert(descriptors.size() == 128);

		for (int i = 0; i < 128; i++) {
			data[i + 128 * k] = descriptors[i];
		}
		k++;
	}

	vl_kdforest_build(forest, feature_a.size(), data);

	vector<point_pair> res;

	VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
	VlKDForestNeighbor neighbours[2];

	for (auto it = feature_b.begin(); it != feature_b.end(); it++){
		float *temp_data = new float[128];

		for (int i = 0; i < 128; i++) {
			temp_data[i] = (it->first)[i];
		}

		int nvisited = vl_kdforestsearcher_query(searcher, neighbours, 2, temp_data);

		float ratio = neighbours[0].distance / neighbours[1].distance;
		if (ratio < 0.5) {
			vector<float> des(128);
			for (int j = 0; j < 128; j++) {
				des[j] = data[j + neighbours[0].index * 128];
			}

			VlSiftKeypoint left = feature_a.find(des)->second;
			VlSiftKeypoint right = it->second;

			// Calculate distance between matched features
			float dx = left.x - right.x;
			float dy = left.y - right.y;
			float distance = sqrt(dx * dx + dy * dy);

			// For 360-degree panorama, features should be within a reasonable distance
			// Use a generous distance threshold based on image dimensions
			// Assuming images are 3840x2160, use 3/4 of image width as max distance
			float max_distance = 3840.0f * 0.75f; // 2880 pixels

			if (distance <= max_distance) {
				res.push_back(point_pair(left, right));
			}
		}

		delete[] temp_data;
		temp_data = NULL;
	}

	vl_kdforestsearcher_delete(searcher);
	vl_kdforest_delete(forest);

	delete[] data;
	data = NULL;

	return res;
}

vector<point_pair> getPointPairsFromFeatureWithDistanceConstraint(const map<vector<float>, VlSiftKeypoint> &feature_a, const map<vector<float>, VlSiftKeypoint> &feature_b, int img_width, int img_height, double overlap_ratio) {
	VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);

	float *data = new float[128 * feature_a.size()];
	int k = 0;
	for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
		const vector<float> &descriptors = it->first;
		assert(descriptors.size() == 128);

		for (int i = 0; i < 128; i++) {
			data[i + 128 * k] = descriptors[i];
		}
		k++;
	}

	vl_kdforest_build(forest, feature_a.size(), data);

	vector<point_pair> res;

	VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
	VlKDForestNeighbor neighbours[2];

	// Calculate maximum allowed distance based on overlap ratio
	// For 360-degree panorama, features should be within a reasonable distance
	int overlap_width = (int)(img_width * overlap_ratio);
	// Use a much more generous distance constraint - 3x the overlap width
	// This allows for the fact that in 360-degree panoramas, features might be
	// further apart due to the circular nature of the arrangement
	float max_distance = overlap_width * 3.0f;

	cout << "Distance constraint: max_distance = " << max_distance << " pixels (overlap_width = " << overlap_width << ")" << endl;

	for (auto it = feature_b.begin(); it != feature_b.end(); it++){
		float *temp_data = new float[128];

		for (int i = 0; i < 128; i++) {
			temp_data[i] = (it->first)[i];
		}

		int nvisited = vl_kdforestsearcher_query(searcher, neighbours, 2, temp_data);

		float ratio = neighbours[0].distance / neighbours[1].distance;
		if (ratio < 0.5) {
			vector<float> des(128);
			for (int j = 0; j < 128; j++) {
				des[j] = data[j + neighbours[0].index * 128];
			}

			VlSiftKeypoint left = feature_a.find(des)->second;
			VlSiftKeypoint right = it->second;

			// Calculate distance between matched features
			float dx = left.x - right.x;
			float dy = left.y - right.y;
			float distance = sqrt(dx * dx + dy * dy);

			if (distance <= max_distance) {
				res.push_back(point_pair(left, right));
			}
		}

		delete[] temp_data;
		temp_data = NULL;
	}

	vl_kdforestsearcher_delete(searcher);
	vl_kdforest_delete(forest);

	delete[] data;
	data = NULL;

	cout << "Distance-constrained matching: " << res.size() << " pairs (from " << feature_b.size() << " features)" << endl;
	return res;
}

Parameters getHomographyFromPoingPairs(const vector<point_pair> &pair) {
	assert(pair.size() == 4);

	float u0 = pair[0].a.x, v0 = pair[0].a.y;
	float u1 = pair[1].a.x, v1 = pair[1].a.y;
	float u2 = pair[2].a.x, v2 = pair[2].a.y;
	float u3 = pair[3].a.x, v3 = pair[3].a.y;

	float x0 = pair[0].b.x, y0 = pair[0].b.y;
	float x1 = pair[1].b.x, y1 = pair[1].b.y;
	float x2 = pair[2].b.x, y2 = pair[2].b.y;
	float x3 = pair[3].b.x, y3 = pair[3].b.y;

	float c1, c2, c3, c4, c5, c6, c7, c8;

	c1 = -(u0*v0*v1*x2 - u0*v0*v2*x1 - u0*v0*v1*x3 + u0*v0*v3*x1 - u1*v0*v1*x2 + u1*v1*v2*x0 + u0*v0*v2*x3 - u0*v0*v3*x2 + u1*v0*v1*x3 - u1*v1*v3*x0 + u2*v0*v2*x1 - u2*v1*v2*x0
		- u1*v1*v2*x3 + u1*v1*v3*x2 - u2*v0*v2*x3 + u2*v2*v3*x0 - u3*v0*v3*x1 + u3*v1*v3*x0 + u2*v1*v2*x3 - u2*v2*v3*x1 + u3*v0*v3*x2 - u3*v2*v3*x0 - u3*v1*v3*x2 + u3*v2*v3*x1)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	c2 = (u0*u1*v0*x2 - u0*u2*v0*x1 - u0*u1*v0*x3 - u0*u1*v1*x2 + u0*u3*v0*x1 + u1*u2*v1*x0 + u0*u1*v1*x3 + u0*u2*v0*x3 + u0*u2*v2*x1 - u0*u3*v0*x2 - u1*u2*v2*x0 - u1*u3*v1*x0
		- u0*u2*v2*x3 - u0*u3*v3*x1 - u1*u2*v1*x3 + u1*u3*v1*x2 + u1*u3*v3*x0 + u2*u3*v2*x0 + u0*u3*v3*x2 + u1*u2*v2*x3 - u2*u3*v2*x1 - u2*u3*v3*x0 - u1*u3*v3*x2 + u2*u3*v3*x1)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	c3 = (u0*v1*x2 - u0*v2*x1 - u1*v0*x2 + u1*v2*x0 + u2*v0*x1 - u2*v1*x0 - u0*v1*x3 + u0*v3*x1 + u1*v0*x3 - u1*v3*x0 - u3*v0*x1 + u3*v1*x0
		+ u0*v2*x3 - u0*v3*x2 - u2*v0*x3 + u2*v3*x0 + u3*v0*x2 - u3*v2*x0 - u1*v2*x3 + u1*v3*x2 + u2*v1*x3 - u2*v3*x1 - u3*v1*x2 + u3*v2*x1)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	c4 = (u0*u1*v0*v2*x3 - u0*u1*v0*v3*x2 - u0*u2*v0*v1*x3 + u0*u2*v0*v3*x1 + u0*u3*v0*v1*x2 - u0*u3*v0*v2*x1 - u0*u1*v1*v2*x3 + u0*u1*v1*v3*x2 + u1*u2*v0*v1*x3 - u1*u2*v1*v3*x0 - u1*u3*v0*v1*x2 + u1*u3*v1*v2*x0
		+ u0*u2*v1*v2*x3 - u0*u2*v2*v3*x1 - u1*u2*v0*v2*x3 + u1*u2*v2*v3*x0 + u2*u3*v0*v2*x1 - u2*u3*v1*v2*x0 - u0*u3*v1*v3*x2 + u0*u3*v2*v3*x1 + u1*u3*v0*v3*x2 - u1*u3*v2*v3*x0 - u2*u3*v0*v3*x1 + u2*u3*v1*v3*x0)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	c5 = -(u0*v0*v1*y2 - u0*v0*v2*y1 - u0*v0*v1*y3 + u0*v0*v3*y1 - u1*v0*v1*y2 + u1*v1*v2*y0 + u0*v0*v2*y3 - u0*v0*v3*y2 + u1*v0*v1*y3 - u1*v1*v3*y0 + u2*v0*v2*y1 - u2*v1*v2*y0
		- u1*v1*v2*y3 + u1*v1*v3*y2 - u2*v0*v2*y3 + u2*v2*v3*y0 - u3*v0*v3*y1 + u3*v1*v3*y0 + u2*v1*v2*y3 - u2*v2*v3*y1 + u3*v0*v3*y2 - u3*v2*v3*y0 - u3*v1*v3*y2 + u3*v2*v3*y1)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	c6 = (u0*u1*v0*y2 - u0*u2*v0*y1 - u0*u1*v0*y3 - u0*u1*v1*y2 + u0*u3*v0*y1 + u1*u2*v1*y0 + u0*u1*v1*y3 + u0*u2*v0*y3 + u0*u2*v2*y1 - u0*u3*v0*y2 - u1*u2*v2*y0 - u1*u3*v1*y0
		- u0*u2*v2*y3 - u0*u3*v3*y1 - u1*u2*v1*y3 + u1*u3*v1*y2 + u1*u3*v3*y0 + u2*u3*v2*y0 + u0*u3*v3*y2 + u1*u2*v2*y3 - u2*u3*v2*y1 - u2*u3*v3*y0 - u1*u3*v3*y2 + u2*u3*v3*y1)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	c7 = (u0*v1*y2 - u0*v2*y1 - u1*v0*y2 + u1*v2*y0 + u2*v0*y1 - u2*v1*y0 - u0*v1*y3 + u0*v3*y1 + u1*v0*y3 - u1*v3*y0 - u3*v0*y1 + u3*v1*y0
		+ u0*v2*y3 - u0*v3*y2 - u2*v0*y3 + u2*v3*y0 + u3*v0*y2 - u3*v2*y0 - u1*v2*y3 + u1*v3*y2 + u2*v1*y3 - u2*v3*y1 - u3*v1*y2 + u3*v2*y1)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	c8 = (u0*u1*v0*v2*y3 - u0*u1*v0*v3*y2 - u0*u2*v0*v1*y3 + u0*u2*v0*v3*y1 + u0*u3*v0*v1*y2 - u0*u3*v0*v2*y1 - u0*u1*v1*v2*y3 + u0*u1*v1*v3*y2 + u1*u2*v0*v1*y3 - u1*u2*v1*v3*y0 - u1*u3*v0*v1*y2 + u1*u3*v1*v2*y0
		+ u0*u2*v1*v2*y3 - u0*u2*v2*v3*y1 - u1*u2*v0*v2*y3 + u1*u2*v2*v3*y0 + u2*u3*v0*v2*y1 - u2*u3*v1*v2*y0 - u0*u3*v1*v3*y2 + u0*u3*v2*v3*y1 + u1*u3*v0*v3*y2 - u1*u3*v2*v3*y0 - u2*u3*v0*v3*y1 + u2*u3*v1*v3*y0)
		/ (u0*u1*v0*v2 - u0*u2*v0*v1 - u0*u1*v0*v3 - u0*u1*v1*v2 + u0*u3*v0*v1 + u1*u2*v0*v1 + u0*u1*v1*v3 + u0*u2*v0*v3 + u0*u2*v1*v2 - u0*u3*v0*v2 - u1*u2*v0*v2 - u1*u3*v0*v1
		- u0*u2*v2*v3 - u0*u3*v1*v3 - u1*u2*v1*v3 + u1*u3*v0*v3 + u1*u3*v1*v2 + u2*u3*v0*v2 + u0*u3*v2*v3 + u1*u2*v2*v3 - u2*u3*v0*v3 - u2*u3*v1*v2 - u1*u3*v2*v3 + u2*u3*v1*v3);

	return Parameters(c1, c2, c3, c4, c5, c6, c7, c8);
}

int numberOfIterations(float p, float w, int num) {
	return ceil(log(1 - p) / log(1 - pow(w, num)));
}

int random(int min, int max) {
	assert(max > min);

	return rand() % (max - min + 1) + min;
}

vector<int> getIndexsOfInliner(const vector<point_pair> &pairs, Parameters H, set<int> seleted_indexs) {
	vector<int> inliner_indexs;

	for (int i = 0; i < pairs.size(); i++) {
		if (seleted_indexs.find(i) != seleted_indexs.end()) {
			continue;
		}

		float real_x = pairs[i].b.x;
		float real_y = pairs[i].b.y;

		float x = getXAfterWarping(pairs[i].a.x, pairs[i].a.y, H);
		float y = getYAfterWarping(pairs[i].a.x, pairs[i].a.y, H);

		float distance = sqrt((x - real_x) * (x - real_x) + (y - real_y) * (y - real_y));
		if (distance < RANSAC_THRESHOLD) {
			inliner_indexs.push_back(i);
		}
	}

	return inliner_indexs;
}

Parameters leastSquaresSolution(const vector<point_pair> pairs, vector<int> inliner_indexs) {
	int calc_size = inliner_indexs.size();

	CImg<double> A(4, calc_size, 1, 1, 0);
	CImg<double> b(1, calc_size, 1, 1, 0);

	for (int i = 0; i < calc_size; i++) {
		int cur_index = inliner_indexs[i];

		A(0, i) = pairs[cur_index].a.x;
		A(1, i) = pairs[cur_index].a.y;
		A(2, i) = pairs[cur_index].a.x * pairs[cur_index].a.y;
		A(3, i) = 1;

		b(0, i) = pairs[cur_index].b.x;
	}

	CImg<double> x1 = b.get_solve(A);

	for (int i = 0; i < calc_size; i++) {
		int cur_index = inliner_indexs[i];

		b(0, i) = pairs[cur_index].b.y;
	}

	CImg<double> x2 = b.get_solve(A);

	return Parameters(x1(0, 0), x1(0, 1), x1(0, 2), x1(0, 3), x2(0, 0), x2(0, 1), x2(0, 2), x2(0, 3));

}

Parameters RANSAC(const vector<point_pair> &pairs) {
	assert(pairs.size() >= NUM_OF_PAIR);

	srand(time(0));

	int iterations = numberOfIterations(CONFIDENCE, INLINER_RATIO, NUM_OF_PAIR);

	vector<int> max_inliner_indexs;

	while (iterations--) {
		vector<point_pair> random_pairs;
		set<int> seleted_indexs;

		for (int i = 0; i < NUM_OF_PAIR; i++) {
			int index = random(0, pairs.size() - 1);
			while (seleted_indexs.find(index) != seleted_indexs.end()) {
				index = random(0, pairs.size() - 1);
			}
			seleted_indexs.insert(index);

			random_pairs.push_back(pairs[index]);
		}

		Parameters H = getHomographyFromPoingPairs(random_pairs);

		vector<int> cur_inliner_indexs = getIndexsOfInliner(pairs, H, seleted_indexs);
		if (cur_inliner_indexs.size() > max_inliner_indexs.size()) {
			max_inliner_indexs = cur_inliner_indexs;
		}
	}

	Parameters t = leastSquaresSolution(pairs, max_inliner_indexs);

	return t;

}

// Unified matching functions for both SIFT and ORB
vector<unified_point_pair> getUnifiedPointPairsFromFeature(const map<vector<float>, UnifiedKeypoint> &feature_a, const map<vector<float>, UnifiedKeypoint> &feature_b, FeatureAlgorithm algorithm) {
    vector<unified_point_pair> res;
    
    if (algorithm == FeatureAlgorithm::SIFT) {
        // Use k-d tree for SIFT (same as original implementation)
        VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);

        float *data = new float[128 * feature_a.size()];
        int k = 0;
        for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
            const vector<float> &descriptors = it->first;
            assert(descriptors.size() == 128);

            for (int i = 0; i < 128; i++) {
                data[i + 128 * k] = descriptors[i];
            }
            k++;
        }

        vl_kdforest_build(forest, feature_a.size(), data);

        VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
        VlKDForestNeighbor neighbours[2];

        for (auto it = feature_b.begin(); it != feature_b.end(); it++){
            float *temp_data = new float[128];

            for (int i = 0; i < 128; i++) {
                temp_data[i] = (it->first)[i];
            }

            int nvisited = vl_kdforestsearcher_query(searcher, neighbours, 2, temp_data);

            float ratio = neighbours[0].distance / neighbours[1].distance;
            if (ratio < 0.5) {
                vector<float> des(128);
                for (int j = 0; j < 128; j++) {
                    des[j] = data[j + neighbours[0].index * 128];
                }

                UnifiedKeypoint left = feature_a.find(des)->second;
                UnifiedKeypoint right = it->second;

                // Calculate distance between matched features
                float dx = left.x - right.x;
                float dy = left.y - right.y;
                float distance = sqrt(dx * dx + dy * dy);

                // For 360-degree panorama, features should be within a reasonable distance
                float max_distance = 3840.0f * 0.75f; // 2880 pixels

                if (distance <= max_distance) {
                    res.push_back(unified_point_pair(left, right));
                }
            }

            delete[] temp_data;
            temp_data = NULL;
        }

        vl_kdforestsearcher_delete(searcher);
        vl_kdforest_delete(forest);
        delete[] data;
        data = NULL;
        
    } else if (algorithm == FeatureAlgorithm::SURF) {
#ifdef OPENCV_AVAILABLE
        // Use OpenCV's BFMatcher for SURF (L2 distance)
        cv::BFMatcher matcher(cv::NORM_L2);
        
        // Convert features to OpenCV format
        vector<cv::KeyPoint> keypoints_a, keypoints_b;
        cv::Mat descriptors_a, descriptors_b;
        
        // Convert feature_a to OpenCV format
        for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_a.push_back(kp);
            
            // Convert descriptor to OpenCV Mat
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_32F);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<float>(0, i) = desc[i];
            }
            descriptors_a.push_back(desc_mat);
        }
        
        // Convert feature_b to OpenCV format
        for (auto it = feature_b.begin(); it != feature_b.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_b.push_back(kp);
            
            // Convert descriptor to OpenCV Mat
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_32F);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<float>(0, i) = desc[i];
            }
            descriptors_b.push_back(desc_mat);
        }
        
        // Match descriptors
        vector<vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors_a, descriptors_b, knn_matches, 2);
        
        // Apply Lowe's ratio test
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i].size() == 2) {
                const cv::DMatch& match1 = knn_matches[i][0];
                const cv::DMatch& match2 = knn_matches[i][1];
                
                if (match1.distance < 0.7f * match2.distance) {
                    UnifiedKeypoint left = feature_a.find(vector<float>(descriptors_a.row(match1.queryIdx).begin<float>(), descriptors_a.row(match1.queryIdx).end<float>()))->second;
                    UnifiedKeypoint right = feature_b.find(vector<float>(descriptors_b.row(match1.trainIdx).begin<float>(), descriptors_b.row(match1.trainIdx).end<float>()))->second;
                    
                    // Calculate distance between matched features
                    float dx = left.x - right.x;
                    float dy = left.y - right.y;
                    float distance = sqrt(dx * dx + dy * dy);
                    
                    // For 360-degree panorama, features should be within a reasonable distance
                    float max_distance = 3840.0f * 0.75f; // 2880 pixels
                    
                    if (distance <= max_distance) {
                        res.push_back(unified_point_pair(left, right));
                    }
                }
            }
        }
        
        cout << "SURF matching found " << res.size() << " matches" << endl;
#else
        cout << "ERROR: SURF matching requires OpenCV. Please install OpenCV or use SIFT instead." << endl;
#endif
        
    } else if (algorithm == FeatureAlgorithm::ORB) {
        cout << "DEBUG: Using ORB algorithm for feature matching" << endl;
#ifdef OPENCV_AVAILABLE
        // Use OpenCV's BFMatcher for ORB (Hamming distance)
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        
        // Convert features to OpenCV format
        vector<cv::KeyPoint> keypoints_a, keypoints_b;
        cv::Mat descriptors_a, descriptors_b;
        
        // Convert feature_a to OpenCV format
        for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_a.push_back(kp);
            
            // Convert descriptor to OpenCV Mat
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_8U);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<uchar>(0, i) = static_cast<uchar>(desc[i]);
            }
            descriptors_a.push_back(desc_mat);
        }
        
        // Convert feature_b to OpenCV format
        for (auto it = feature_b.begin(); it != feature_b.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_b.push_back(kp);
            
            // Convert descriptor to OpenCV Mat
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_8U);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<uchar>(0, i) = static_cast<uchar>(desc[i]);
            }
            descriptors_b.push_back(desc_mat);
        }
        
        // Match descriptors
        vector<vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors_a, descriptors_b, knn_matches, 2);
        
        // Apply Lowe's ratio test
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i].size() == 2) {
                const cv::DMatch& match1 = knn_matches[i][0];
                const cv::DMatch& match2 = knn_matches[i][1];
                
                if (match1.distance < 0.7f * match2.distance) {
                    UnifiedKeypoint left = feature_a.find(vector<float>(descriptors_a.row(match1.queryIdx).begin<uchar>(), descriptors_a.row(match1.queryIdx).end<uchar>()))->second;
                    UnifiedKeypoint right = feature_b.find(vector<float>(descriptors_b.row(match1.trainIdx).begin<uchar>(), descriptors_b.row(match1.trainIdx).end<uchar>()))->second;
                    
                    // Calculate distance between matched features
                    float dx = left.x - right.x;
                    float dy = left.y - right.y;
                    float distance = sqrt(dx * dx + dy * dy);
                    
                    // For 360-degree panorama, features should be within a reasonable distance
                    float max_distance = 3840.0f * 0.75f; // 2880 pixels
                    
                    if (distance <= max_distance) {
                        res.push_back(unified_point_pair(left, right));
                    }
                }
            }
        }
        
        cout << "ORB matching found " << res.size() << " matches" << endl;
        cout << "DEBUG: ORB matching completed successfully" << endl;
#else
        cout << "ERROR: ORB matching requires OpenCV. Please install OpenCV or use SIFT instead." << endl;
        cout << "DEBUG: ORB matching failed - OpenCV not available" << endl;
#endif
    }
    
    return res;
}

vector<unified_point_pair> getUnifiedPointPairsFromFeatureWithDistanceConstraint(const map<vector<float>, UnifiedKeypoint> &feature_a, const map<vector<float>, UnifiedKeypoint> &feature_b, int img_width, int img_height, double overlap_ratio, FeatureAlgorithm algorithm) {
    vector<unified_point_pair> res;
    
    if (algorithm == FeatureAlgorithm::SIFT) {
        // Use k-d tree for SIFT with distance constraint
        VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);

        float *data = new float[128 * feature_a.size()];
        int k = 0;
        for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
            const vector<float> &descriptors = it->first;
            assert(descriptors.size() == 128);

            for (int i = 0; i < 128; i++) {
                data[i + 128 * k] = descriptors[i];
            }
            k++;
        }

        vl_kdforest_build(forest, feature_a.size(), data);

        VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
        VlKDForestNeighbor neighbours[2];

        // Calculate maximum allowed distance based on overlap ratio
        int overlap_width = (int)(img_width * overlap_ratio);
        float max_distance = overlap_width * 3.0f;

        cout << "Distance constraint: max_distance = " << max_distance << " pixels (overlap_width = " << overlap_width << ")" << endl;

        for (auto it = feature_b.begin(); it != feature_b.end(); it++){
            float *temp_data = new float[128];

            for (int i = 0; i < 128; i++) {
                temp_data[i] = (it->first)[i];
            }

            int nvisited = vl_kdforestsearcher_query(searcher, neighbours, 2, temp_data);

            float ratio = neighbours[0].distance / neighbours[1].distance;
            if (ratio < 0.5) {
                vector<float> des(128);
                for (int j = 0; j < 128; j++) {
                    des[j] = data[j + neighbours[0].index * 128];
                }

                UnifiedKeypoint left = feature_a.find(des)->second;
                UnifiedKeypoint right = it->second;

                // Calculate distance between matched features
                float dx = left.x - right.x;
                float dy = left.y - right.y;
                float distance = sqrt(dx * dx + dy * dy);

                if (distance <= max_distance) {
                    res.push_back(unified_point_pair(left, right));
                }
            }

            delete[] temp_data;
            temp_data = NULL;
        }

        vl_kdforestsearcher_delete(searcher);
        vl_kdforest_delete(forest);
        delete[] data;
        data = NULL;
        
    } else if (algorithm == FeatureAlgorithm::SURF) {
#ifdef OPENCV_AVAILABLE
        // Use OpenCV's BFMatcher for SURF with distance constraint
        cv::BFMatcher matcher(cv::NORM_L2);
        
        // Convert features to OpenCV format
        vector<cv::KeyPoint> keypoints_a, keypoints_b;
        cv::Mat descriptors_a, descriptors_b;
        
        // Convert feature_a to OpenCV format
        for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_a.push_back(kp);
            
            // Convert descriptor to OpenCV Mat
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_32F);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<float>(0, i) = desc[i];
            }
            descriptors_a.push_back(desc_mat);
        }
        
        // Convert feature_b to OpenCV format
        for (auto it = feature_b.begin(); it != feature_b.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_b.push_back(kp);
            
            // Convert descriptor to OpenCV Mat
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_32F);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<float>(0, i) = desc[i];
            }
            descriptors_b.push_back(desc_mat);
        }
        
        // Match descriptors
        vector<vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors_a, descriptors_b, knn_matches, 2);
        
        // Apply Lowe's ratio test with distance constraint
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i].size() == 2) {
                const cv::DMatch& match1 = knn_matches[i][0];
                const cv::DMatch& match2 = knn_matches[i][1];
                
                if (match1.distance < 0.7f * match2.distance) {
                    UnifiedKeypoint left = feature_a.find(vector<float>(descriptors_a.row(match1.queryIdx).begin<float>(), descriptors_a.row(match1.queryIdx).end<float>()))->second;
                    UnifiedKeypoint right = feature_b.find(vector<float>(descriptors_b.row(match1.trainIdx).begin<float>(), descriptors_b.row(match1.trainIdx).end<float>()))->second;
                    
                    // Calculate distance between matched features
                    float dx = left.x - right.x;
                    float dy = left.y - right.y;
                    float distance = sqrt(dx * dx + dy * dy);
                    
                    // Apply distance constraint based on image dimensions and overlap ratio
                    float max_distance = sqrt(img_width * img_width + img_height * img_height) * overlap_ratio;
                    
                    if (distance <= max_distance) {
                        res.push_back(unified_point_pair(left, right));
                    }
                }
            }
        }
        
        cout << "SURF matching with distance constraint found " << res.size() << " matches" << endl;
#else
        cout << "ERROR: SURF matching requires OpenCV. Please install OpenCV or use SIFT instead." << endl;
#endif
        
    } else if (algorithm == FeatureAlgorithm::ORB) {
        cout << "DEBUG: Using ORB algorithm for feature matching with distance constraint" << endl;
#ifdef OPENCV_AVAILABLE
        // Use OpenCV's BFMatcher for ORB with distance constraint
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        
        // Convert features to OpenCV format (same as above)
        vector<cv::KeyPoint> keypoints_a, keypoints_b;
        cv::Mat descriptors_a, descriptors_b;
        
        // Convert feature_a to OpenCV format
        for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_a.push_back(kp);
            
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_8U);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<uchar>(0, i) = static_cast<uchar>(desc[i]);
            }
            descriptors_a.push_back(desc_mat);
        }
        
        // Convert feature_b to OpenCV format
        for (auto it = feature_b.begin(); it != feature_b.end(); it++) {
            cv::KeyPoint kp(it->second.x, it->second.y, it->second.scale, it->second.angle);
            keypoints_b.push_back(kp);
            
            vector<float> desc = it->first;
            cv::Mat desc_mat(1, desc.size(), CV_8U);
            for (size_t i = 0; i < desc.size(); i++) {
                desc_mat.at<uchar>(0, i) = static_cast<uchar>(desc[i]);
            }
            descriptors_b.push_back(desc_mat);
        }
        
        // Match descriptors
        vector<vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors_a, descriptors_b, knn_matches, 2);
        
        // Calculate maximum allowed distance based on overlap ratio
        int overlap_width = (int)(img_width * overlap_ratio);
        float max_distance = overlap_width * 3.0f;
        
        cout << "Distance constraint: max_distance = " << max_distance << " pixels (overlap_width = " << overlap_width << ")" << endl;
        
        // Apply Lowe's ratio test and distance constraint
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i].size() == 2) {
                const cv::DMatch& match1 = knn_matches[i][0];
                const cv::DMatch& match2 = knn_matches[i][1];
                
                if (match1.distance < 0.7f * match2.distance) {
                    UnifiedKeypoint left = feature_a.find(vector<float>(descriptors_a.row(match1.queryIdx).begin<uchar>(), descriptors_a.row(match1.queryIdx).end<uchar>()))->second;
                    UnifiedKeypoint right = feature_b.find(vector<float>(descriptors_b.row(match1.trainIdx).begin<uchar>(), descriptors_b.row(match1.trainIdx).end<uchar>()))->second;
                    
                    // Calculate distance between matched features
                    float dx = left.x - right.x;
                    float dy = left.y - right.y;
                    float distance = sqrt(dx * dx + dy * dy);
                    
                    if (distance <= max_distance) {
                        res.push_back(unified_point_pair(left, right));
                    }
                }
            }
        }
        
        cout << "Distance-constrained ORB matching: " << res.size() << " pairs" << endl;
        cout << "DEBUG: ORB matching with distance constraint completed successfully" << endl;
#else
        cout << "ERROR: ORB matching requires OpenCV. Please install OpenCV or use SIFT instead." << endl;
        cout << "DEBUG: ORB matching with distance constraint failed - OpenCV not available" << endl;
#endif
    }
    
    return res;
}

Parameters getHomographyFromUnifiedPointPairs(const vector<unified_point_pair> &pairs) {
    // Convert unified point pairs to regular point pairs for homography calculation
    vector<point_pair> regular_pairs;
    for (const auto& pair : pairs) {
        // Create SIFT keypoints from unified keypoints
        VlSiftKeypoint kp_a, kp_b;
        kp_a.x = pair.a.x;
        kp_a.y = pair.a.y;
        kp_a.sigma = pair.a.scale;
        kp_a.ix = pair.a.ix;
        kp_a.iy = pair.a.iy;
        kp_a.o = pair.a.octave;
        kp_a.is = 0;  // SIFT doesn't have is field in our unified structure
        
        kp_b.x = pair.b.x;
        kp_b.y = pair.b.y;
        kp_b.sigma = pair.b.scale;
        kp_b.ix = pair.b.ix;
        kp_b.iy = pair.b.iy;
        kp_b.o = pair.b.octave;
        kp_b.is = 0;  // SIFT doesn't have is field in our unified structure
        
        regular_pairs.push_back(point_pair(kp_a, kp_b));
    }
    
    return getHomographyFromPoingPairs(regular_pairs);
}

Parameters RANSACUnified(const vector<unified_point_pair> &pairs) {
    // Convert unified point pairs to regular point pairs for RANSAC
    vector<point_pair> regular_pairs;
    for (const auto& pair : pairs) {
        // Create SIFT keypoints from unified keypoints
        VlSiftKeypoint kp_a, kp_b;
        kp_a.x = pair.a.x;
        kp_a.y = pair.a.y;
        kp_a.sigma = pair.a.scale;
        kp_a.ix = pair.a.ix;
        kp_a.iy = pair.a.iy;
        kp_a.o = pair.a.octave;
        kp_a.is = 0;  // SIFT doesn't have is field in our unified structure
        
        kp_b.x = pair.b.x;
        kp_b.y = pair.b.y;
        kp_b.sigma = pair.b.scale;
        kp_b.ix = pair.b.ix;
        kp_b.iy = pair.b.iy;
        kp_b.o = pair.b.octave;
        kp_b.is = 0;  // SIFT doesn't have is field in our unified structure
        
        regular_pairs.push_back(point_pair(kp_a, kp_b));
    }
    
    return RANSAC(regular_pairs);
}