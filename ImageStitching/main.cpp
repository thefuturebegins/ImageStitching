// ImageStitching.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"

#include <iostream>
#include <cassert>
#include <cmath>
#include "CImg.h"
#include "Projection.h"
#include "Feature.h"
#include "Match.h"
#include "Warping.h"
#include "FileReading.h"
#include "Stitching.h"

#define FILE_FOLDER "ImageStitching/dataset3/"

using namespace cimg_library;
using namespace std;

int main(int argc, char **argv) {

	string file_folder(FILE_FOLDER);
	vector<string> image_files;

	cout << "Getting files from: " << file_folder << endl;
	getAllFiles(file_folder, image_files);
	cout << "Found " << image_files.size() << " files" << endl;

	// Filter for image files only
	vector<string> image_files_filtered;
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

	cout << "Filtered to " << image_files_filtered.size() << " image files" << endl;

	vector<CImg<unsigned char>> src_imgs(image_files_filtered.size());
	vector<map<vector<float>, VlSiftKeypoint>> features(image_files_filtered.size());

	cout << "Loading images..." << endl;
	for (int i = 0; i < image_files_filtered.size(); i++) {
		cout << "Loading image " << i << ": " << image_files_filtered[i] << endl;
		src_imgs[i] = CImg<unsigned char>(image_files_filtered[i].c_str());
		cout << "Image " << i << " loaded successfully" << endl;
	}

	cout << "Starting stitching process..." << endl;
	CImg<unsigned char> res = stitching(src_imgs);
	cout << "Stitching completed successfully!" << endl;

	// res.display(); // Display disabled for headless operation
	cout << "Saving result..." << endl;
	res.save("ImageStitching/res/pano3.jpg");
	cout << "Result saved to ImageStitching/res/pano3.jpg" << endl;

	return 0;
}



