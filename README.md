# ImageStitching
A solution for images stitchinging based on SIFT/ORB, kd-tree, RANSAC and Multi-band Blending.  
Based on [CImg Library](http://cimg.eu/), [Vlfeat library](http://www.vlfeat.org/), and [OpenCV](https://opencv.org/).

## Features
- **Dual Feature Matching Algorithms**: Choose between SIFT and ORB for feature detection and matching
- **Command-line Algorithm Selection**: Use `--algorithm sift` or `--algorithm orb` flags
- **Backward Compatibility**: Defaults to SIFT for existing workflows

## Input
- A set of images, supporting disordered images.
- Two sample input sets, "dataset1" and "dataset2".

## Output
- A stitched image.
- ![result1](https://github.com/AmazingZhen/ImageStitching/blob/master/ImageStitching/res/pano1.jpg?raw=true)
- ![result2](https://github.com/AmazingZhen/ImageStitching/blob/master/ImageStitching/res/pano2.jpg?raw=true)

## Usage

### Basic Usage
```bash
# Use SIFT (default)
./bin/ImageStitching

# Use ORB for faster processing
./bin/ImageStitching --algorithm orb

# Show help
./bin/ImageStitching --help
```

### Algorithm Comparison
- **SIFT**: More accurate, slower, better for high-quality results
- **ORB**: Faster processing, good for real-time applications, binary descriptors

## Algorithmic process
- Image registration
  + Cylinder projection
  + Feature extraction (SIFT or ORB)
  + Feature matching (k-d tree for SIFT, BFMatcher for ORB)
  + RANSAC and least squares for homography
- Image blending
  + Image wrapping
  + Multi-band blending

## Areas for improvement
- Compensate explosion error.
