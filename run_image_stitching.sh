#!/bin/bash
# Wrapper script to run image stitching without VLFeat debug messages

clear
./image_stitching 2>&1 | grep -v "VLFeat DEBUG"
