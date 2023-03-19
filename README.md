# 🤖 Calibration and Augmented Reality

This project is about learning how to calibrate a camera and then use the calibration to generate virtual objects in a scene. The end result should be a program that can detect a target and then place a virtual object in the scene relative to the target that moves and orients itself correctly given motion of the camera or target.

## Author

Yao Zhong, zhong.yao@northeastern.edu

## Environment

MacOS M1 chip

IDE: VS Code

Build with Makefile

## How to run the code

### For task 1~3(calibration)

In the treminal

1. Enter command `make calibrate` to compile cpp files

2. Then `./calibrate` to run the excutable

3. Keypress Instructions:

   - `q`: exit the program
   - `s`: save calibrate images and when images is more than 5, the calibration will automatically re-compute.

### For task 4~6(virtual object) and extention 2(ArUco)

In the treminal

1. Enter command `make objProjection` to compile cpp files

2. Then `./objProjection` to run the excutable

3. Keypress Instructions:

   - `q`: exit the program
   - `s`: save images for the report(used when developing)
   - `g`: When developing, pressing "g" for generating a new ArUco label.
   - `m`: change between chessboard or ArUco
   - `o`: decide if we project the virtual object or just the axes

### For task 7 (robust feature) and extension 1

In the treminal

1. Enter command `make patternDetection` to compile cpp files

2. Then `./patternDetection` to run the excutable

3. Keypress Instructions:

   - `q`: exit the program
   - `s`: save images for the repor, and save descriptors of a frame for the latter match step(if under SURF mode)
   - `t`: switch between harris corner and SURF feature
   - `r`: swich between feature dectetion and object projection
