# Projector Camera Calibration
This an application for calibrating procam system practically. User calibrates the system with a small board (about B4 paper size), whatever the projector's focusing distance is.

## Compatibility
Current version only works on Linux

## Pre-requirements
1. CMAKE 2.8
2. OpenCV 3.1.0
3. PCL 1.7
4. OpenSceneGraph 3.4.0
5. xinerama, X11

## Compilation
1. Checkout the directory
2. mkdir build
3. cd build
4. cmake ..
5. make

## Usage
Print function help:
./procamcalib -?
-i : input stream name (filename or camera ID)
-o : output file name
-b : file including points coordinates on calibration board
-p : file including points coordinates to project
-n : number of views
-r : record the calibration input (both the video stream and n images used for calibration)

Example:
./procamcalib -i 1 -b ./patterns/B4/B4Pattern.txt -p ./patterns/B4/B4PatternProjected.txt -o calibrationResult -n 10 -r

More patterns are available in ./patterns/

Video: https://www.youtube.com/watch?v=Npd0rKHBH_w

## Contributing
1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History
9 June 2016: First commit

## Issues
Some OS may have projector-camera synchronization problem, it will be solved later.
