#!/bin/bash
cd "$1"
for D in */; do
	paramFile="${D}${D::-1}.yaml"
	echo "Processing $paramFile ..."   # your processing here
	../build/bin/Debug/augmentation -c "${paramFile}" -i 1 -b ../patterns/asymCircles/AcircleA4Print.txt -g ../patterns/asymCircles/AcircleA4Projected.txt -a ../patterns/asymCircles/AcircleA4Projected.png -e front
done
