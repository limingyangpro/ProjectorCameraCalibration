#!/bin/bash
# use : ./getErrorA0.sh foldercontainsdata
cd "$1"
for D in */; do
	paramFile="${D}${D::-1}.yaml"
	echo "Processing $paramFile ..."   # your processing here
	../build/bin/Debug/augmentation -c "${paramFile}" -i 1 -b ../patterns/asymCirclesA0/AcircleA0Print.txt -g ../patterns/asymCirclesA0/AcircleA0Projected.txt -a ../patterns/asymCirclesA0/AcircleA0Projected.png -e front
done
