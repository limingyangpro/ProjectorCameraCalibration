#!/bin/bash
# use : ./getErrorA4.sh foldercontainsdata errorFileName.err
cd "$1"
rm rerun.txt
for D in */; do
	paramFile="${D}${D::-1}.yaml"
	echo "Processing $paramFile ..."   # your processing here
	cmd="../build/bin/Debug/augmentation -c ${paramFile} -i 1 -b ../patterns/asymCircles/AcircleA4Print.txt -g ../patterns/asymCircles/AcircleA4Projected.txt -a ../patterns/asymCircles/AcircleA4Projected.png -e $2"
	if $cmd ; then
	  echo "Command succeeded"
	else
	  echo "Command failed"
	  "$cmd" >> rerun.txt
	fi
done

