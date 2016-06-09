#!/bin/bash
# use : ./getErrorA0.sh foldercontainsdata errorFileName.err
cd "$1"
rm rerun.txt
for D in */; do
	paramFile="${D}${D::-1}.yaml"
	echo "Processing $paramFile ..."   # your processing here
	cmd="../build/bin/Debug/augmentation -c ${paramFile} -i 1 -b ../patterns/asymCirclesA0/AcircleA0Print.txt -g ../patterns/asymCirclesA0/AcircleA0Projected.txt -a ../patterns/asymCirclesA0/AcircleA0Projected.png -e $2 "
	if $cmd ; then
	  echo "Command succeeded"
	else
	  echo "Command failed"
	  "$cmd" >> rerun.txt
	fi
done


