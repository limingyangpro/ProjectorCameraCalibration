#!/bin/bash
# ./run5.sh focusDistance
declare -a arr=("our0" "our1" "our2" "our3" "our4")
cd ..
mkdir result
cd result
mkdir allresult
for file in "${arr[@]}"
do
	testname=$1$file
	../build/bin/Debug/procamcalib -i 1 -b ../patterns/B4/B4Pattern.txt -p ../patterns/B4/B4PatternProjected.txt -o "$testname"
done

