#!/bin/bash
cd ..
mkdir result
cd result
testname=test
../build/bin/Debug/procamcalib -i 1 -b ../patterns/B4/B4Pattern.txt -p ../patterns/B4/B4PatternProjected.txt -o "$testname"

paramFile="test/test.yaml"
echo "Processing $paramFile ..."   # your processing here
cmd="../build/bin/Debug/augmentation -c ${paramFile} -i 1 -b ../patterns/asymCirclesA0/AcircleA0Print.txt -g ../patterns/asymCirclesA0/AcircleA0Projected.txt -a ../patterns/asymCirclesA0/AcircleA0Projected.png -e front "
if $cmd ; then
  echo "Command succeeded"
else
  echo "Command failed"
  "$cmd" >> rerun.txt
fi

