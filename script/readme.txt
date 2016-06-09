**How to use the script **
1. ./run5.sh ??cm
This command calibrates 5 times successively, put results in "result" folder, and each result is in subfolder "??cmresult0", "??cmresult1"...

2. ./getFrontErrorA0.sh targetFolder
This commands test all calibration result in targetFolder, and create "front.err" file
Target Folder's composition:
(target/root)
--AAA (1 calibration result folder)
  --AAA.yaml
  --front.err (file to be created, .err is added automatically)
  
3. ./getErrorA0.sh targetFolder errorFileName.err
This commands test all calibration result in targetFolder, and create "errorFileName"
errorFileName convention (for rotation):
x_15, x+15, x_30, x+30... (rotation about x-axis)
y_15, y+15, y_30,... (rotation about y-axis)