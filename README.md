# Hough-Transform-From-Scratch
Basic Car License Plate Detection Using Hough Transform

## Dataset
  License Plate Dataset: <a href="https://drive.google.com/drive/folders/1CdWrhUm5lHWG19O-FIhVYJz6UXtZ7ZJu?usp=sharing">Link<a/>

## Requirements
    Modules:
    -numpy==1.20.1
    -opencv-python==4.5.1.48
    -math
    -sys
    -os
    Python Version 3.8.0


## Running
	-Should run program as "python houghTransform.py"
	-Program asks you to select one of functions ,generalSuccessTest or individualSuccessTest
	-After choosing one option, should type interval(e.g 0-150) for generalSuccessTest and number for individualSuccessTest(e.g 0).
	-Then program starts running and it took aprrox. 10-20 seconds to return result for individualSuccessTest and increases according to 
    interval you type on generalSuccessTest 
	
	
## Extra
	For individualSuccessTest;
		-Uncomment line 112 and 113 to save on current directory edge detected and final rectangle drawed image
		-To see filtered lines, uncomment line 223
		-To see intersection points, uncomment line 257
		-To see edge detected image on separate window, uncomment 421
	
	For generalSuccesTest;
		-To save every image with success rate to directory, uncomment line 129 and "directory_1" with desired directory 
		which is created earlier.
	

