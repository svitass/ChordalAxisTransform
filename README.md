# ChordalAxisTransform  
2D Delaunay Triangulation实现，CAT实现

input files: input files/*.jpg     
A figure with only one outline, not a mask!!!   
The algorithm determines whether the edge of the triangle is a boundary according to the color of the pixel in the binary image.   
Therefore, if the mask image is used, this will fail and the correct skeleton cannot be generated.  

Steps to run the program:  
(1) line 497 in CAT.cpp: Modify the path of the input image  
(2) line 520 in CAT.cpp: Modify the step size of variable j to control the sampling density.    
     If it is set to 1, it means that each point on the contour is a sampling point, and the operation will be very time-consuming.  
(3) line 540 in CAT.cpp: Display DT results, comment out if not needed  
(4) line 544 in CAT.cpp: Display CAT results, comment out if not needed  
