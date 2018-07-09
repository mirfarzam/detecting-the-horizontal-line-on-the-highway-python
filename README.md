# detecting-the-horizontal-line-on-the-highway-python
this is a python code which finds horizontal line of the highway while driving from point of view of car. 

The goal is finding horizon line of Highway real-time. At first I use “HoughLine” algorithm  to find the lines of the view. After that, I use RANSAC algorithm to find intersection points.
for this RANSAC algorithm I just choose the lines that their theta’s be between 0.3 to 1.4 radian or 1.6 to 2 to omit those lines which are detected  but they are for highway lights, other cars and bridges. some of highway ground lines stripped so i sum up each frame with second frame before current frame to join those lines too. It has few error and I think they are not very important.
