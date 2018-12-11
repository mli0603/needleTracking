# needleTracking Algorith
Pattern tracking algorithm with a stereo camera. The project is intended to be a feasibility test for the idea of attaching a laser projector to the end of a lumbar puncture needle, so that the delicate motion of the needle can be tracked accurately with camera remotely tracking. 

## Current approach
By utilizing a stereo camera, the 3D location of projected pattern can be found. By solving the perspective-n-points problem, the pose of the laser pointer can be found. More details of the implementation can be found at link (https://github.com/mli0603/needleTracking/blob/master/report/Needle_Tracking_Algorithm.pdf).

## Result
![alt text](https://github.com/mli0603/needleTracking/blob/master/report/result.png)

*Unit: mm*

The error of the result is 700mm after confirming no bugs have been found in the algorith. Sources of error have been analyzed:
  - Camera calibration: 0.8 px reprojection error
  - Dected feature points in the patter compared to manually labeled points: 2.83 px on average, 2.17 px stddev
  - Triangulation: -0.76mm on average, 6.01 mm stddev
  - Other errors including manual calibration of laser patter
  
## Limitation
1. Using ICP to find the point correspondence will fail to detect the correct feature points when the pattern is projected on a slanted wall.
2. Perspective-n-point is sensitive to 3D location error. This was shown in a simulation study by introducing up to 1mm error to a 50mm projected pattern, and the recovered location is shifted by 8.3 mm.

## Current investigation
Virtual rigid body proposed by Alexis Cheng et. al.
https://www.scopus.com/record/display.uri?eid=2-s2.0-84943569395&origin=inward&txGid=6366b0971ab1a1e600ad0696c18ebaea
