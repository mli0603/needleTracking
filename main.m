%% define laser parameters
load calibration/featurePoints.mat % for finding the direction

% L/d, L is the distance from a feature point to the centre, d is the distance, 
% obtained from UR5 calibration
pattern2DistanceRatio = [0.0274	0.0218	0.0292 0.0276 6.08E-03 0.0194 0.0204 9.77E-03]; 

focalLength = 5; % estimated in mm
pattern2Centre = pattern2DistanceRatio * focalLength; % scale with focal length
center = featurePoints(9,:);

% finding the direction and times length
for i = 1:size(featurePoints,1)-1
    pattern(i,:) = pattern2Centre(i)*(featurePoints(i,:)-center)/norm(featurePoints(i,:)-center);
end

% add centre to pattern
pattern(9,:) = [0,0];

imageSize = [16,12]; % px
principalPoint = imageSize*0.5; % px
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
laserParam = cameraParameters('IntrinsicMatrix',intrinsics.IntrinsicMatrix);

patternPoints = principalPoint + pattern;
%% load stereo parameters
load('calibration/calibrationSession.mat');
stereoParams = calibrationSession.CameraParameters;
camParamL = stereoParams.CameraParameters1;
camParamR = stereoParams.CameraParameters2;

%% loop through dataset
laserOrientation = [];
laserLocation = [];
failureCase = [];

dataSet = 12:15;
for idx = dataSet
    close all
    %% load images
    imgDataL = imread(['data/imgLData',int2str(idx),'.jpg']);
    imgDataR = imread(['data/imgRData',int2str(idx),'.jpg']);

    %% 3D reconstruction
    [points3D, imgLRect, imgRRect] = reconstruction3D(stereoParams, imgDataL, imgDataR,0);
    ptCloud = pointCloud(points3D, 'Color', imgLRect);

    % Create a streaming point cloud viewer
    player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
        'VerticalAxisDir', 'down');

    % Visualize the point cloud
    view(player3D, ptCloud);

    %% feature detection and matching
    featuresL = findFeatures(imgLRect);
    matchedFeauresL =  matchFeatures(featuresL, patternPoints, 0);

    featuresR = findFeatures(imgRRect);
    matchedFeauresR =  matchFeatures(featuresR, patternPoints, 0);

    %% find 3D position
    inlierPatternPoints_tmp = [];
    inlierPatternPoints = [];
    inlierFeaturePoints = [];
    matched3D = [];
    % first, filter based on rectification (the y value of image location)
    for i = 1:size(matchedFeauresL,1)
        % choose points that have y offset less than 2 pixels
        if abs(matchedFeauresL(i,2) - matchedFeauresR(i,2)) < 3
            xyz = double(reshape(points3D(matchedFeauresL(i,2),matchedFeauresL(i,1),:),[1,3]));
            matched3D = [matched3D;xyz];
            inlierPatternPoints_tmp = [inlierPatternPoints_tmp;patternPoints(i,:)];
        end
    end
    
    % sanity check, do we have more than 4 points
    if size(matched3D,1) < 4
        disp("Fewer than 4 points, cannot determine laser location");
        failureCase = [failureCase;idx];
        continue
    end

    % secondly, filter based on depth (the z value of the 3d points)
    mean_depth = mean(matched3D(:,3));
    for i = 1:size(matched3D, 1)
        % filter by 5mm depth error
        if abs(matched3D(i,3) - mean_depth) < 0.005
            inlierFeaturePoints = [inlierFeaturePoints;matched3D(i,:,:)];
            inlierPatternPoints = [inlierPatternPoints;inlierPatternPoints_tmp(i,:)];
        end
    end

    % sanity check, do we have more than 4 points
    if size(inlierFeaturePoints,1) < 4
        disp("Fewer than 4 points, cannot determine laser location");
        failureCase = [failureCase;idx];
        continue
    end

    %% find pose and location
    [laserOrientation_tmp,laserLocation_tmp, ~] = estimateWorldCameraPose(...
             inlierPatternPoints,inlierFeaturePoints * 1000,laserParam); % convert to meter
    laserOrientation = [laserOrientation;laserOrientation_tmp];
    laserLocation = [laserLocation;laserLocation_tmp];
end

%% Calculate the difference between laser location
% load UR5 data
load('data/UR5Data.mat')

% find success case
successCase = dataSet;
successCase=setdiff(successCase,failureCase);

% find corresponding ur5 locations
for i= 1:size(successCase,2)
    ur5LocationSuccess(i,:) = ur5Location(successCase(i)+1,:);
    ur5RotationSuccess(i,:) = ur5Rotation(successCase(i)+1,:);
end

% Laser
figure
hold on
for i = 1:size(laserLocation,1)
    plot3(laserLocation(i,1),laserLocation(i,2),laserLocation(i,3),'*');
end
xlabel('x')
ylabel('y')
zlabel('z')
grid on
axis equal

% UR5
figure
hold on
for i = 1:size(laserLocation,1)
    plot3(ur5LocationSuccess(i,1),ur5LocationSuccess(i,2),ur5LocationSuccess(i,3),'*');
end
xlabel('x')
ylabel('y')
zlabel('z')
grid on
axis equal

for i = 1:size(laserLocation,1)-1
    fprintf('%f %f\n',norm(laserLocation(i,:)-laserLocation(i+1,:)),norm(ur5LocationSuccess(i,:)-ur5LocationSuccess(i+1,:)))
end