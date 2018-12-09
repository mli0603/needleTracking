%% laser pointer intrinsics
focalLength = 5; % mm 
imageSize = [100,800]; % px
principalPoint = imageSize*0.5; % px
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
camParams = cameraParameters('IntrinsicMatrix',intrinsics.IntrinsicMatrix);

O1 = [0,10,0];
%% define image points
vec1 = [-1,2];
vec2 = [0,-1];
vec3 = [4,3];
vec4 = [-3,-3];

vec = [vec1;vec2;vec3;vec4];
imagePoints = vec + principalPoint;

%% define world points
% projected plane
syms x y z 
planeEqn = 0.1*x+ 0.1*y + z == 50;
for i = 1:size(imagePoints,1)
    [x_tmp,y_tmp,z_tmp] = solveIntersection(O1,[vec(i,:),focalLength],planeEqn,x,y,z);
    projectedPoints(i,:) = [x_tmp,y_tmp,z_tmp];
end

% world to cam
R = rotx(45);
t = [0,0,0];

% calculate world points
for i = 1:size(projectedPoints,1)
    worldPoints(i,:) = ((R*projectedPoints(i,:)')+t')' + rand(1,3)*1;
end

worldPoints = double(worldPoints);

%% if we don't have the correct scale
scale = 3;
focalLengthScaled = 5*scale; % mm 
imageSizeScaled = [100,800]*scale; % px
principalPointScaled = imageSize*0.5*scale; % px
intrinsicsScaled = cameraIntrinsics(focalLengthScaled,principalPointScaled,imageSizeScaled);
camParamsScaled = cameraParameters('IntrinsicMatrix',intrinsicsScaled.IntrinsicMatrix);
imagePointsScaled = imagePoints*scale;
%% solve world orientation
% returns the orientation and worldLocation of the center of camera
[worldOrientation,worldLocation, statusCode] = estimateWorldCameraPose(...
     imagePointsScaled,worldPoints,camParamsScaled) 
 
%% plot
pcshow(worldPoints,'b','VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',500);
hold on
% pcshow([imagePoints,focalLength*ones(size(imagePoints,1),1)],'g','VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',500);
plotCamera('Size',1,'Orientation',worldOrientation,'Location',...
    worldLocation);
hold off
xlabel('X_w')
ylabel('Y_w')
zlabel('Z_w')