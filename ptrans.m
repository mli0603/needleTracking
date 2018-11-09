%% laser pointer intrinsics
f_laser = 1; % mm 
imgSize_laser = [640,480]; % mm or px, doesnt matter, scale to match the camera
principlePoint_laser = imgSize_laser*0.5; % mm or px, doesnt matter, scale to match the camera
laserIntrinsics = cameraIntrinsics(f_laser,principlePoint_laser,imgSize_laser);
laserParam = cameraParameters('IntrinsicMatrix',laserIntrinsics.IntrinsicMatrix);

Ol = [0,0,0];

%% camera intrinsics
f_cam = 5; % mm 
imgSize_cam = [640,480]; % px
principalPoint_cam = imgSize_cam*0.5; % px
camIntrinsics = cameraIntrinsics(f_cam,principalPoint_cam,imgSize_cam);
cameraParam = cameraParameters('IntrinsicMatrix',camIntrinsics.IntrinsicMatrix);

%% define image points on laser pointer lens
vec1 = [-1,2];
vec2 = [0,-1];
vec3 = [4,3];
vec4 = [-3,-3];
vec5 = [-2,2];
vec6 = [1,4];
vec7 = [3,-1];
vec8 = [-4,0];
vec9 = [0,0];
offset_vec = [vec1;vec2;vec3;vec4;vec5;vec6;vec7;vec8;vec9]/10;
imagePoints1 = offset_vec + principlePoint_laser;

%% find projected points
% projected plane
syms x y z 
ceilingEqn = 0.1*x+0.1*y+z == 2000;
for i = 1:size(imagePoints1,1)
    [x_tmp,y_tmp,z_tmp] = solveIntersection(Ol,[offset_vec(i,:),f_laser],ceilingEqn,x,y,z);
    projectedPoints(i,:) = [x_tmp,y_tmp,z_tmp];
end
projectedPoints = double(projectedPoints);

%% find projected points on camera
% define transformation matrix
R = rotx(30)
t = [1,0,-10]

O2 = Ol+t;
n = (R*[0,0,f_cam]')';  % normal of camera plane, rotated from laser pointer lens
principalPointOnLens = n+O2;
camPlaneEqn = n*[x-principalPointOnLens(1);y-principalPointOnLens(2);z-principalPointOnLens(3)] == 0;

for i = 1:size(projectedPoints,1)
    offset_vec = projectedPoints(i,:)-O2;
    [x_tmp,y_tmp,z_tmp] = solveIntersection(O2,offset_vec,camPlaneEqn,x,y,z);
    offset = inv(R)*([x_tmp,y_tmp,z_tmp]-principalPointOnLens)';
    imagePoints2(i,:) = offset(1:2)'+principalPoint_cam;
end

%% using known fundamental matrix to find the px location in the second camera
% sanity check
Kl = inv(laserIntrinsics.IntrinsicMatrix')';
E = [0,-(O2(3)-Ol(3)),O2(2)-Ol(2);O2(3)-Ol(3),0,-(O2(1)-Ol(1));-(O2(2)-Ol(2)),O2(1)-Ol(1),0]*R;
Kr = inv(laserIntrinsics.IntrinsicMatrix');
F = Kl*E*Kr;
for i = 1:size(imagePoints1,1)
    ul = imagePoints1(i,1); ur = imagePoints2(i,1);
    vl = imagePoints1(i,2); vr = imagePoints2(i,2);
    (F(1,1)*ul+F(2,1)*vl+F(3,1))*ur+(F(1,2)*ul+F(2,2)*vl+F(3,2))*vr+(F(1,3)*ul+F(2,3)*vl+F(3,3))
end
%% solve relative pose
% return the relative pose and translation of camera wrt laser
E_ = estimateEssentialMatrix(imagePoints1,imagePoints2,laserParam,cameraParam);
[relativeOrientation,relativeLocation] = relativeCameraPose(E_,laserParam,cameraParam,imagePoints1,imagePoints2)

%% plot
pcshow(projectedPoints,'b','VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',100);
% pcshow([imagePoints1-principalPoint1,focalLength1*ones(size(imagePoints1,1),1)],'b','VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',100);
hold on
% pcshow([imagePoints2-principalPoint2,focalLength2*ones(size(imagePoints2,1),1)],'g','VerticalAxis','Y','VerticalAxisDir','down','MarkerSize',500);
plotCamera('Size',60,'Orientation',eye(3),'Location',...
    Ol,'Label','Laser');
plotCamera('Size',60,'Orientation',relativeOrientation,'Location',...
    relativeLocation,'Color',[0,1,0],'Label','Camera');
hold off
xlabel('X_w')
ylabel('Y_w')
zlabel('Z_w')
