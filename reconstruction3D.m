function  [points3D, imgLRect, imgRRect] = reconstruction3D(stereoParams, imgL, imgR, debug)
    % this function reconstructs 3D positions from a given stereo camera
    % and pair of images
    % image is filtered with median filter and diffusion filter
    % param
    % stereoParams: parameters of stereo camera
    % imgL: image of left camera
    % imgR: image of Right camera
    % output
    % points3D: reconstructed 3D points [u,v,3], where [u,v,1] = x,
    % [u,v,2] = y, and [u,v,3] = z
    %% rectify image
    [imgLRect, imgRRect] = ...
        rectifyStereoImages(imgL, imgR, stereoParams);

    imgLRectGray  = rgb2gray(imgLRect);
    imgRRectGray = rgb2gray(imgRRect);

    if debug
        figure
        imshow(imgLRectGray)

        figure
        imshow(imgRRectGray)
    end

    %% filter noise
    % median
    imgLRectGray = medfilt2(imgLRectGray);
    imgRRectGray = medfilt2(imgRRectGray);
    % diffusion
    imgLRectGray = imdiffusefilt(imgLRectGray);
    imgRRectGray = imdiffusefilt(imgRRectGray);

    figure
    imshow(imgLRectGray)
    figure
    imshow(imgRRectGray)
    %% find disparity map
    disparityMap = disparity(imgLRectGray, imgRRectGray);
    figure;
    imshow(disparityMap, [0, 64]);
    title('Disparity Map');
    colormap jet
    colorbar

    %% reconstruct
    points3D = reconstructScene(disparityMap, stereoParams);
    % Convert to meters and create a pointCloud object
    points3D = points3D ./ 1000;
    
end

% %% calculate from triangulation, read image
% imgLTest3 = imread('test/imgLTest3.jpg');
% imgRTest3 = imread('test/imgRTest3.jpg');
% imgLTest3_undistort = undistortImage(imgLTest3,camParamL); 
% imgRTest3_undistort = undistortImage(imgRTest3,camParamR); 
% figure
% imshow(imgLTest3_undistort)
% figure
% imshow(imgRTest3_undistort)
% 
% %%
% % find points in left image
% cornerPointsL = [592 741; 637 736; 692 361; 737 357];
% % find points in right image
% cornerPointsR = [527 746; 569 743; 625 368; 668 365];
% 
% worldPoint = triangulate(cornerPointsL,cornerPointsR,stereoParams)

% %% Calculate from matrix
% syms ur vr ul vl p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p33 p34 m11 m12 m13 m14 m21 m22 m23 m24 m31 m32 m33 m34 real
% P = [p11 p12 p13 p14; p21 p22 p23 p24; p31 p32 p33 p34];
% M = [m11 m12 m13 m14; m21 m22 m23 m24; m31 m32 m33 m34];
% % Pr
% Pr = [camParamR.IntrinsicMatrix' [0;0;0]];
% % Ml
% Pl = [camParamL.IntrinsicMatrix' [0;0;0]];
% Ml = Pl*[[stereoParams.RotationOfCamera2 stereoParams.TranslationOfCamera2'];[0 0 0 1]];
% 
% % find world coordinate
% p1 = computeWorldPoint(Pr,Ml,ul1,vl1,ur1,vr1)
% p2 = computeWorldPoint(Pr,Ml,ul2,vl2,ur2,vr2)
% norm(p2-p1)