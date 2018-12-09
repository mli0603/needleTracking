function features = findFeatures(I)
%     % undistort
%     I_undistort = undistortImage(I,camParam); 
    
    % filter the image to smoothe out the noise, using Anisotropic
    % diffusion filter to preserve the edge
    I_filtered = imdiffusefilt(I);
    figure
    imshow(I_filtered)
    
    % convert to gray image
    I_gray = rgb2gray(I_filtered);
    
    % find the brightest spot (center point)
    [maxVal,idx] = max(I_gray(:));
    [row,col] = ind2sub(size(I_gray),idx);
    hold on
    plot(col,row,'g*');
    hold off
         
    % hsv filter
    [maskToKeep,~] = hsvFilter(I_filtered);
    I_gray(~maskToKeep) = 0;
    
    % binarize
    I_gray(I_gray>(maxVal-10)) = 0;
    I_gray(I_gray<80) = 0;
    
    % thinning
    I_thin = bwmorph(I_gray,'thin',Inf);
    
    % find corners
    features = corner(I_thin);
    hold on
    plot(features(:,1),features(:,2),'r*');
    hold off
    
    % add the bright spot to feature list
    features = [features;[col,row]];
end