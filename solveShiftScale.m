function [shift,scale] = solveShiftScale(imagePoints,featurePoints,pattern)
    % centre to centre shift
    shift = featurePoints(end,:) - imagePoints(end,:); 
    
    % scale
    featureDiff = featurePoints(1:end-1,:)-featurePoints(end,:);
    maxDistFeature = -Inf;
    maxDistPattern = -Inf;
    for i = 1:size(featurePoints,1)-1
        diffVec(i) = norm(featureDiff(i,:));
    end
    maxDistFeature = mean(diffVec)+std(diffVec)*1.5;
    
    for i = 1:size(pattern,1)
        maxDistPattern = max(maxDistPattern,norm(pattern(i,:)));
    end
    scale = maxDistFeature/maxDistPattern; % shape scale
end