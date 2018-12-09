function F = solve2D2DRegistration(pts2D1, pts2D2)
    % param: 
    % pts3D1 [2,N] matrix
    % pts3D2 [2,N] matrix
    % output: relative transformation from pts3D1 to pts3D2    
    % F: homogenous transfromation

    %fist find the reference point 
    pts2D1_0 = findRefPoint(pts2D1);
    pts2D2_0 = findRefPoint(pts2D2);

    %compute the relative translation to the reference point
    pts2D1_hat = pts2D1 - pts2D1_0;
    pts2D2_hat = pts2D2 - pts2D2_0;

    %get H matrix
    H = zeros(2,2);
    for i=1:size(pts2D1,2)
        H = H + pts2D1_hat(:,i) * pts2D2_hat(:,i)';
    end

    % compute SVD of H 
    [U,S,V] = svd(H);
    
    % compute R
    R = V * U';
    % check the coplanar but not colinear case
    if(det(R) < 0)
        fprintf('The inputs are coplanar, needs to convert reflection to rotation.\n');
%         V(:,3) = -V(:,3);
%         R= V * U';
    end   
    
    %compute t 
    t = pts2D2_0 - R * pts2D1_0;
    
    % compute F
    F = [R,t;0 0 1];

end