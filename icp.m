function [C, D] = icp(F0, moving, fixed )
    % F0: initial guess
    % C: registered points
    %% initialization 
    %get initial value
    [C,D] = icpMatchingSimple(F0,moving,fixed);
    
    n = 1;    % number of iteration
    F = F0;
    terminate = false;
    mean_error(n) = mean(D);
    threshold_distance = 3*mean_error(n); %may change for each iteration 
    threshold_error = 0.1;
%     threshold_cov = 0.005;
    %% iterate  
    
    
    while(terminate == false)
        clear A B
        n = n + 1;
        %select the point pairs for 3D-3D registration 
        idx = 1;
        for i=1:size(D,2)
            if(D(1,i)<threshold_distance)
                A(:,idx) = moving(:,i);
                B(:,idx) = C(:,i);
                idx = idx+1;
            end
        end

        %find the best transformation 
        F = solve2D2DRegistration(A,B);
        %match closest point 
        [C,D] = icpMatchingSimple(F0,moving,fixed);
        max_error = max(D);
        mean_error(n) = mean(D);
        cov_error_norm = sqrt(sum(D.^2))/size(D,2);
%         disp('Iteration'); disp(n);
%         disp('max error'); disp(max_error);
%         disp('mean error'); disp(mean_error(n));
%         disp('average error covariance'); disp(cov_error_norm);
        
        %update threshold_point 
        threshold_distance = 3*mean_error(n);
        
        %terminate condition 
        if (round(mean_error(n-1)/mean_error(n),2) <= 1 && round(mean_error(n-1)/mean_error(n),2) >= 0.95) ...
                && (mean_error(n) < threshold_error) && (max_error < 3 * threshold_error) %...
                % && (cov_error_norm < threshold_cov)
            terminate = true;
        elseif (mean_error(n-1) - mean_error(n) < 1E-5)
            terminate = true;
        end
    end
end