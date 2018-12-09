function [C,D] = icpMatchingSimple(Freg,moving,fixed)
    %param
    % Freg [3,3]: current guess of Freg
    % moving [2,N,]: moving points
    % fixed [2,N_]: fixed points
    %output
    % C [2,N]: closest points in fixed point set to moving
    % D [2,N]: distance between C and moving
    
    for i = 1:size(moving,2)
        % transform with current guess
        s = transformF(Freg,moving(:,i));
        
        % initialize distance
        bound = Inf;
        
        % find closest point on fixed
        for j = 1:size(fixed,2)
            c = fixed(:,j);
            distance = norm(s-c);
            if distance < bound
                bound = distance;
                c_closest = c;
            end           
        end
        
        C(:,i) = c_closest;
        D(:,i) = bound; 
    end
end