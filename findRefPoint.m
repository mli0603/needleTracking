function [G0] = findRefPoint(G)
    % find the reference point from a set of measured points
    % the mid point in this case is taken as the reference point
    % G [3,N]: input pts
    % G0 [3,1]: reference pts

    G0 = sum(G,2)./size(G,2);
    
end