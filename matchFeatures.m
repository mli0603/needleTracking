function [matchedFeaures] =  matchFeatures(features, patternPoints, findRotation)
    % read principle point
    principlePoint = patternPoints(9,:);
    pattern = patternPoints - principlePoint;
    
    % compute known set with shift and scale
    [shift,scale] = solveShiftScale(patternPoints,features,pattern);
    knownSet = principlePoint + shift + scale*pattern;
    hold on
    plot(knownSet(:,1),knownSet(:,2),'b*');

    if findRotation
        % find initial guess
        [A,~] = MinVolEllipse(features',.01); % find minimum ellipse
        [V,~] = eig(A); % find eigen vectors, col 1 is the principal axis
        V = round(V,4);
        if V(2,1) ~= 0
            principalAxisAngle = atan(V(1,1)/V(2,1));
        end
        T1 = eye(3,3);
        T1(1:2,3) = knownSet(end,:)';
        F0 = rotz(-rad2deg(principalAxisAngle));
        T2 = eye(3,3);
        T2(1:2,3) = -knownSet(end,:)';
        rotated = transformF(T1*F0*T2,knownSet')';
    else
        rotated = knownSet;
    end
    
    plot(rotated(:,1),rotated(:,2),'w*');

    % icp
    [matchedFeaures,~] = icp(eye(3,3),rotated',features');
    matchedFeaures = matchedFeaures';

    plot(matchedFeaures(:,1),matchedFeaures(:,2),'g*');
    hold off
    legend({'ICP','Found Feature Point','Scaled and Shifted Pattern','Initial Guess'})
end