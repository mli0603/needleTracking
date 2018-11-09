function [xl,yl,zl] = solveIntersection(o,vec,planeEqn,x,y,z)
    %param
    % o [3,1]: camera location
    % vec [3,1]: vector of a point deviating from camera position, in mm
    % planeEqn: eqn of a plane
    % x,y,z: sym variables
    
    syms s

    line = o + s * vec;
    xl = line(1);
    yl = line(2);
    zl = line(3);
    Eqn = subs(planeEqn, {x y z},{xl yl zl});
    s = solve(Eqn, s);
    
    if s < 0
        error('Scale is less than zero, cannot intersect');
    end
    
    line = o + s * vec;
    xl = double(line(1));
    yl = double(line(2));
    zl = double(line(3));
end