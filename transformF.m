function [ p_t ] = transformF( F,p )
    %Using transformation matrix to transform 3D point to 3D point 
    %param:
    %F: [3,3] transformation matrix 
    %p: [2,N] point 
    %ouput 
    %pt: [2,N] point
    
    p_t = F(1:2,1:2)*p + F(1:2,3);

end

