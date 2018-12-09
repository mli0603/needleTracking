function I = nonMaximumSupp(I,ksize)
    % pad the image
    I_pad = padarray(I,[(ksize-1)/2 (ksize-1)/2],'symmetric','both');
    
    % iterate through the image
    for i = (ksize-1)/2 +1 : size(I_pad,1)-(ksize-1)/2
        for j = (ksize-1)/2 + 1 :size(I_pad,2)-(ksize-1)/2
            kernel = I_pad(i-(ksize-1)/2:i+(ksize-1)/2,j-(ksize-1)/2:j+(ksize-1)/2);
            if kernel((ksize+1)/2 ,(ksize+1)/2 ) < max(kernel(:))
                I(i-(ksize-1)/2 ,j-(ksize-1)/2 ) = 0;
            end
        end
    end
end