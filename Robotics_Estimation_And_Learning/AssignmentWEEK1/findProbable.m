function [ mask, centroid ] = findProbable( Im,threshold )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    function p = probabilityDensity(val,mu,sigma)
        
        sigma_p2 = power(sigma,2);
        denom = sqrt(2*pi*sigma_p2)  ;
        numer = exp(-1 * power((val - mu),2)/(2*sigma_p2));
        p = numer/denom;
        
    end

    function p = probability(val,mu,sigma)
        a = val - sigma/2.0; % interval start
        b = val + sigma/2.0; % interval end
        
        fa = probabilityDensity(a,mu,sigma);
        fb = probabilityDensity(a,mu,sigma);
        
        % numerical integration by trapezoidal rule
        p = (b - a) * (fa + fb)*0.5;        

    end

im_size = size(Im);
mask = false(im_size(1),im_size(2));
rows = im_size(1);
cols = im_size(2);
hue_array = rgb2hsv(Im);
hue_array = hue_array(:,:,1);
%[m, std_dev] = Gaussian(reshape(hue_array,rows*cols,1));
m = 0.1572;
std_dev = 0.0247;
for r = 1:rows
    for c = 1:cols
        val = hue_array(r,c);
        mask(r,c) = probability(val,m,std_dev) < threshold;
    end    
end

CC = bwconncomp(mask);
S = regionprops(CC,'Centroid');

ind = 1;
temp = 0;
num_elements = 0;
for i=1:length(S)
    temp = length(CC.PixelIdxList{i});
    if temp > num_elements
        num_elements = temp;
        ind = i;
    end
end
    

centroid = S(ind).Centroid;
end

