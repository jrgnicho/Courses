% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

im_size = size(I);
mask = false(im_size(1),im_size(2));
rows = im_size(1);
cols = im_size(2);
hue_array = rgb2hsv(I);
hue_array = hue_array(:,:,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = 0.1572;
sigma = 0.0247;
interval_width = sigma;
threshold = 0.1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
    function p = probabilityDensity(val,mu,sigma)
        
        sigma_p2 = power(sigma,2);
        denom = sqrt(2*pi*sigma_p2)  ;
        numer = exp(-1 * power((val - mu),2)/(2*sigma_p2));
        p = numer/denom;
        
    end

    function p = probability(val,mu,sigma,w)
        a = val - w/2.0; % interval start
        b = val + w/2.0; % interval end
        
        fa = probabilityDensity(a,mu,sigma);
        fb = probabilityDensity(a,mu,sigma);
        
        % numerical integration by trapezoidal rule
        p = (b - a) * (fa + fb)*0.5;        

    end

for r = 1:rows
    for c = 1:cols
        val = hue_array(r,c);
        mask(r,c) = probability(val,mu,sigma,interval_width) > threshold;
    end    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

CC = bwconncomp(mask);
S = regionprops(CC,'Centroid');

ind = 1;
temp = 0;
num_elements = 0;
loc = [0, 0];
segI = mask;

% finding centroid of largest blob
if ~isempty(S)
    
    for i=1:length(S)
        temp = length(CC.PixelIdxList{i});
        if temp > num_elements
            num_elements = temp;
            ind = i;
        end
    end    
    
    loc = S(ind).Centroid;
end
    

% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
