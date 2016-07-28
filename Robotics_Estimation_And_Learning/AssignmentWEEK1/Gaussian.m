function [ mean, std_dev ] = Gaussian( data )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
mean = sum(data,1)/length(data);
std_dev = std(data);
variance = power(std_dev,2);

end

