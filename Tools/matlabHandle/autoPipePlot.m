close all;clc; clear;

deg_theta = 2:0.2:37;
deg_theta = deg_theta';
answer_length = zeros(size(deg_theta));

for i  = 1 : size(deg_theta)
    answer_length(i) = theta2Length(deg_theta(i));
end

plot(deg_theta , answer_length);