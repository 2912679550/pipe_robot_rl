%% 本脚本用于验证105管道机器人入弯里程与推杆长度换算的数学关系
close all; clc ; 

%% 定义机械参数取值
l1 = 62.0;
l2 = 270.0;
l3 = 115.0;
R = 300.0;
r = 180.0;
rw = 50.0;
deg_alpha = 51.8748;

%% 开始求解
input_theta = 31.2440; % 单位为°
rad_theta = input_theta / 180.0 * pi;
input_value = l3 + (1 - cos(rad_theta)) * (r + R) - cos(rad_theta) * rw;
% 直接使用匿名函数
valueFunc = @(m) (l3 - rw) *(2 * cos(m) * cos(m) - 1) + 2 * l1 * cos(m) * sqrt( 1 - cos(m) * cos(m)) + l2 * sqrt( 1 - cos(m)*cos(m)) - input_value;

beta0 = 10.0 / 180 * pi;
% options = optimset('Display' , 'iter');
[rad_beta , fval] = fsolve(valueFunc , beta0 );
disp(rad_beta);
% rad_beta = acos(cos_beta) ;
deg_beta = rad_beta / pi * 180.0;
disp(deg_beta);
cos_beta = cos(rad_beta);

% cos_beta0 = 0.8;
% % options = optimset('Display' , 'iter');
% [cos_beta , fval] = fsolve(valueFunc , cos_beta0 );
% disp(cos_beta);
% rad_beta = acos(cos_beta) ;
% deg_beta = rad_beta / pi * 180.0;
% disp(deg_beta);

% 解算推杆长度
deg_answer = deg_alpha - deg_beta;
length = power(100.4241 , 2) + power(83 , 2) - 2 * 100.4241 * 83 * cos(deg_answer / 180 * pi);
length = sqrt(length);
disp(["target length :" , length]);

% 验算此时入弯距离x的值：
x = (l1 + l2 / (2 * cos_beta)) * (1 + cos(2 * rad_beta)) - (l3 - rw) * sin(2 * rad_beta) - sin(rad_theta) * (R + r + rw);
disp(x)



