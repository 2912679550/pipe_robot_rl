%% 本脚本用于绘制收集到的rosbag数据，绘制夹紧过程的电流与夹紧长度关系图
clc ; close all; clear;
front_ori_data = load("data\front.csv");
back_ori_data  = load("data\back.csv");

length_gate = -1; % 丝杠行程小于0.5mm的部分将被剔除

%% 数据预处理：删除第四列（丝杠长度）小于 length_gate 的行
% 处理前侧数据
front_length = front_ori_data(:, 4); % 提取第四列（丝杠长度）
valid_front_idx = front_length >= length_gate; % 找到满足条件的行索引
front_data = front_ori_data(valid_front_idx, :); % 保留满足条件的行

% 处理后侧数据
back_length = back_ori_data(:, 4); % 提取第四列（丝杠长度）
valid_back_idx = back_length >= length_gate; % 找到满足条件的行索引
back_data = back_ori_data(valid_back_idx, :); % 保留满足条件的行

clear("back_ori_data");
clear("front_ori_data");
clear("front_length" , "valid_front_idx");
clear("back_length" , "valid_back_idx");

%% 开始绘图
subplot(3 , 2 , 1);
plot(front_data(: , 4) , "LineWidth", 1.5);
xlabel("序号");
ylabel("丝杠行程(mm)");
title("丝杠行程-前");

subplot(3 , 2 , 2);
plot(back_data(: , 4) , "LineWidth", 1.5);
xlabel("序号");
ylabel("丝杠行程(mm)");
title("丝杠行程-后");

subplot(3 , 2 , 3);
plot(front_data(: , 3) , "LineWidth", 0.5);
xlabel("序号");
ylabel("电流");
title("电流-前");

subplot(3 , 2 , 4);
plot(back_data(: , 3) , "LineWidth", 1.5);
xlabel("序号");
ylabel("电流");
title("电流-后");

subplot(3 , 2 , 5);
hold on;
plot(front_data(: , 1:2))
legend( '左' , '右' );
xlabel("序号");
ylabel("压缩量(mm)");
title("压缩量读数-前");

subplot(3 , 2 , 6);
hold on;
plot(back_data(: , 1:2))
legend( '左' , '右' );
xlabel("序号");
ylabel("压缩量(mm)");
title("压缩量读数-后");
