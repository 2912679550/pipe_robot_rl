%% 本脚本用于绘制收集到的rosbag数据，绘制夹紧过程的电流与夹紧长度关系图
clc ; close all; clear;
front_ori_data = load("data\output_f.csv");
back_ori_data  = load("data\output_b.csv");

length_gate = 0.2; % 丝杠行程小于0.5mm的部分将被剔除

%% 数据预处理：删除第四列（丝杠长度）小于 length_gate 的行
% 处理前侧数据
front_data = abs(front_ori_data); % 提取第四列（丝杠长度）


% 处理后侧数据
back_length = abs(back_ori_data); % 提取第四列（丝杠长度）
valid_back_idx = back_length(: , 1) >= length_gate; % 找到满足条件的行索引
back_data = back_ori_data(valid_back_idx, :); % 保留满足条件的行


clear("back_ori_data");
clear("front_ori_data");
clear("front_length" , "valid_front_idx");
clear("back_length" , "valid_back_idx");

%% 开始绘图
% subplot(2 , 1 , 1);
% hold on;
% plot(front_data(: , 1:2))
% legend( '左' , '右' );
% xlabel("序号");
% ylabel("回传电流值(A)");
% title("前侧");
% 
% subplot(2 , 1 , 2);
hold on;
plot(back_data(: , 1:2))
legend( '左' , '右' );
xlabel("序号");
ylabel("回传电流值(A)");
title("后侧");
