%% 本脚用于探究自动过弯时的主驱动轮线速度与辅助驱动轮线速度之间的关系
close all; clc; clear;

%% 定义机械参数取值
% 可变的管道尺寸参数, 以及期望的进弯速度，单位为m/s
% R = 300.0 / 1000;
% r = 180.0 / 1000;
R = 300.0;
r = 180.0;
target_v = 0.02;


% 固定的机器人尺寸参数
% l1 = 62.0 / 1000;
% l2 = 270.0 / 1000;
% l3 = 115.0 / 1000;
% l4 = 280.12 / 1000;
l1 = 62.0;
l2 = 270.0;
l3 = 115.0;
l4 = l3 + 1.5 * r;
deg_alpha = 51.8748;
rw = 50.0;



% 全输入形式的非线性几何约束关系
f = @(b , t) ...
    (l3 - rw) *(2 * cos(b) * cos(b) - 1) + ...
    2 * l1 * cos(b) * sqrt( 1 - cos(b) * cos(b)) + ...
    l2 * sqrt( 1 - cos(b)*cos(b)) - ...
    ( l3 + (1 - cos(t)) * (r + R) - cos(t) * rw );
x = @(b , t) ...
    (l1 + l2 / (2 * cos(b) )) * (1 + cos(2 * b)) - (l3 - rw) * sin(2 * b) - sin(t) * (R + r + rw);
t_ac = @(b) ...
    l1 + l2 / (2 * cos(b));
x_q = @(b , t) ... 
    t_ac(b) * (1 + cos( 2 * b)) - l4 * sin( 2 * b ) - x(b , t);
y_q = @(b) ...
    -t_ac(b) * sin(2 * b) - l4 * cos(2 *b) + l3 + R + r;

%% 开始求解
% 生成虚拟进弯角度 theta 数值
input_theta = (1.5 : 0.2 :50)';
% 结果容器
result_rad_beta = [];
target_length = [];
M_v_aix = [];
M_v_cir = [];
Q_v_aix = [];
Q_v_cir = [];
M_x = [];
M_y = [];
Q_x = [];
Q_y = [];

%% 求解迭代
for i = 1 : size(input_theta , 1)
    temp_theta = input_theta(i);
    rad_theta = temp_theta / 180.0 * pi;
% 1. 先求解当前对应的beta值
    input_value = l3 + (1 - cos(rad_theta)) * (r + R) - cos(rad_theta) * rw; % 构造的非线性方程的右侧部分
    % 构造待求微分方程，注意方程中的beta单位为弧度
    valueFunc = @(m) (l3 - rw) *(2 * cos(m) * cos(m) - 1) + 2 * l1 * cos(m) * sqrt( 1 - cos(m) * cos(m)) + l2 * sqrt( 1 - cos(m)*cos(m)) - input_value;
    beta0 = 0.0 / 180 * pi;    % 给定beta的初值
    [rad_beta , ~] = fsolve(valueFunc , beta0 );
    
    cos_beta = cos(rad_beta);
    deg_beta = rad_beta / pi * 180.0;

% 2. 验算此刻后侧轮子距离进弯的长度x，如果x接近0，说明可以停止进弯动作了
    length_x = x(rad_beta , rad_theta);
    if length_x <= 0.01
        break;
    end

% 3. 求解并存储当前的推杆长度
    deg_answer = deg_alpha - deg_beta;
    length = power(100.4241 , 2) + power(83 , 2) - 2 * 100.4241 * 83 * cos(deg_answer / 180 * pi);
    length = sqrt(length);
    

% 4. 求解直角坐标系表示下的M点速度
    theta_dot = target_v / ( (R +r) );    % 单位为rad/s
    dxm_dtheta = cos(rad_theta) * (R + r) ; % 单位为m
    dym_dtheta =-sin(rad_theta) * (R + r) ; % 单位为m
    x_m = sin(rad_theta) * (R + r);
    y_m = cos(rad_theta) * (R + r);
    v_xm = dxm_dtheta * theta_dot;
    v_ym = dym_dtheta * theta_dot;

% 5. 采用数值微分的方式求解beta与theta的非线性方程与两个角度的偏分关系
    ori_f = f(rad_beta , rad_theta);
    delta = 1e-4;
    new_theta = rad_theta + delta;
    new_beta  = rad_beta + delta;
    
    new_f_beta = f(new_beta , rad_theta);
    new_f_theta = f(rad_beta , new_theta);

    df_dbeta = (new_f_beta - ori_f) / delta;
    df_dtheta = (new_f_theta - ori_f) / delta;

% 6. 得到dbeta_d_theta ， 最终得到 beta_dot
    dbeta_dtheta = -df_dtheta / df_dbeta;
    beta_dot = dbeta_dtheta * theta_dot;
 
% 7. 采用数值微分的方式求解x_q与y_q分别与beta的偏导数
    x_q_ori = x_q(rad_beta , rad_theta);
    y_q_ori = y_q(rad_beta);
    
    delta = 1e-4;
    new_beta  = rad_beta + delta;
    new_theta = rad_theta + delta;

    new_xq_beta = x_q(new_beta , rad_theta);
    new_xq_theta = x_q(rad_beta , new_theta);
    new_yq = y_q(new_beta);

    dxq_dbeta = (new_xq_beta - x_q_ori) / delta;
    dxq_dtheta = (new_xq_theta - x_q_ori) / delta;
    dyq_dbeta = (new_yq - y_q_ori) / delta;

% 8. 解得q点的xy速度大小
    v_xq = dxq_dbeta * beta_dot + dxq_dtheta * theta_dot;
    v_yq = dyq_dbeta * beta_dot;

% 9. 将数据坐标转换为舵轮的轴向与周向速度形式
    v_m_aix = v_xm * cos(rad_theta) - v_ym * sin(rad_theta);
    v_m_cir = 0.0;
    v_q_aix =  v_xq * cos(2 * rad_beta) - v_yq * sin(2 * rad_beta);
    v_q_cir = -v_xq * sin(2 * rad_beta) - v_yq * cos(2 * rad_beta);

% 10. 存储数据
    result_rad_beta = [result_rad_beta ; rad_beta]; % 存储此时beta值
    target_length = [target_length ; length];
    M_v_aix = [M_v_aix ; v_m_aix];
    M_v_cir = [M_v_cir ; v_m_cir];
    Q_v_aix = [Q_v_aix ; v_q_aix];
    Q_v_cir = [Q_v_cir ; v_q_cir];
    M_x = [M_x ; x_m];
    M_y = [M_y ; y_m];
    Q_x = [Q_x ; x_q_ori];
    Q_y = [Q_y ; y_q_ori];

end

%% 解算结束，开始绘图
total_num = size(result_rad_beta , 1);
used_theta_deg = input_theta(1 : total_num);
% 绘制 theta - beta 角关系图
subplot(4 , 2 , 1);
deg_result_beta = result_rad_beta * 180 / pi;
plot(used_theta_deg , deg_result_beta , LineWidth=2.0);
title('进弯theta角与beta角关系图');
xlabel('theta(°)');
ylabel('beta(°)');
% 绘制 theta - length关系图
subplot(4 , 2 , 2);
plot(used_theta_deg , target_length , LineWidth=2.0);
title('进弯theta角与推杆长度关系图');
xlabel('theta(°)');
ylabel('length(mm)');
% 绘制 主动轮 接触点轨迹图
subplot(4 , 2 , 3);
plot(M_x , M_y , LineWidth=2.0);
title('主动轮与管道接触点轨迹图');
xlabel('x(mm)');
ylabel('y(mm)');
% 绘制 辅助轮 接触点轨迹图
subplot(4 , 2 , 4);
plot(Q_x , Q_y , LineWidth=2.0);
title('辅助轮与管道接触点轨迹图');
xlabel('x(mm)');
ylabel('y(mm)');

% 绘制 主动轮 轴向速度 期望图
subplot(4 , 2 , 5);
plot(used_theta_deg , M_v_aix , LineWidth=2.0);
title('主动轮轴向速度与进弯角度图');
xlabel('theta(°)');
ylabel('v_aix(m/s)');

% 绘制 主动轮 周向速度 期望图
subplot(4 , 2 , 6);
plot(used_theta_deg , M_v_cir , LineWidth=2.0);
title('主动轮周向速度与进弯角度图');
xlabel('theta(°)');
ylabel('v_cir(m/s)');

% 绘制 辅助轮 轴向速度 期望图
subplot(4 , 2 , 7);
plot(used_theta_deg , Q_v_aix , LineWidth=2.0);
title('辅助轮轴向速度与进弯角度图');
xlabel('theta(°)');
ylabel('v_aix(m/s)');

% 绘制 辅助轮 周向速度 期望图
subplot(4 , 2 , 8);
plot(used_theta_deg , Q_v_cir , LineWidth=2.0);
title('辅助轮周向速度与进弯角度图');
xlabel('theta(°)');
ylabel('v_cir(m/s)');





