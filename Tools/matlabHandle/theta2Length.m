function length = theta2Length(input_theta)
    %% 定义机械参数取值
    l1 = 62.0;
    l2 = 270.0;
    l3 = 115.0;
    R = 300.0;
    r = 180.0;
    rw = 50.0;
    deg_alpha = 51.8748;
    
    %% 开始求解
%     input_theta = 37.4321; % 单位为°
    rad_theta = input_theta / 180.0 * pi;
    input_value = l3 + (1 - cos(rad_theta)) * (r + R) - cos(rad_theta) * rw;
    % 直接使用匿名函数
    valueFunc = @(m) (l3 - rw) *(2 * m * m - 1) + 2 * l1 * m * sqrt( 1 - m * m) + l2 * sqrt( 1 - m*m) - input_value;
    cos_beta0 = 0.9;
    % options = optimset('Display' , 'iter');
    [cos_beta , ~] = fsolve(valueFunc , cos_beta0 );
    disp(cos_beta);
    rad_beta = acos(cos_beta) ;
    deg_beta = rad_beta / pi * 180.0;
    deg_answer = deg_alpha - deg_beta;
    length = power(100.4241 , 2) + power(83 , 2) - 2 * 100.4241 * 83 * cos(deg_answer / 180 * pi);
    length = sqrt(length);
end

