% 假设你的数据已经加载为列向量 x 和 y
% 例如：x = [...]; y = [...];

x = load("dia.txt");
y = load("angle.txt");

% 6次多项式拟合
p = polyfit(x, y, 6);

% 生成拟合曲线上的点
x_fit = linspace(min(x), max(x), 200);
y_fit = polyval(p, x_fit);

% 绘制原始数据和拟合曲线
figure;
scatter(x, y, 'b', 'filled'); hold on;
plot(x_fit, y_fit, 'r-', 'LineWidth', 2);
xlabel('x');
ylabel('y');
title('6次多项式拟合');
legend('原始数据', '6次多项式拟合');
grid on;

disp(p);