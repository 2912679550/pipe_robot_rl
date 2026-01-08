clear;clc;
% 生成 5x5 有波动的矩阵，最终每个元素保留两位小数，且全体均值为 target_mean
beta = 1e4;
target_mean = 121.93 * beta;
nrows = 5; ncols = 5;
n = nrows * ncols;

% 1) 生成有波动的初始数据（可调整尺度）
A = randn(nrows, ncols) * 1.0 * beta + target_mean;   % randn 可产生正态波动，系数控制幅度
A = A - mean(A(:)) + target_mean;              % 平移保证未量化前的均值为 target_mean

% 2) 将值转换为“分”（cent）单位（非整数），并用 floor + largest-remainder 分配法
orig_cents = A * 100;          % 每个元素以分为单位（浮点）
floored = floor(orig_cents);   % 先下取整（分）
residuals = orig_cents - floored;

n = nrows * ncols;
target_sum_cents = round(target_mean * 100) * n;  % 目标总和（分），为整数
needed = target_sum_cents - sum(floored);         % 需要额外分配的 1-cent 数目（整数）

% 安全保护：避免 needed 超出索引范围
needed = max(min(needed, n), -n);

% 3) 把 needed 个 1-cent 分配给 residuals 最大的位置（或从最小处减去）
final_cents = floored;
if needed > 0
    [~, idx_sorted] = sort(residuals(:), 'descend');
    idx = idx_sorted(1:needed);
    final_cents(idx) = final_cents(idx) + 1;
elseif needed < 0
    [~, idx_sorted] = sort(residuals(:), 'ascend');
    idx = idx_sorted(1:-needed);
    final_cents(idx) = final_cents(idx) - 1;
end

% 4) 恢复为两位小数矩阵并输出表格
B = reshape(final_cents, nrows, ncols) / 100;
T_p = ones(size(B)) * 500.0;
T_p = T_p ./ B * beta;

T = array2table(B, 'VariableNames', {'C1','C2','C3','C4','C5'});
disp(T);
fprintf('Mean = %.2f\n', mean(B(:)));

