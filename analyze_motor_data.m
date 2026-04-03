%% WHEELTEC C50X 电机调试数据分析
% 用途：读取串口调试上位机导出的 CSV，绘制速度跟踪与 PWM 输出图
% 使用：将此文件放在 CSV 同目录，或修改下方 CSV_FILE 路径
% 作者：自动生成  日期：2026-04-02

clc; clear; close all;

%% ========== 配置区（修改这里）==========
CSV_FILE = "D:\radar\car\debug_monitor\debug_data_kp6000_ki150_kd0_20260326_180423.csv"
% 是否自动裁剪到有效运动段（去掉首尾全零部分）
AUTO_TRIM   = true;
TRIM_MARGIN = 0.5;   % 有效段前后各保留的秒数

% 是否在最后导出 PNG（true = 保存，false = 仅显示）
EXPORT_PNG  = false;
EXPORT_DPI  = 150;

% 绘图字体大小
FONT_SIZE = 12;

% 稳态判定：|实际速度 - 目标速度| < 目标速度 * 相对阈值 + 绝对阈值
% 同时要求目标速度非零，且持续满足条件（不统计加速段）
SETTLE_REL  = 0.10;   % 相对阈值：目标速度的 10%
SETTLE_ABS  = 0.05;   % 绝对阈值：0.05 m/s（防止目标值很小时比例过严）
%% =======================================

%% 1. 读取数据
fprintf('正在读取: %s\n', CSV_FILE);
data = readtable(CSV_FILE, 'VariableNamingRule', 'preserve');

% 兼容列名首尾空格
data.Properties.VariableNames = strtrim(data.Properties.VariableNames);

t        = data.time_s;
final_a  = data.final_a;
final_b  = data.final_b;
target_a = data.target_a;
target_b = data.target_b;
output_a = double(data.output_a);
output_b = double(data.output_b);

fprintf('总帧数: %d，时长: %.2f s，采样率: %.0f Hz\n', ...
    height(data), t(end)-t(1), 1/median(diff(t)));

%% 2. 从文件名解析 PID 参数（用于标题）
[~, fname, ~] = fileparts(CSV_FILE);
pid_str = fname;
kp_match = regexp(fname, 'kp(\d+)', 'tokens'); kp = '';
ki_match = regexp(fname, 'ki(\d+)', 'tokens'); ki = '';
kd_match = regexp(fname, 'kd(\d+)', 'tokens'); kd = '';
if ~isempty(kp_match), kp = kp_match{1}{1}; end
if ~isempty(ki_match), ki = ki_match{1}{1}; end
if ~isempty(kd_match), kd = kd_match{1}{1}; end
if ~isempty(kp)
    pid_label = sprintf('Kp=%s  Ki=%s  Kd=%s', kp, ki, kd);
else
    pid_label = fname;
end

%% 3. 裁剪有效运动段
if AUTO_TRIM
    active = abs(target_a) > 0.01 | abs(target_b) > 0.01 | ...
             abs(final_a)  > 0.05 | abs(final_b)  > 0.05;
    idx = find(active);
    if isempty(idx)
        warning('未检测到有效运动段，使用全部数据');
        mask = true(size(t));
    else
        t_start = t(idx(1))   - TRIM_MARGIN;
        t_end   = t(idx(end)) + TRIM_MARGIN;
        mask = t >= t_start & t <= t_end;
        fprintf('有效运动段: %.2f s ~ %.2f s（%.2f s）\n', ...
            t(idx(1)), t(idx(end)), t(idx(end))-t(idx(1)));
    end
    t        = t(mask);
    final_a  = final_a(mask);  final_b  = final_b(mask);
    target_a = target_a(mask); target_b = target_b(mask);
    output_a = output_a(mask); output_b = output_b(mask);
    % 时间轴归零，从 0 开始
    t = t - t(1);
end

%% 4. 统计分析（仅稳态段：速度已收敛到目标附近，排除启动过渡段）
% 稳态掩码：目标非零 且 误差在阈值范围内
settled_a = abs(target_a) > 0.01 & ...
            abs(final_a - target_a) < SETTLE_REL * abs(target_a) + SETTLE_ABS;
settled_b = abs(target_b) > 0.01 & ...
            abs(final_b - target_b) < SETTLE_REL * abs(target_b) + SETTLE_ABS;

if sum(settled_a) < 5
    warning('电机 A 稳态样本不足（%d 点），请检查 SETTLE_REL/SETTLE_ABS 阈值', sum(settled_a));
end
if sum(settled_b) < 5
    warning('电机 B 稳态样本不足（%d 点），请检查 SETTLE_REL/SETTLE_ABS 阈值', sum(settled_b));
end

err_a = final_a - target_a;
err_b = final_b - target_b;

err_mean_a = mean(err_a(settled_a));  err_std_a = std(err_a(settled_a));
err_mean_b = mean(err_b(settled_b));  err_std_b = std(err_b(settled_b));

fprintf('\n===== 统计摘要（稳态段）=====\n');
fprintf('电机 A — 稳态样本: %d 点（%.1f s），误差均值: %+.4f m/s，标准差: %.4f m/s\n', ...
    sum(settled_a), sum(settled_a)/100, err_mean_a, err_std_a);
fprintf('电机 B — 稳态样本: %d 点（%.1f s），误差均值: %+.4f m/s，标准差: %.4f m/s\n', ...
    sum(settled_b), sum(settled_b)/100, err_mean_b, err_std_b);
fprintf('PWM A  — 范围: [%d, %d]，均值: %.1f\n', min(output_a), max(output_a), mean(output_a));
fprintf('PWM B  — 范围: [%d, %d]，均值: %.1f\n', min(output_b), max(output_b), mean(output_b));
fprintf('稳态判定阈值：相对 %.0f%%，绝对 %.2f m/s\n', SETTLE_REL*100, SETTLE_ABS);
fprintf('==============================\n\n');

%% ===== 绘图公共设置 =====
C_ACT  = [0.122, 0.467, 0.706];   % 蓝色 — 实际速度
C_TGT  = [0.839, 0.153, 0.157];   % 红色 — 目标速度
C_ERR  = [0.173, 0.627, 0.173];   % 绿色 — 误差
C_PWMA = [0.549, 0.337, 0.294];   % 棕色 — PWM A
C_PWMB = [0.890, 0.467, 0.761];   % 粉色 — PWM B
LW = 0.8;

set_ax = @(ax) set(ax, 'FontSize', FONT_SIZE, 'GridAlpha', 0.3, ...
    'GridLineStyle', '--', 'Box', 'on');

%% ===== Figure 1：A/B 速度跟踪叠加 =====
fig1 = figure('Name', 'A/B 速度跟踪', 'NumberTitle', 'off', ...
    'Position', [100 400 900 500]);

C_ACT_B = [1.000, 0.498, 0.055];   % 橙色 — 电机 B 实际速度
C_TGT_B = [0.580, 0.404, 0.741];   % 紫色 — 电机 B 目标速度

ax1a = subplot(2,1,1);
hold on;
plot(t, target_a, '--', 'Color', C_TGT,   'LineWidth', LW, 'DisplayName', '目标 A');
plot(t, final_a,  '-',  'Color', C_ACT,   'LineWidth', LW, 'DisplayName', '实际 A');
plot(t, target_b, '--', 'Color', C_TGT_B, 'LineWidth', LW, 'DisplayName', '目标 B');
plot(t, final_b,  '-',  'Color', C_ACT_B, 'LineWidth', LW, 'DisplayName', '实际 B');
hold off;
ylabel('速度 (m/s)', 'FontSize', FONT_SIZE);
title(sprintf('A/B 速度跟踪   (%s)', pid_label), 'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-1, 'NumColumns', 2);
grid on; set_ax(ax1a);

ax1b = subplot(2,1,2);
hold on;
% 稳态区间背景阴影
fill_settled = @(mask, clr) fill(t([find(mask,1,'first') find(mask,1,'first') ...
    find(mask,1,'last') find(mask,1,'last')], 1), ...
    [min(ylim)-1 max(ylim)+1 max(ylim)+1 min(ylim)-1], clr, ...
    'FaceAlpha', 0.08, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(t, err_a, '-', 'Color', C_ACT,   'LineWidth', LW, 'DisplayName', sprintf('误差 A  μ=%+.3f σ=%.3f (稳态)', err_mean_a, err_std_a));
plot(t, err_b, '-', 'Color', C_ACT_B, 'LineWidth', LW, 'DisplayName', sprintf('误差 B  μ=%+.3f σ=%.3f (稳态)', err_mean_b, err_std_b));
% 用散点标出参与统计的稳态点
plot(t(settled_a), err_a(settled_a), '.', 'Color', C_ACT,   'MarkerSize', 3, 'HandleVisibility', 'off');
plot(t(settled_b), err_b(settled_b), '.', 'Color', C_ACT_B, 'MarkerSize', 3, 'HandleVisibility', 'off');
hold off;
yline(0, 'k--', 'LineWidth', 0.8);
ylabel('误差 (m/s)', 'FontSize', FONT_SIZE);
xlabel('时间 (s)', 'FontSize', FONT_SIZE);
title(sprintf('跟踪误差（稳态判定：±%.0f%% + %.2f m/s，点=统计样本）', SETTLE_REL*100, SETTLE_ABS), ...
    'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-1);
grid on; set_ax(ax1b);

linkaxes([ax1a, ax1b], 'x');
xlim([t(1), t(end)]);

%% ===== Figure 2（保留，单独对比）已合并，跳过 =====

%% ===== Figure 3：A/B PWM 叠加 =====
fig3 = figure('Name', 'A/B PWM 输出', 'NumberTitle', 'off', ...
    'Position', [140 360 900 350]);

ax3 = axes;
hold on;
plot(t, output_a, '-', 'Color', C_PWMA, 'LineWidth', LW, 'DisplayName', 'PWM A');
plot(t, output_b, '-', 'Color', C_PWMB, 'LineWidth', LW, 'DisplayName', 'PWM B');
hold off;
ylabel('PWM 值', 'FontSize', FONT_SIZE);
xlabel('时间 (s)', 'FontSize', FONT_SIZE);
title(sprintf('A/B PWM 输出   (%s)', pid_label), 'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-1);
grid on; set_ax(ax3);
xlim([t(1), t(end)]);

%% ===== Figure 4：汇报主图（上：A/B速度叠加，下：A/B PWM叠加）=====
fig4 = figure('Name', '综合分析（汇报用）', 'NumberTitle', 'off', ...
    'Position', [50 50 1100 650]);

% 上：A/B 速度叠加
ax4_1 = subplot(2,1,1);
hold on;
plot(t, target_a, '--', 'Color', C_TGT,   'LineWidth', LW, 'DisplayName', '目标 A');
plot(t, final_a,  '-',  'Color', C_ACT,   'LineWidth', LW, 'DisplayName', '实际 A');
plot(t, target_b, '--', 'Color', C_TGT_B, 'LineWidth', LW, 'DisplayName', '目标 B');
plot(t, final_b,  '-',  'Color', C_ACT_B, 'LineWidth', LW, 'DisplayName', '实际 B');
hold off;
ylabel('速度 (m/s)', 'FontSize', FONT_SIZE);
title('A/B 速度跟踪', 'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-2, 'NumColumns', 2);
grid on; set_ax(ax4_1); xlim([t(1), t(end)]);

% 下：A/B PWM 叠加
ax4_2 = subplot(2,1,2);
hold on;
plot(t, output_a, '-', 'Color', C_PWMA, 'LineWidth', LW, 'DisplayName', 'PWM A');
plot(t, output_b, '-', 'Color', C_PWMB, 'LineWidth', LW, 'DisplayName', 'PWM B');
hold off;
ylabel('PWM 值', 'FontSize', FONT_SIZE);
xlabel('时间 (s)', 'FontSize', FONT_SIZE);
title('A/B PWM 输出', 'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-2);
grid on; set_ax(ax4_2); xlim([t(1), t(end)]);

linkaxes([ax4_1, ax4_2], 'x');

% 总标题
sgtitle(sprintf('WHEELTEC C50X 电机直线前进测试'), ...
    'FontSize', FONT_SIZE+2, 'FontWeight', 'bold');

%% ===== 可选：导出 PNG =====
if EXPORT_PNG
    export_fig_safe = @(fig, name) print(fig, name, '-dpng', ...
        sprintf('-r%d', EXPORT_DPI));
    [folder, ~, ~] = fileparts(which(CSV_FILE));
    if isempty(folder), folder = pwd; end
    export_fig_safe(fig1, fullfile(folder, [fname '_motorA_speed.png']));
    export_fig_safe(fig2, fullfile(folder, [fname '_motorB_speed.png']));
    export_fig_safe(fig3, fullfile(folder, [fname '_pwm.png']));
    export_fig_safe(fig4, fullfile(folder, [fname '_summary.png']));
    fprintf('图片已导出至: %s\n', folder);
end

fprintf('分析完成。\n');
