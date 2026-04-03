%% WHEELTEC C50X AFC 调试数据分析（电机 B）
% 用途：分析含 AFC（自适应前馈补偿）的调试数据，对比 PI 与 AFC 各自贡献
% 数据来源：afc_output_b = AFC 增量 PWM；output_b = PI+AFC 总 PWM
% 使用：修改 CSV_FILE 后直接运行

clc; clear; close all;

%% ========== 配置区 ==========
CSV_FILE = "D:\radar\car\debug_monitor\afc_data_kp6000_ki150_kd0_20260402_161306.csv";

AUTO_TRIM   = true;
TRIM_MARGIN = 0.5;   % 有效段前后各保留的秒数

EXPORT_PNG  = false;
EXPORT_DPI  = 150;

FONT_SIZE = 12;

% 稳态判定：|实际速度 - 目标速度| < 目标速度 * 相对阈值 + 绝对阈值
SETTLE_REL  = 0.10;   % 相对阈值：目标速度的 10%
SETTLE_ABS  = 0.05;   % 绝对阈值：0.05 m/s
%% ==============================

%% 1. 读取数据
fprintf('正在读取: %s\n', CSV_FILE);
data = readtable(CSV_FILE, 'VariableNamingRule', 'preserve');
data.Properties.VariableNames = strtrim(data.Properties.VariableNames);

t        = data.time_s;
final_b  = data.final_b;
target_b = data.target_b;
output_b = double(data.output_b);
afc_b    = double(data.afc_output_b);

% PI 分量 = 总输出 - AFC 增量
pi_b = output_b - afc_b;

fprintf('总帧数: %d，时长: %.2f s，采样率: %.0f Hz\n', ...
    height(data), t(end)-t(1), 1/median(diff(t)));

%% 2. 解析文件名 PID 参数
[~, fname, ~] = fileparts(char(CSV_FILE));
kp_m = regexp(fname, 'kp(\d+)', 'tokens');
ki_m = regexp(fname, 'ki(\d+)', 'tokens');
kd_m = regexp(fname, 'kd(\d+)', 'tokens');
kp = '?'; ki = '?'; kd = '?';
if ~isempty(kp_m), kp = kp_m{1}{1}; end
if ~isempty(ki_m), ki = ki_m{1}{1}; end
if ~isempty(kd_m), kd = kd_m{1}{1}; end
pid_label = sprintf('Kp=%s  Ki=%s  Kd=%s', kp, ki, kd);

%% 3. 裁剪有效运动段
if AUTO_TRIM
    active = abs(target_b) > 0.01 | abs(final_b) > 0.05;
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
    vars = {'t','final_b','target_b','output_b','afc_b','pi_b'};
    for i = 1:numel(vars)
        eval([vars{i} ' = ' vars{i} '(mask);']);
    end
    t = t - t(1);   % 归零
end

%% 4. 统计分析（仅稳态段：排除启动过渡段）
err_b = final_b - target_b;

settled_b = abs(target_b) > 0.01 & ...
            abs(err_b) < SETTLE_REL * abs(target_b) + SETTLE_ABS;

if sum(settled_b) < 5
    warning('稳态样本不足（%d 点），请检查 SETTLE_REL/SETTLE_ABS 阈值', sum(settled_b));
end

err_mean_b = mean(err_b(settled_b));
err_std_b  = std( err_b(settled_b));

mot_pwm = abs(output_b) > 10;
afc_ratio_b = mean(abs(afc_b(mot_pwm)) ./ (abs(output_b(mot_pwm)) + 1)) * 100;

fprintf('\n===== 统计摘要（电机 B，稳态段）=====\n');
fprintf('稳态样本: %d 点（%.1f s）\n', sum(settled_b), sum(settled_b)/100);
fprintf('速度误差均值: %+.4f m/s，标准差: %.4f m/s\n', err_mean_b, err_std_b);
fprintf('PWM 总输出范围: [%d, %d]\n', min(output_b), max(output_b));
fprintf('AFC 增量范围:   [%.1f, %.1f]\n', min(afc_b), max(afc_b));
fprintf('AFC 均值贡献占比: %.1f%%\n', afc_ratio_b);
fprintf('稳态判定阈值：相对 %.0f%%，绝对 %.2f m/s\n', SETTLE_REL*100, SETTLE_ABS);
fprintf('=======================================\n\n');

%% ===== 绘图公共设置 =====
C_ACT  = [1.000, 0.498, 0.055];   % 橙  — 实际速度
C_TGT  = [0.580, 0.404, 0.741];   % 紫  — 目标速度
C_TOT  = [0.890, 0.467, 0.761];   % 粉  — 总 PWM
C_AFC  = [0.737, 0.741, 0.133];   % 黄绿 — AFC 分量
C_PI   = [0.500, 0.500, 0.500];   % 灰  — PI 分量
LW = 0.8;

set_ax = @(ax) set(ax, 'FontSize', FONT_SIZE, 'GridAlpha', 0.3, ...
    'GridLineStyle', '--', 'Box', 'on');

%% ===== Figure 1：速度跟踪（含误差）=====
fig1 = figure('Name', '电机 B 速度跟踪', 'NumberTitle', 'off', ...
    'Position', [80 420 950 500]);

ax1a = subplot(2,1,1);
hold on;
plot(t, target_b, '--', 'Color', C_TGT, 'LineWidth', LW, 'DisplayName', '目标速度');
plot(t, final_b,  '-',  'Color', C_ACT, 'LineWidth', LW, 'DisplayName', '实际速度');
hold off;
ylabel('速度 (m/s)', 'FontSize', FONT_SIZE);
title(sprintf('电机 B 速度跟踪  （AFC 开启）  %s', pid_label), 'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-1);
grid on; set_ax(ax1a);

ax1b = subplot(2,1,2);
hold on;
plot(t, err_b, '-', 'Color', C_ACT, 'LineWidth', LW, 'DisplayName', ...
    sprintf('误差  μ=%+.3f σ=%.3f (稳态)', err_mean_b, err_std_b));
plot(t(settled_b), err_b(settled_b), '.', 'Color', C_ACT, 'MarkerSize', 3, ...
    'HandleVisibility', 'off');
hold off;
yline(0, 'k--', 'LineWidth', 0.8);
ylabel('误差 (m/s)', 'FontSize', FONT_SIZE);
xlabel('时间 (s)', 'FontSize', FONT_SIZE);
title(sprintf('跟踪误差（稳态判定：±%.0f%% + %.2f m/s，点=统计样本）', SETTLE_REL*100, SETTLE_ABS), ...
    'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-1);
grid on; set_ax(ax1b);

linkaxes([ax1a, ax1b], 'x'); xlim([t(1), t(end)]);

%% ===== Figure 2：PWM 分解（总输出 / PI / AFC）=====
fig2 = figure('Name', '电机 B PWM 分解', 'NumberTitle', 'off', ...
    'Position', [100 380 950 380]);

ax2 = axes;
hold on;
plot(t, output_b, '-', 'Color', C_TOT, 'LineWidth', LW+0.4, 'DisplayName', '总输出 (PI+AFC)');
plot(t, pi_b,     '-', 'Color', C_PI,  'LineWidth', LW,     'DisplayName', 'PI 分量');
plot(t, afc_b,    '-', 'Color', C_AFC, 'LineWidth', LW,     'DisplayName', 'AFC 分量');
hold off;
yline(0, 'k--', 'LineWidth', 0.8);
ylabel('PWM 值', 'FontSize', FONT_SIZE);
xlabel('时间 (s)', 'FontSize', FONT_SIZE);
title(sprintf('电机 B PWM 分解   AFC 贡献占比 %.1f%%   (%s)', afc_ratio_b, pid_label), ...
    'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-1);
grid on; set_ax(ax2); xlim([t(1), t(end)]);

%% ===== Figure 3：AFC 学习曲线 =====
fig3 = figure('Name', '电机 B AFC 学习曲线', 'NumberTitle', 'off', ...
    'Position', [120 340 950 330]);

ax3 = axes;
plot(t, afc_b, '-', 'Color', C_AFC, 'LineWidth', LW);
yline(0, 'k--', 'LineWidth', 0.8);
ylabel('AFC 增量 PWM', 'FontSize', FONT_SIZE);
xlabel('时间 (s)', 'FontSize', FONT_SIZE);
title(sprintf('电机 B AFC 补偿量随时间变化   (%s)', pid_label), 'FontSize', FONT_SIZE);
grid on; set_ax(ax3); xlim([t(1), t(end)]);

%% ===== Figure 4：汇报主图（3 行）=====
fig4 = figure('Name', '综合分析（汇报用）', 'NumberTitle', 'off', ...
    'Position', [50 30 1000 780]);

% 第1行：速度跟踪
ax4_1 = subplot(3,1,1);
hold on;
plot(t, target_b, '--', 'Color', C_TGT, 'LineWidth', LW, 'DisplayName', '目标速度');
plot(t, final_b,  '-',  'Color', C_ACT, 'LineWidth', LW, 'DisplayName', '实际速度');
hold off;
ylabel('速度 (m/s)', 'FontSize', FONT_SIZE);
title('速度跟踪', 'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-2);
grid on; set_ax(ax4_1); xlim([t(1), t(end)]);

% 第2行：PWM 分解
ax4_2 = subplot(3,1,2);
hold on;
plot(t, output_b, '-', 'Color', C_TOT, 'LineWidth', LW+0.4, 'DisplayName', '总输出 (PI+AFC)');
plot(t, pi_b,     '-', 'Color', C_PI,  'LineWidth', LW,     'DisplayName', 'PI 分量');
plot(t, afc_b,    '-', 'Color', C_AFC, 'LineWidth', LW,     'DisplayName', 'AFC 分量');
hold off;
ylabel('PWM 值', 'FontSize', FONT_SIZE);
title(sprintf('PWM 分解（AFC 占比 %.1f%%）', afc_ratio_b), 'FontSize', FONT_SIZE);
legend('Location', 'best', 'FontSize', FONT_SIZE-2);
grid on; set_ax(ax4_2); xlim([t(1), t(end)]);

% 第3行：AFC 补偿量
ax4_3 = subplot(3,1,3);
plot(t, afc_b, '-', 'Color', C_AFC, 'LineWidth', LW);
yline(0, 'k--', 'LineWidth', 0.8);
ylabel('AFC 增量 PWM', 'FontSize', FONT_SIZE);
xlabel('时间 (s)', 'FontSize', FONT_SIZE);
title('AFC 补偿量', 'FontSize', FONT_SIZE);
grid on; set_ax(ax4_3); xlim([t(1), t(end)]);

linkaxes([ax4_1, ax4_2, ax4_3], 'x');

sgtitle('电机 B — AFC 补偿直线前进测试', ...
    'FontSize', FONT_SIZE+2, 'FontWeight', 'bold');

%% ===== 可选：导出 PNG =====
if EXPORT_PNG
    export_fig_safe = @(fig, name) print(fig, name, '-dpng', ...
        sprintf('-r%d', EXPORT_DPI));
    folder = fileparts(char(CSV_FILE));
    if isempty(folder), folder = pwd; end
    export_fig_safe(fig1, fullfile(folder, [fname '_B_speed.png']));
    export_fig_safe(fig2, fullfile(folder, [fname '_B_pwm_decomp.png']));
    export_fig_safe(fig3, fullfile(folder, [fname '_B_afc_curve.png']));
    export_fig_safe(fig4, fullfile(folder, [fname '_B_summary.png']));
    fprintf('图片已导出至: %s\n', folder);
end

fprintf('分析完成。\n');
