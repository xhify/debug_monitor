%% Sweep output-only low-pass filters for PWM3000 step identification
% Reads the exported motor A/B CSV files, applies a zero-phase low-pass
% filter to the measured speed only, identifies 2nd-order plants for a
% range of cutoff frequencies, and retunes PI controllers from each result.

clear; clc; close all;

%% Configuration
baseDir = fileparts(mfilename('fullpath'));
fileA = fullfile(baseDir, 'debug_data_pwm3000_20260407_155102_motor_a_for_matlab.csv');
fileB = fullfile(baseDir, 'debug_data_pwm3000_20260407_155102_motor_b_for_matlab.csv');

Ts = 0.01;
filterOrder = 2;
cutoffHzList = [0.3, 0.5, 0.8, 1.0];
modelOrder = 2;

%% Load data
tblA = readtable(fileA, 'VariableNamingRule', 'preserve');
tblB = readtable(fileB, 'VariableNamingRule', 'preserve');

requiredVars = {'time_s', 'u_pwm', 'y_speed_mps'};
assert(all(ismember(requiredVars, tblA.Properties.VariableNames)), ...
    'Motor A CSV is missing one of: %s', strjoin(requiredVars, ', '));
assert(all(ismember(requiredVars, tblB.Properties.VariableNames)), ...
    'Motor B CSV is missing one of: %s', strjoin(requiredVars, ', '));

fs = 1 / Ts;
yA_raw = tblA.y_speed_mps;
yB_raw = tblB.y_speed_mps;
uA = tblA.u_pwm;
uB = tblB.u_pwm;
tA = tblA.time_s;
tB = tblB.time_s;

numCutoffs = numel(cutoffHzList);

fitAList = nan(numCutoffs, 1);
fitBList = nan(numCutoffs, 1);
kpAList = nan(numCutoffs, 1);
kiAList = nan(numCutoffs, 1);
kdAList = nan(numCutoffs, 1);
kpBList = nan(numCutoffs, 1);
kiBList = nan(numCutoffs, 1);
kdBList = nan(numCutoffs, 1);
positiveAList = false(numCutoffs, 1);
positiveBList = false(numCutoffs, 1);

filteredA = cell(numCutoffs, 1);
filteredB = cell(numCutoffs, 1);
sysAList = cell(numCutoffs, 1);
sysBList = cell(numCutoffs, 1);
caList = cell(numCutoffs, 1);
cbList = cell(numCutoffs, 1);

%% Sweep cutoff frequencies
for idx = 1:numCutoffs
    cutoffHz = cutoffHzList(idx);
    wn = cutoffHz / (fs / 2);
    assert(wn > 0 && wn < 1, 'cutoffHz must be between 0 and Nyquist frequency.');

    [b, a] = butter(filterOrder, wn, 'low');

    yA_filt = filtfilt(b, a, yA_raw);
    yB_filt = filtfilt(b, a, yB_raw);

    filteredA{idx} = yA_filt;
    filteredB{idx} = yB_filt;

    idA_filt = iddata(yA_filt, uA, Ts, ...
        'TimeUnit', 'seconds', ...
        'InputName', {'PWM'}, ...
        'InputUnit', {'count'}, ...
        'OutputName', {'SpeedFiltered'}, ...
        'OutputUnit', {'m/s'}, ...
        'ExperimentName', sprintf('motor_a_%.2f_hz', cutoffHz));

    idB_filt = iddata(yB_filt, uB, Ts, ...
        'TimeUnit', 'seconds', ...
        'InputName', {'PWM'}, ...
        'InputUnit', {'count'}, ...
        'OutputName', {'SpeedFiltered'}, ...
        'OutputUnit', {'m/s'}, ...
        'ExperimentName', sprintf('motor_b_%.2f_hz', cutoffHz));

    sysA_filt = tfest(idA_filt, modelOrder);
    sysB_filt = tfest(idB_filt, modelOrder);

    CA_filt = pidtune(sysA_filt, 'PI');
    CB_filt = pidtune(sysB_filt, 'PI');

    fitAList(idx) = sysA_filt.Report.Fit.FitPercent;
    fitBList(idx) = sysB_filt.Report.Fit.FitPercent;
    kpAList(idx) = CA_filt.Kp;
    kiAList(idx) = CA_filt.Ki;
    kdAList(idx) = CA_filt.Kd;
    kpBList(idx) = CB_filt.Kp;
    kiBList(idx) = CB_filt.Ki;
    kdBList(idx) = CB_filt.Kd;
    positiveAList(idx) = CA_filt.Kp > 0 && CA_filt.Ki > 0;
    positiveBList(idx) = CB_filt.Kp > 0 && CB_filt.Ki > 0;

    sysAList{idx} = sysA_filt;
    sysBList{idx} = sysB_filt;
    caList{idx} = CA_filt;
    cbList{idx} = CB_filt;
end

summaryA = table(cutoffHzList(:), fitAList, kpAList, kiAList, kdAList, positiveAList, ...
    'VariableNames', {'cutoff_hz', 'fit_percent', 'kp', 'ki', 'kd', 'positive_gains'});
summaryB = table(cutoffHzList(:), fitBList, kpBList, kiBList, kdBList, positiveBList, ...
    'VariableNames', {'cutoff_hz', 'fit_percent', 'kp', 'ki', 'kd', 'positive_gains'});

validA = find(positiveAList);
validB = find(positiveBList);

bestAIdx = [];
bestBIdx = [];
if ~isempty(validA)
    [~, localIdx] = max(fitAList(validA));
    bestAIdx = validA(localIdx);
end
if ~isempty(validB)
    [~, localIdx] = max(fitBList(validB));
    bestBIdx = validB(localIdx);
end

%% Report summary
fprintf('Low-pass filtered identification sweep\n');
fprintf('  Sample time  : %.4f s\n', Ts);
fprintf('  Filter order : %d\n', filterOrder);
fprintf('  Model order  : %d\n', modelOrder);
fprintf('  Cutoffs (Hz) : %s\n\n', mat2str(cutoffHzList));

disp('Motor A sweep summary:');
disp(summaryA);

disp('Motor B sweep summary:');
disp(summaryB);

if isempty(bestAIdx)
    warning('Motor A has no cutoff with positive PI gains.');
else
    fprintf('Best Motor A cutoff: %.2f Hz, fit %.2f%%, Kp %.10g, Ki %.10g\n', ...
        cutoffHzList(bestAIdx), fitAList(bestAIdx), kpAList(bestAIdx), kiAList(bestAIdx));
end

if isempty(bestBIdx)
    warning('Motor B has no cutoff with positive PI gains.');
else
    fprintf('Best Motor B cutoff: %.2f Hz, fit %.2f%%, Kp %.10g, Ki %.10g\n', ...
        cutoffHzList(bestBIdx), fitBList(bestBIdx), kpBList(bestBIdx), kiBList(bestBIdx));
end

%% Plot raw versus filtered outputs for all cutoff candidates
colors = lines(numCutoffs);
fig = figure('Name', 'Low-pass Sweep Comparison', ...
    'NumberTitle', 'off', ...
    'Position', [120, 120, 1200, 780]);

subplot(2, 1, 1);
yyaxis left;
plot(tA, yA_raw, 'Color', [0.55, 0.55, 0.55], 'LineWidth', 1.0, ...
    'DisplayName', 'A raw output');
hold on;
for idx = 1:numCutoffs
    plot(tA, filteredA{idx}, 'Color', colors(idx, :), 'LineWidth', 1.3, ...
        'DisplayName', sprintf('A %.2f Hz', cutoffHzList(idx)));
end
ylabel('Speed (m/s)');
yyaxis right;
plot(tA, uA, '--', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.0, ...
    'DisplayName', 'A PWM input');
ylabel('PWM');
grid on;
title('Motor A: raw output and low-pass sweep');
xlabel('Time (s)');
legend('Location', 'best');
hold off;

subplot(2, 1, 2);
yyaxis left;
plot(tB, yB_raw, 'Color', [0.55, 0.55, 0.55], 'LineWidth', 1.0, ...
    'DisplayName', 'B raw output');
hold on;
for idx = 1:numCutoffs
    plot(tB, filteredB{idx}, 'Color', colors(idx, :), 'LineWidth', 1.3, ...
        'DisplayName', sprintf('B %.2f Hz', cutoffHzList(idx)));
end
ylabel('Speed (m/s)');
yyaxis right;
plot(tB, uB, '--', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.0, ...
    'DisplayName', 'B PWM input');
ylabel('PWM');
grid on;
title('Motor B: raw output and low-pass sweep');
xlabel('Time (s)');
legend('Location', 'best');
hold off;
