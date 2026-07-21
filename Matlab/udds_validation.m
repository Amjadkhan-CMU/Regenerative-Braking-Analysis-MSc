% MATLAB Validation Script — Regenerative Braking Energy Analysis
% CST7000 MSc Dissertation
% Author: Md Amjad Hossain Khan | st20341331
% Cardiff Metropolitan University | MSc Robotics and AI
% Supervisor: Paul Jenkins
% Purpose:
% This script independently replicates the Python energy calculations
% to validate the methodology used in the main analysis notebook.
% Same vehicle parameters, same UDDS dataset, same energy model.
% If results match within 5%, the Python model is confirmed valid.


clc; clear; close all;

fprintf('MATLAB Validation — Regenerative Braking\n'); %[output:5a90f767]
fprintf('Md Amjad Hossain Khan | st20341331\n'); %[output:959e4a5a]
fprintf('Cardiff Metropolitan University\n'); %[output:4f3587da]


% Vehicle Parameters
% These must match the Python notebook exactly
% Sources: Szumska (2025), He et al. (2022), Chidambaram et al. (2023)

m           = 1521;    % vehicle mass kg (Nissan Leaf 2nd gen)
eta_motor   = 0.92;    % peak motor efficiency (Szumska, 2025)
eta_inv     = 0.97;    % inverter efficiency (He et al., 2022)
eta_bat     = 0.95;    % battery charging efficiency (Chidambaram et al., 2023)
eta_full    = eta_motor * eta_inv * eta_bat;
g           = 9.81;
fric_thresh = 0.30 * g;   % friction braking threshold m/s2 (He et al., 2022)
min_v       = 2.78;        % minimum regen speed m/s = 10 km/h (Szumska and Skuza, 2025)
soc_upper   = 0.95;        % SOC upper limit (Zu et al., 2024)
soc_initial = 0.80;
soc_final   = 0.25;

fprintf('Vehicle Parameters Confirmed:\n'); %[output:78a18652]
fprintf('  Mass                : %d kg\n', m); %[output:89350e86]
fprintf('  Motor efficiency    : %.2f\n', eta_motor); %[output:7c3b1e39]
fprintf('  Inverter efficiency : %.2f\n', eta_inv); %[output:8feb3338]
fprintf('  Battery efficiency  : %.2f\n', eta_bat); %[output:6f4e10ec]
fprintf('  Combined eta        : %.4f (%.2f%%)\n', eta_full, eta_full*100); %[output:966e3652]
fprintf('  Friction threshold  : %.3f m/s2 (0.3g)\n', fric_thresh); %[output:5ef77d4f]
fprintf('  Min regen speed     : %.2f m/s (%.1f km/h)\n\n', min_v, min_v*3.6); %[output:57fa2db1]

% Load UDDS Dataset
% Source: U.S. Environmental Protection Agency (EPA)
% File format: tab-separated, 2 header rows, columns: time(s) | speed(mph)

udds_file = 'udds.txt';

if ~isfile(udds_file)
    error(['udds.txt not found in MATLAB Drive.\n' ...
           'Please upload it using the file panel on the left.\n' ...
           'Download from: https://www.epa.gov/vehicle-and-fuel-emissions-testing/dynamometer-drive-schedules']);
end

% Read file skipping 2 header rows
raw_data  = readmatrix(udds_file, 'FileType', 'text', ...
                       'NumHeaderLines', 2, 'Delimiter', '\t');
time_s    = raw_data(:,1);
speed_mph = raw_data(:,2);
speed_ms  = speed_mph * 0.44704;
speed_kmh = speed_mph * 1.60934;
n_points  = length(time_s);

fprintf('UDDS Dataset Loaded [EPA Official File]:\n'); %[output:571933ed]
fprintf('  Data points   : %d\n', n_points); %[output:5e082c16]
fprintf('  Duration      : %.0f s (%.1f min)\n', max(time_s), max(time_s)/60); %[output:40905378]
fprintf('  Max speed     : %.1f mph (%.2f m/s)\n', max(speed_mph), max(speed_ms)); %[output:4249edf4]
fprintf('  Avg speed     : %.2f mph\n', mean(speed_mph)); %[output:4decd394]
fprintf('  Stopped time  : %.1f%%n\n', sum(speed_ms < 0.5)/n_points*100); %[output:3ea64507]

% Calculate Acceleration 
dt    = diff(time_s);
dv    = diff(speed_ms);
accel = dv ./ dt;
accel = [accel; accel(end)];  % pad to match length

% SOC Profile
% Linear depletion from 80% to 25% matching Python notebook
soc_profile = linspace(soc_initial, soc_final, n_points);

% Braking Event Detection and Energy Calculation
% Replicates Python Section 5 and Section 6 exactly

E_recoverable_total = 0;
E_recovered_total   = 0;
n_events            = 0;
event_log           = [];  % stores per-event results

in_brake  = false;
start_idx = 1;

for i = 2:n_points
    a = accel(i);
    v = speed_ms(i);

    % Detect braking event start
    if ~in_brake && a < -0.10 && v > min_v
        in_brake  = true;
        start_idx = i;

    % Detect braking event end
    elseif in_brake && (a >= -0.04 || v <= min_v)
        end_idx = i;
        dur     = end_idx - start_idx;

        if dur >= 2
            seg_v  = speed_ms(start_idx:end_idx);
            seg_a  = accel(start_idx:end_idx);
            v_i    = seg_v(1);
            v_f    = seg_v(end);
            e_k    = 0.5 * m * max(0, v_i^2 - v_f^2);
            pk_dec = abs(min(seg_a));
            soc_v  = soc_profile(start_idx);

            if e_k > 0

                % Constraint factor f
                f = 1.0;
                if soc_v >= soc_upper
                    f = 0.0;
                elseif soc_v >= 0.90
                    f = f * (1.0 - ((soc_v - 0.90)/(soc_upper - 0.90)) * 0.5);
                elseif soc_v <= 0.20
                    f = 0.30;
                elseif soc_v <= 0.30
                    f = f * (0.30 + (soc_v - 0.20)/(0.30 - 0.20) * 0.70);
                end

                % Friction threshold
                if pk_dec > fric_thresh
                    regen_share = max(0, 1 - (pk_dec - fric_thresh)/fric_thresh);
                    f = f * regen_share;
                end

                % Speed constraint
                if v_i < min_v
                    f = f * 0.30;
                elseif v_i < 5.0
                    f = f * 0.60;
                elseif v_i < 8.0
                    f = f * 0.80;
                end

                % Duration constraint
                if dur <= 2
                    f = f * 0.70;
                elseif dur <= 4
                    f = f * 0.85;
                end

                % Deceleration-dependent motor efficiency
                decel_ratio = pk_dec / fric_thresh;
                if decel_ratio < 0.15
                    eta_m = eta_motor * 0.70;
                elseif decel_ratio < 0.30
                    eta_m = eta_motor * 0.82;
                elseif decel_ratio < 0.50
                    eta_m = eta_motor * 0.90;
                elseif decel_ratio < 0.80
                    eta_m = eta_motor * 0.96;
                else
                    eta_m = eta_motor * 1.00;
                end

                eta_event = eta_m * eta_inv * eta_bat * f;
                e_rec     = e_k * eta_event;

                E_recoverable_total = E_recoverable_total + e_k;
                E_recovered_total   = E_recovered_total   + e_rec;
                n_events            = n_events + 1;

                event_log(end+1, :) = [v_i*3.6, pk_dec, dur, ...
                                       e_k/3600, e_rec/3600, ...
                                       e_rec/e_k*100, soc_v, f];
            end
        end
        in_brake = false;
    end
end

% Convert to Wh
E_rec_Wh  = E_recoverable_total / 3600;
E_rcvd_Wh = E_recovered_total   / 3600;
E_gap_Wh  = E_rec_Wh - E_rcvd_Wh;
avg_rate  = mean(event_log(:,6));
gap_rate  = 100 - avg_rate;

% Print MATLAB Results 
fprintf('MATLAB UDDS Energy Recovery Results:\n'); %[output:58e76a5b]
fprintf('  Braking events detected : %d\n',     n_events); %[output:2e55f457]
fprintf('  Total recoverable (Wh)  : %.3f\n',   E_rec_Wh); %[output:33e73a7b]
fprintf('  Total recovered (Wh)    : %.3f\n',   E_rcvd_Wh); %[output:7893764d]
fprintf('  Total energy gap (Wh)   : %.3f\n',   E_gap_Wh); %[output:54ff267e]
fprintf('  Average recovery rate   : %.2f%%n', avg_rate); %[output:290c34b1]
fprintf('  Average energy gap      : %.2f%%n\n', gap_rate); %[output:56a1d535]

% Cross Validation Against Python
% Python results from Jupyter notebook (Section 16 summary)
py_events  = 45;
py_rec     = 803.804;
py_rcvd    = 593.864;
py_gap     = 209.939;
py_rate    = 62.45;

fprintf('Cross-Validation: MATLAB vs Python\n'); %[output:20fb1910]
fprintf('%-32s %10s %10s %10s %8s\n', 'Metric', 'Python', 'MATLAB', 'Diff (Wh/%)', 'Status'); %[output:63dbd9c9]
fprintf('%s\n', repmat('-', 1, 75)); %[output:2dfbe9fe]

results = {
    'Braking events',        py_events,  n_events,   '';
    'Total recoverable (Wh)',py_rec,     E_rec_Wh,   '';
    'Total recovered (Wh)',  py_rcvd,    E_rcvd_Wh,  '';
    'Total energy gap (Wh)', py_gap,     E_gap_Wh,   '';
    'Avg recovery rate (%)', py_rate,    avg_rate,    '';
};

all_pass = true;
for i = 1:size(results,1) %[output:group:9ec8eb78]
    name   = results{i,1};
    py_val = results{i,2};
    ml_val = results{i,3};
    if py_val ~= 0
        diff_pct = abs(ml_val - py_val) / py_val * 100;
    else
        diff_pct = 0;
    end
    if diff_pct <= 5.0
        status = 'PASS';
    else
        status = 'REVIEW';
        all_pass = false;
    end
    fprintf('  %-30s %10.3f %10.3f %9.2f%%  [%s]\n', ... %[output:3e39d4c6]
            name, py_val, ml_val, diff_pct, status); %[output:3e39d4c6]
end %[output:group:9ec8eb78]

fprintf('\n');
if all_pass %[output:group:008b9d4e]
    fprintf('VALIDATION RESULT: PASS\n');
    fprintf('All metrics within 5%% tolerance.\n');
    fprintf('Python energy model independently validated by MATLAB.\n');
else
    fprintf('VALIDATION RESULT: REVIEW NEEDED\n'); %[output:6df71af8]
    fprintf('One or more metrics exceed 5%% tolerance.\n'); %[output:3eaa1cb2]
    fprintf('Check parameter consistency between Python and MATLAB.\n'); %[output:86853d32]
end %[output:group:008b9d4e]

% Publication Figure 
figure('Color', 'white', 'Position', [50 50 1200 500]); %[output:058d1539]

% Left: Speed profile with braking events highlighted
subplot(1, 2, 1); %[output:058d1539]
plot(time_s, speed_kmh, 'Color', [0.08 0.40 0.75], 'LineWidth', 0.9); %[output:058d1539]
hold on; %[output:058d1539]
braking_mask = accel < -0.10 & speed_ms > min_v;
for i = 1:n_points-1
    if braking_mask(i)
        patch([time_s(i) time_s(i+1) time_s(i+1) time_s(i)], ... %[output:058d1539]
              [0 0 speed_kmh(i+1) speed_kmh(i)], ... %[output:058d1539]
              [0.78 0.15 0.15], 'EdgeColor', 'none', 'FaceAlpha', 0.45); %[output:058d1539]
    end
end
xlabel('Time (s)'); %[output:058d1539]
ylabel('Speed (km/h)'); %[output:058d1539]
title(sprintf('Figure A.1: UDDS Speed Profile — MATLAB Validation\n%d Braking Events Detected', n_events), ... %[output:058d1539]
      'FontWeight', 'bold'); %[output:058d1539]
legend({'Speed profile', 'Braking events'}, 'Location', 'northeast', 'FontSize', 9); %[output:058d1539]
grid on; box off; %[output:058d1539]
set(gca, 'FontName', 'Arial', 'FontSize', 10); %[output:058d1539]

% Right: Recovery rate distribution histogram
subplot(1, 2, 2); %[output:058d1539]
histogram(event_log(:,6), 10, ... %[output:058d1539]
          'FaceColor', [0.08 0.40 0.75], ... %[output:058d1539]
          'EdgeColor', 'white', 'LineWidth', 1.2); %[output:058d1539]
xlabel('Recovery Rate (%)'); %[output:058d1539]
ylabel('Number of Braking Events'); %[output:058d1539]
title(sprintf('Figure A.2: Recovery Rate Distribution\nMean = %.1f%%  |  Std = %.1f%%  |  n = %d events', ... %[output:058d1539]
              mean(event_log(:,6)), std(event_log(:,6)), n_events), ... %[output:058d1539]
      'FontWeight', 'bold'); %[output:058d1539]
grid on; box off; %[output:058d1539]
set(gca, 'FontName', 'Arial', 'FontSize', 10); %[output:058d1539]

% Save figure at 300 DPI
set(gcf, 'Color', 'white');
set(gca, 'Color', 'white'); %[output:058d1539]
print('MATLAB_Validation_Figure', '-dpng', '-r300'); %[output:058d1539]

subplot(1,2,2) %[output:058d1539]
xlim([1 67]) %[output:058d1539]
ylim([-3.6 14.6]) %[output:058d1539]
fprintf('\nFigure saved: MATLAB_Validation_Figure.png\n'); %[output:90b9d771]
fprintf('Script complete.\n'); %[output:6eee28ad]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":9.3}
%---
%[output:5a90f767]
%   data: {"dataType":"text","outputData":{"text":"MATLAB Validation — Regenerative Braking\n","truncated":false}}
%---
%[output:959e4a5a]
%   data: {"dataType":"text","outputData":{"text":"Md Amjad Hossain Khan | st20341331\n","truncated":false}}
%---
%[output:4f3587da]
%   data: {"dataType":"text","outputData":{"text":"Cardiff Metropolitan University\n","truncated":false}}
%---
%[output:78a18652]
%   data: {"dataType":"text","outputData":{"text":"Vehicle Parameters Confirmed:\n","truncated":false}}
%---
%[output:89350e86]
%   data: {"dataType":"text","outputData":{"text":"  Mass                : 1521 kg\n","truncated":false}}
%---
%[output:7c3b1e39]
%   data: {"dataType":"text","outputData":{"text":"  Motor efficiency    : 0.92\n","truncated":false}}
%---
%[output:8feb3338]
%   data: {"dataType":"text","outputData":{"text":"  Inverter efficiency : 0.97\n","truncated":false}}
%---
%[output:6f4e10ec]
%   data: {"dataType":"text","outputData":{"text":"  Battery efficiency  : 0.95\n","truncated":false}}
%---
%[output:966e3652]
%   data: {"dataType":"text","outputData":{"text":"  Combined eta        : 0.8478 (84.78%)\n","truncated":false}}
%---
%[output:5ef77d4f]
%   data: {"dataType":"text","outputData":{"text":"  Friction threshold  : 2.943 m\/s2 (0.3g)\n","truncated":false}}
%---
%[output:57fa2db1]
%   data: {"dataType":"text","outputData":{"text":"  Min regen speed     : 2.78 m\/s (10.0 km\/h)\n\n","truncated":false}}
%---
%[output:571933ed]
%   data: {"dataType":"text","outputData":{"text":"UDDS Dataset Loaded [EPA Official File]:\n","truncated":false}}
%---
%[output:5e082c16]
%   data: {"dataType":"text","outputData":{"text":"  Data points   : 1370\n","truncated":false}}
%---
%[output:40905378]
%   data: {"dataType":"text","outputData":{"text":"  Duration      : 1369 s (22.8 min)\n","truncated":false}}
%---
%[output:4249edf4]
%   data: {"dataType":"text","outputData":{"text":"  Max speed     : 56.7 mph (25.35 m\/s)\n","truncated":false}}
%---
%[output:4decd394]
%   data: {"dataType":"text","outputData":{"text":"  Avg speed     : 19.58 mph\n","truncated":false}}
%---
%[output:3ea64507]
%   data: {"dataType":"text","outputData":{"text":"  Stopped time  : 20.3%n\n","truncated":false}}
%---
%[output:58e76a5b]
%   data: {"dataType":"text","outputData":{"text":"MATLAB UDDS Energy Recovery Results:\n","truncated":false}}
%---
%[output:2e55f457]
%   data: {"dataType":"text","outputData":{"text":"  Braking events detected : 57\n","truncated":false}}
%---
%[output:33e73a7b]
%   data: {"dataType":"text","outputData":{"text":"  Total recoverable (Wh)  : 841.132\n","truncated":false}}
%---
%[output:7893764d]
%   data: {"dataType":"text","outputData":{"text":"  Total recovered (Wh)    : 629.468\n","truncated":false}}
%---
%[output:54ff267e]
%   data: {"dataType":"text","outputData":{"text":"  Total energy gap (Wh)   : 211.663\n","truncated":false}}
%---
%[output:290c34b1]
%   data: {"dataType":"text","outputData":{"text":"  Average recovery rate   : 60.21%n","truncated":false}}
%---
%[output:56a1d535]
%   data: {"dataType":"text","outputData":{"text":"  Average energy gap      : 39.79%n\n","truncated":false}}
%---
%[output:20fb1910]
%   data: {"dataType":"text","outputData":{"text":"Cross-Validation: MATLAB vs Python\n","truncated":false}}
%---
%[output:63dbd9c9]
%   data: {"dataType":"text","outputData":{"text":"Metric                               Python     MATLAB Diff (Wh\/%)   Status\n","truncated":false}}
%---
%[output:2dfbe9fe]
%   data: {"dataType":"text","outputData":{"text":"---------------------------------------------------------------------------\n","truncated":false}}
%---
%[output:3e39d4c6]
%   data: {"dataType":"text","outputData":{"text":"  Braking events                     45.000     57.000     26.67%  [REVIEW]\n  Total recoverable (Wh)            803.804    841.132      4.64%  [PASS]\n  Total recovered (Wh)              593.864    629.468      6.00%  [REVIEW]\n  Total energy gap (Wh)             209.939    211.663      0.82%  [PASS]\n  Avg recovery rate (%)              62.450     60.209      3.59%  [PASS]\n","truncated":false}}
%---
%[output:6df71af8]
%   data: {"dataType":"text","outputData":{"text":"VALIDATION RESULT: REVIEW NEEDED\n","truncated":false}}
%---
%[output:3eaa1cb2]
%   data: {"dataType":"text","outputData":{"text":"One or more metrics exceed 5% tolerance.\n","truncated":false}}
%---
%[output:86853d32]
%   data: {"dataType":"text","outputData":{"text":"Check parameter consistency between Python and MATLAB.\n","truncated":false}}
%---
%[output:058d1539]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAC0AAAAtCAYAAAA6GuKaAAAGLUlEQVR4AeyZy28URxDGv571eu1dv2OMLT8wBttygGCiKJESuEREiiJFCuJEOBEpJwcukSJFuXDgQsRfEHHPBSFxyjUcQBwinBviJRDGMhjj92u99k761zCb3cku9i67BEuMpqanq6urv6mp7uqHd\/v2bf\/u3bv+wsKCH1yTk5M+\/IAoh8jfuXPHn5mZcaKrq6v+vXv3nOytW7dcisxmhI5AZmxszOkKHk+fPs3oQTdtQEH71PNkr3Q6Lci+uruurk6e54pc3irMlEciESUSCcf\/vx4OmTFGGxsbGQy1tbUCXPAhgA4K4\/G4otGoy8ZiMSfnMm\/w4UADamVlJadZLA1lM8mHrVxdXe1EKONjBwYGBO3Zs0dBGQJNTU2OT1k2n7JiyQusubi4qKmpKfEB1meVSqX+o6uqqkr19fU5fPIAhml9T8+ePXN\/LVsH5Q0NDYiUhbxsLdPT07IO7xoGfHYZ72ErwwM0gJCHAHv\/\/n2hi7wxRi0tLaqpqUG8ZDLGZOp6+C45Y14wjTGugba2Nhnzgkc51qKD8h4mZHfu3Cn+BEApJ8UN4AMaXrnI6+\/vd76Wnfb09KixsVEBDz\/cu3ev8NlCDeOzfX19GhwcdPpIe3t7xV8I19m1a5eTQW9XV1dOMQaAD9Ev6OwQ7\/Agb21tLafSdsh4ExMTWl9f3w5YMxi9ZDIpOk+Gsw1e3OjBcJcdXN523G70YKzeTr7tQGPlbQXaGOPGY6z9trtFgM\/5dJAJp4Ty58+f5w3pYdmt5suhc0ugjx07JgLH8PCwTpw4oYMHD7p8Pt7ly5dfib8soAm3UBDO87WYjDQoNnRK87FuLS0tyS4YNDc352h+fj6HxxCaT0c5eR6dEMDBHDmv8kSbtGO\/FoZ\/FB+ATEdHh86fP68rV67owoULLiXPfIPySpID7dlVyqsaMzv25WBgPnHu3DkdPnxYjx8\/1o0bNzQ+Pq4jR444t6l0hHU+zewNa+cgy5OJr684S588edJNjK5fv64zZ87o0qVLOn36tK5duyZWNpWOsB4Wbm5uzgMxixW37vEyu9Iy7Driyuqq\/vjrkQPJPBu6evWqWwBUOsJ6dMKXePImo2NJjU43Zsoaug+oeUenknaO9aj5qMxHPziXGBoa0lqix618GPMLBquMptJfPIYglkjLy8sKE745Mb+hzzrSGuxtVGNLnZrjxi5mPc0ko5qaXNR4Vb9+m\/pCf0c+1Z\/+Uc0lI87aFQUtezGE0aHC9OTJE321L65fv+1y9MuXTeprjSoW9eTLqDq1otbkjKOpxAGtp335byDCeiMjI9oKXbx4UYe6Y2qt3rCfWfieX6z8osK7efOmtkKjo6MOaTSdsqkvz0\/b9N+bkQWC4\/s8K0desaoXlte1lkrLU1pVnsmp3p6atU7jK21RbzaEss+ynKcfhXn0uZxGbKZo0EurG1q3HlJb7emDnhoNtEWtmhd3+3sx1casSuOpUISFzwKZiVi4D+XLM01Q6LIthDibZN\/f36s131NjIqqz332o389+rlPHD2nkmyH9\/P3Hak5U2Y4aydldUtYF6Pb2dnXZVfhWiOibVd29Fg36+Cft6u9qEZaOR1IiAv70da+gLruJVGNHls0iLMCptxVC1iHNehQNmrpsvrAXwYzvwYMHYmgkxR\/hbxphUfIaVBJodpKY5eGbBCD8jpRQDp\/y18C0adWSQKOVOUt3d3dmF4rdqM7OzoK+TJ1yUcmgywWgFD3vQJditVLqvLN02GqMKg8fPpQ9kZI90XIpQyNDZViWqSxLNjb1kSUlDz8sW7SlmcaGleTLcxJgj9fE6pxTgCZ75kIKCMDMzs5mqjFcsnvLxzAqEQUZNsnDpzwjbF+KBo0iGrZ1C95McthiQIBNcjbRg7S1tdWtblhHshOADB\/IxzHOI0uY3717tzv6g48scgEVDRpAm1kbGQCxyidUB42REuKxIuUcLJFiCMrq8xxCwQ+vOYsGbYxxvxxlhQigHHdw5EBYLyQHn7\/GmpIPwTXgBURdpriUIxfwSwKNJQMFxabMowFhjHEHS+jC2vwVfD5bXwCa8tcCna202Hc6FB2QHQD+BqAAZIyRMSavOmOMK+NDA4F\/AAAA\/\/\/gUo\/NAAAABklEQVQDAMKRpHgCQh\/HAAAAAElFTkSuQmCC","height":30,"width":30}}
%---
%[output:90b9d771]
%   data: {"dataType":"text","outputData":{"text":"\nFigure saved: MATLAB_Validation_Figure.png\n","truncated":false}}
%---
%[output:6eee28ad]
%   data: {"dataType":"text","outputData":{"text":"Script complete.\n","truncated":false}}
%---
