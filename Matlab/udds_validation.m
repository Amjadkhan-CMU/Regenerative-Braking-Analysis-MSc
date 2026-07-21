% MATLAB Validation Script — Regenerative Braking Energy Analysis
% CST7000 MSc Dissertation
% Author: Md Amjad Hossain Khan | st20341331
% Cardiff Metropolitan University | MSc Robotics and AI
% Supervisor: Dr Paul Jenkins
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
    fprintf('VALIDATION RESULT: REVIEW NEEDED\n'); %[output:3eaa1cb2]
    fprintf('One or more metrics exceed 5%% tolerance.\n'); %[output:86853d32]
    fprintf('Check parameter consistency between Python and MATLAB.\n'); %[output:66681814]
end %[output:group:008b9d4e]

% Figure
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
xlabel('Time (s)', 'Color', 'k', 'FontSize', 11); %[output:058d1539]
ylabel('Speed (km/h)', 'Color', 'k', 'FontSize', 11); %[output:058d1539]
title(sprintf('Figure A.1: UDDS Speed Profile — MATLAB Validation\n%d Braking Events Detected', n_events), ... %[output:058d1539]
      'FontWeight', 'bold', 'Color', 'k', 'FontSize', 12); %[output:058d1539]
legend({'Speed profile', 'Braking events'}, 'Location', 'northeast', 'FontSize', 9, 'TextColor', 'k'); %[output:058d1539]
grid on; box off; %[output:058d1539]
set(gca, 'FontName', 'Arial', 'FontSize', 10, ... %[output:058d1539]
         'Color', 'white', 'XColor', 'k', 'YColor', 'k', ... %[output:058d1539]
         'GridColor', [0.85 0.85 0.85]); %[output:058d1539]

% Right: Recovery rate distribution histogram
subplot(1, 2, 2); %[output:058d1539]
histogram(event_log(:,6), 10, ... %[output:058d1539]
          'FaceColor', [0.08 0.40 0.75], ... %[output:058d1539]
          'EdgeColor', 'white', 'LineWidth', 1.2); %[output:058d1539]
xlabel('Recovery Rate (%)', 'Color', 'k', 'FontSize', 11); %[output:058d1539]
ylabel('Number of Braking Events', 'Color', 'k', 'FontSize', 11); %[output:058d1539]
title(sprintf('Figure A.2: Recovery Rate Distribution\nMean = %.1f%%  |  Std = %.1f%%  |  n = %d events', ... %[output:058d1539]
              mean(event_log(:,6)), std(event_log(:,6)), n_events), ... %[output:058d1539]
      'FontWeight', 'bold', 'Color', 'k', 'FontSize', 12); %[output:058d1539]
grid on; box off; %[output:058d1539]
set(gca, 'FontName', 'Arial', 'FontSize', 10, ... %[output:058d1539]
         'Color', 'white', 'XColor', 'k', 'YColor', 'k', ... %[output:058d1539]
         'GridColor', [0.85 0.85 0.85]); %[output:058d1539]

% Save figure at 300 DPI
set(gcf, 'Color', 'white', 'InvertHardcopy', 'off');
print('MATLAB_Validation_Figure', '-dpng', '-r300'); %[output:058d1539]

fprintf('\nFigure saved: MATLAB_Validation_Figure.png\n'); %[output:90b9d771]
fprintf('Script complete.\n'); %[output:6eee28ad]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":63.7}
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
%[output:3eaa1cb2]
%   data: {"dataType":"text","outputData":{"text":"VALIDATION RESULT: REVIEW NEEDED\n","truncated":false}}
%---
%[output:86853d32]
%   data: {"dataType":"text","outputData":{"text":"One or more metrics exceed 5% tolerance.\n","truncated":false}}
%---
%[output:66681814]
%   data: {"dataType":"text","outputData":{"text":"Check parameter consistency between Python and MATLAB.\n","truncated":false}}
%---
%[output:058d1539]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAcsAAADACAYAAABrskgwAAAQAElEQVR4AezdCdzt1dQ48L3uv0j0upSUkiuRDEmmCJWx0IAiUuoVIjQQylReQ0oylMykSYrweiNThQpRqZTScCuaI0qpqP\/z\/Z27n7uf3z3nPOec5zzj\/d3PXc\/ev73Xntbee6211x7OvHuafw0FGgrMKAp897vfvWfevHn3POIRj7jnhz\/84T133nnnPTvssMM9a6yxxj1XX331PXffffc93\/jGNyqcXXfdtfqeUQ1oKtNQYA5SYF5q\/jUUaCgwYyjwr3\/9K33+859PD37wg9OIoEwvfOEL07LLLjumfhGRtthii\/T85z8\/\/frXv04333zzmPjmo6FAQ4HhU6ARlsOn6fBzbHJcaijwt7\/9LZ1\/\/vmVIFxrrbU6tvs+97lPJVCvv\/76RMB2RGwiGgo0FBgKBeYNJZcmk4YCDQUaCjQUaCgwhynQCMs53LlN06aUAkMp7L\/+67\/SIx\/5yPTLX\/4yLVy4sGOeN910UzrvvPMqXGk6IjYRDQUaCgyFAo2wHAoZm0waCgyHAve9733T1ltvnS677LK0zTbbpD\/96U9LZHz77bend73rXenss89OW221VZJmCaQmoKFAQ4GhUmBUWF5zzTXpYQ97WIqItvD\/\/t\/\/S+uuu276+te\/nkzWei3+93\/\/t226iFZ+97vf\/dJLXvKS6kDCPffcU08++n3BBRekbbfdNj3wgQ8cza\/XtKOZFB5l\/fznP0+bbrppkk9Eqz7y32mnndJFF11UYM9u72tf+9pRmumP8VrzsY99bBQ\/okWXiMUuej3rWc9K6IeO4+WX48cbSxGLy1DnnI4\/ohV3\/\/vff7RuD3rQg9LJJ5+cynh1z+n4I1rp4OTw2ebmdrzpTW+qqv7b3\/42PepRj6rocMQRRyR0ff3rX59WXHHF9OUvfzk997nPTf\/+97+r+IhW+yMiyce4Nr6N84hW3IIFC9KWW245Bv9tb3tbVVb9DzpGtNJFDObOmzev4hmf\/exn2\/KMepn5+3vf+96YOr74xS\/uui+rvRHd64gO6HHppZfmYjq6f\/nLXxJaRbTydOCqEzJeiLdEtHDVvRNuu3B9mvkuped3v\/vdKFrZB9o4GtHFI718IiKZuyCiVbeI9q698Q984APpH\/\/4xxI5\/+c\/\/0lf+cpX0k9+8pMxcd3q1inNmAwm8FHSLGLJNo0nq0oaob38eqnOqLAcD\/nuu++uzD6I9OxnPzv9+c9\/Hi\/JmPh\/\/vOf6f\/+7\/\/SBhtskPbaa6901113jYnHjA855JD0+Mc\/Ph177LHJQYeMUKY98MADE9wc181VhrI22mijdNJJJyX5ZHz5f+1rX0uPecxj0re+9a0c3LgFBdCLOXCTTTZJ3\/72t4uYyfeWE9fYW2WVVSa\/0Gks4e9\/\/3s64YQTOtbAmDeezaE77rgjvfWtb63G7ZlnnrlEGgeEjGvj2zjPCFdccUWqM\/Mf\/\/jH6brrrssoQ3XVmal41113TZtvvnnSxvEKIPyPOeaYMWinn356uuSSS8aE9fuBDujx5Cc\/Of3mN7\/pmvwhD3lIMuYzEpp1OkR16YjwdSIZ7qMf\/ej0tKc9jXdWgTZ88IMfTGhjjOTKL1y4sBK4r3vd63pWdgZJk8sblotfGHeDyqpO9ZjXKaJbOI33v\/\/7v9Ott97aDa1j3EEHHZQ+\/elPj4n\/wx\/+kN73vvclDR0TUfvYe++9l9Byaiijnz\/96U\/TJz7xidHvdh7l7bLLLkn57eKbsFT1yXTQ6N3vfne1crr22murFdZc7gsWFWbVbm181ateVSmslBjzh\/A855xzlkhCMTSurey+8IUvpDvvvLMSVC94wQuWwP3jH\/9YWXuWiBhygLn4gx\/8YNxcr7rqqvSLX\/xiDJ6rMZSEMYEDfsjLarqb4I6I9NKXvnS0hF\/96lfp8ssvH\/0uPeolT2Gu8rjywz8bgcn\/q1\/96mjVrTbPOOOM0e9ePIOkaZPv0IImKqvKiswrP7J\/jTXWSFdffXW1gqMd0vbOPffc6m5XxqGRfv\/738+fY9wddthhNK30TBVWb494xCNG8Q4++OAxA\/BHP\/pRyoNuzTXXTDRm5Zr0v\/\/979N6661XpfXNLCCuCujy5\/jjj6\/qAYU5wmCQXlrlPfShDxWVbrzxxnTcccdV\/qX1z\/7771\/RSn8BfXb44Yen5ZdfviIJGlllVh99\/KmPJXmXoIycHb+xk7+f8YxnJCYV9wy54nNae3YZby64VlOEWkSkpz\/96W2bhMEzmS233HJVPEF38cUXVybLKmDRH2OcKQ79bGmgn0NA8+fPX4SRUrlSV7Y5MRo54ilpjeb4gb4ciUpcgp2bv8XfcsstiRVHGLBlY9+VH9gakBd\/JzAvmUHFq\/u97nUv3mrVrf3VR5c\/9XGsPHSlNFAeJKWYWAHxdwIrRCtF8X\/961\/TWWedxTsGLBayAqCelJkxCBP8KPtgouPdPCY40KMEPJdSmqtKqdGP+buTO8y6dSqjl3Bj0NjLbTKOx5NVT3rSkyorozRW0quuumovRaW2wrKeEqNiHv3GN75R7TvmePsoncwTGYe73MjkftnLXpa++93vppVWWklQMiFMjOpj5A8tecSp\/lv2MwkoNyKqfY+PfvSjo0yBSaaXDi3z3H333RNGExEVA6YFvvOd70z5H9MVQvsubdobb7xxuvDCC9MrX\/nKKl1EpMc97nHpZz\/7WSVc4JdAyDAV0zAjokrzzGc+s+O+H\/s+ZrVgZD8pomV\/l\/83v\/nNJC61+WeyM4VHtPD5hbVBHThInxFc6JYzMbH42fjZ+iMioQ+z+corr1z1j8FLuYEHtGGfffZJD3jAA6p4fdqOHhhpRFR74tIBF+8jWntw6GpMRLTabD+V2Z7myyIBH7TrQybJXN+IqEzvj33sY6v6ECr6W1rAdBMR1aGZ73znOwltIyJhhh\/\/+MehVIDeBIP2RER15xFDK83HFWIPf5hBKZ9QnQsgAPkzbLbZZpUXw8YYqo+RP6eddlq18l5nnXWqPf6RoOq\/rQeM3ApthRVWqMKUUTc\/ZoHp8QP9FtFqd0mPKvGAfwjn7bfffjS1Otx2222j3zx1Olr1CTd\/tOOJT3yiz8p0SoG2qja\/Vltttar\/Ilrjo0Lq8IeSTLHHHKHgHbvttltStm8g7i1vectonvrDnrA4wETOqhXRGn\/GHlOf8Sf+CU94QrW9lMeD8VnuuwvXt\/Y\/81g0d5h485aWMV6WQWmKaJWX9yyVd+9737uqp8UHGkdENb\/QpRs\/Rjsrv5LX2Mu1qJCndhhfVtLmBWVHGMhzMSIqniZdxOK6GTPjpTHvTznllHT00Uensg7agd9ERDL\/0MPevDkXsbht6JPG+YfO48mqXNeISPoCP8vZ5rLL9qmHfel5GakX10Vojci4mKIVR\/4ez8W0HPLJeOXBERM+h++7775Jp9L+DGLhKktj9m3SYL7Cu4HycvxOO+2UDj300DH7MyaH\/IABs8wyy6SMn12My6AmvJQvnMmWsK3v4zEXenGFEHZZHK40mBrGyiSsLOHA4N1xxx3Tq1\/96kTDEQbkTzirXynwxSnTKhsj9A34n\/KUp6Ss5QqbKjCxXvOa16QbbrihKtKEsedTfYz8MfnsFWVBW9IDM6y3byTJEv8xAMqWQZ4j0c6+HcWKQM7h7VzKzpVXXjka5btklKMRhcfE1C60FUwIU2L0n\/aYkMav9ojX3wcccECymiv7Utx4YJXmQA4842qTkT1i\/gwOV5iwxpfVpHA0UT4\/pp4Fn+92oIyyXquvvnp6znOeU6HqG\/lVH0P8gwkddthhozmaj8aHgE50tAoUj\/6ETTaHwj\/yyCPTxiPKq\/llXMED9iO5oBwL0hBq+opQ8g0HnHrqqdX5CPHCI6JaCESE6ERRoqREtL6NAyZXkQSUejB35\/patZnneTwYn6XiJJxwtZ2Rw80dc1ycfCMiKTPzISZp4SWUZToxnVfb+hBdtttuu7b7i9povtmbLMeBsi0+7IMrx2qaoOfvBNKUNJd3J9wcLg36GNvqWNZBO3L56ExB+tKXvjR6riW3bc8996yUw5xnN3cQWaVO5oSyy\/bhUejel7BUOZI4a6sypC0K7wUiohqgGdfq0qDyrZIYBb\/K6dSHP\/zhicbjUJDTf\/2UJR+dovP5DVADE1MxGAhfptc82OC0AxOe1k1oYpgIyZyj82mneS9D3H777Te63\/L+978\/0aINeCZgeVthmSD84HOf+1zCAPgJTHUB\/MLEM1\/zA+1\/z3veMzqIchkUlq222mpUYMGdKFiZ0Cw\/+clPjmaFwY5+LPIY5Mwa2olxYChOr6IHlG4Tqdy7dgDESlaaDJnBWWVY\/eRw7kc+8pGkLwgZfSGsDsZRDnvFK15R7duhL38OL5lrDuOqN6CAaIuJROGxR7jHHntU+7i2CyiM4tXFuKDoMGsJk894AC+btAhESoH0xn1Oqx1o65sARzfC3kozIqrDKGguvh0ow4lU7cnxrCzGWURLGBirOa5flyJCQcIXCKGc\/o1vfOOoAqdtpUm2HR2N55zWfH3ve9+bzFPCSbi5QCjxw1Vnc8LKRBgwPyKiWnnpDwpVOT5svbCQ6Tvh5jCrhrRMr8YyP8aOpmuvvbbPRBliffJBOTEmjXXfEVFZmeRJ0RcGIoJTWaUoWBGt78zzzB2rTatkiPp8\/fXXT7kO5rXwDMZufR4oM4\/BE088sRrjZT\/ntJSPumUhx5WuxYwVrTpS\/HIcC4ExaCyxKOZwrn5QZ2myciM8Iqo5Kg2BiJ8LBwSi9gB9LAzgp\/iMeSSdlXhEi24UGGMNXi\/Qr6wyr1ghmKxZIpQP+IX1LSyZ2\/KqDpEMol4qnnFcuM5+DBaBfRNgBrF9Ct8ZMAqnzXbeeedE0HFzmozTyUV0zC4Lq4wnPU0B07Tc\/vCHPzwqgDJOdk1yK0L1tsRnJpBOPGGfTcmIbPUp3KSjBdFu1AFjpy0abEcddVQ1sUwEe6\/wTZYPfehDyb4S+J\/\/+Z+qreJMMgOd3\/WJvLKwgsVUleEqgfTygTcIyCsiKiYTEQnj01Z9LD\/MlUmSvw5Mtdppb8zkFY\/G3PGAkMHwOuER2sbF2PiUMEKHzPI1i3o8bTQLQkwIc0VbwGyrX6UxGTAt\/jowPT3vec+rzE6YLFo7ACFvuMYNZpfHRWYUGFruJ3jdwBygTcPBQKy++LWPC4wd5fBTUN\/xjndUDNVK0+qLMieuE1jR5HEKJyIqcxezdhYGGKxTp\/oN44M3LEBzgo4ZLudZpyPGZk6KR2cuOuof88m3scBVb2MOHn6UlWxx3QBNWcYIBFamiKiUHoqEOYavuaYiD7SlIBOMvgEacd2DpaQR+L5zuPHAIuYb36L4iNcu84NC4Vs8F6iP8cMfEdX1tiw86uOSgsRCABdERGUtMzbkYVVK+IsbBLSf0prTmvPZT9Exv5VjVU145DhKR\/YTitmPT5qj0uAn6J\/jKENobmyYwxFRRUVEsuVmh3bgMQAAEABJREFUHkin3DweCVaPcVSIPfwxNrQJKj7WTVYZW\/Y64RpzmfbqYEyYF30LS5mVIOPyux+\/tCWhCS77dwYqJmqg6qAyTytMAhWhy\/BOfrZx2i6NxZ7TU5\/61IrRZnzl02CZKMpBnOMNmNJEjHGZbDkeg+FnynAQgB8DwsT4Ac03TxSCH7O1Ulm4cKHoai+VFlt9jPzRyfIY8SZao9UtP6bHBcwZmAU\/wMw32GAD3qGDiWGFqYx65uIoEmW4gderWQ\/TKdtV5sNvgMPhVxY3Q0QkAzl\/ly6a6Vth6JT3yn2jdc4LDsVFeB2Yu\/V3DteuPKGkL9sNjykcrv7Vb\/zjASFG6YKHSeZxE9FiHsLNk2y+NEccVsnjyTwwoeG1A2PSwZM8+eGgmVWy1VEWNBQ5c0\/8sACjougwmZozEa02taOjrZW88nG3VB3QkdKRlRBhwNyQNz8wx7njgb6mNLlrqow8r5Wd5yJhoW\/lZSVDAEW06i2MQkr5phjrF2FAmvp4sI8pTjtYuXI\/CwPSlKstYYAwzqtp3xkIXf2Uv83HXEYOo9wZi\/l7PDdicdsItE5jyXy232jlTlgSPvW87QWbdzmc8hfRyh+\/K+MI1Tzv9WVEC49AJUBzHvh\/HvvGOtmQ4\/p1y\/6qp1VG7gs0fvnLX54c\/LHQodDC71tYYl60W4l1dp60vnsBK7CMp3I6KH9n1yS2F2WvxiQ3yJg4EQ4O8wczCX8vEBHVhvLb3\/726pg8YhBu9g9yeqYFEzN\/ZxdjXW651unDHFZ2prrROmh9OZ75MiJGV2lolM0H6GfgYSB5wBHm6BDRSsMvTH40sExvm8\/CQDkxfZsgOpx\/WGCwWMkTVhhsu3wNdP1VxqFHXtWZfA4lUERKyCZXYe55lelLv7ajgTDWB24JFIs8LsrwvIcqDLNilYho0ZdfmDjl5\/x9l4DGDlZEtNKV\/aLvrHAiWnERkazOc3rm0ojFcRFj\/fI1+R0cyWmM8Yio7rvJP4dz84Ql\/Gjixo9w7Qf633cJNHGMV\/+V4dmsFBHpM5\/5zGiUQy3dVvmjiDWPla36oKn9wRyt76zgMNEcxvVNeeXXTnQktPWFsFLRQMdynopnSeG2g3walrkWf4LDtGnLhXDx7cQnupvbvtXTPOa3oiEM+VlyKC9Whb6Bg2HGNFr5zmMvtyNicT8TzHBAO7qaO7kOcDJQrvNqOofhGVbavgkULrrX5wTFMFtN4GRAC3vq8s5h3Exz\/m5glUhBJzw6zVdKkLmf86HYZb\/xgUb5W7n6PCKqQ3QUGXFc44N\/ooDX6lv5aD8+zN8OIiI5XFYqKXgI5QCdq0VWu4TdwmSAKHAIDR3O3ytkDQ4+bZ8gMjnkFdESanngwomIpJEGntWfMIQuD3sIqwPza0Rr4NKEcp3hGewGDUZBgxCGCRGg\/P0AE5\/8ek1DiIBe8bXVAKrjY7T1sIl8ZyajvAwG+Be\/+MVUMot6GdoO6uGT8d0P3fopX3vb4WdG2C5uGGGYDktDL3mVihLBmOvGZGV+WPGU+RCUxjaGX4Z381MWe61Pu3yMg5JBwmFiZmoc5njNKxL59wKECysVWmX8vIrN39klbPJKFk\/Ar0oex7xKEGcFBP\/Kabu5zktguiUOeqlbGcZf1sE3wPizCTanIUC7rZaky2DuUOaMOWEUCAKEPwOc7OdaDXIzsEaw+KnHsBXzXAa3Hb8T3i\/0K6ushI0tC4Q8v3KZrA99rSwNlLznJhMmAKsP\/l7AwLMqzLiEmIlvometTwe1m+AYGgaQ047nMs3kiWtlo4PrabSnFKL1eN80+lJbElauRLTfhLEKEQesmtS3HWB6hI\/VjUkB3yCkTbXDp41l8x7zE3yATtwMJk29njluql0TCSNQrglY16pN8mz6jIhEcYHbDjAqipS4dgwXE9GP4kugiOVv\/YNJZPqaRFYQ4jGM7PddB9dBcjr9nvfdpMM0c1y\/rnydwLYaq5fZ6Vs7xLG2aLOVrhO6wkpAc4qlFVQZ3oufKdZY6gW3VxwrWYeYMr7VUBYc6Gi1kuN6cZnCx6Mba4exIz9zzgojzzdhBKGxww8PD+IHHm\/I35TFkkewjDl4p7+t4PIevXY4nWwuyyMiEhM0PKC\/9Yu4XkAd5JlxtZlFLCJSXv0wa9bnvDkCL6fLrvFCCEW06mXs2n5KKWOk6rk7dRWizZQnfmA8McOy+JlbGU9cBv1q7udvNM7+kt8Ji4hkdS4fdcltJQdsf8CZCGjvILIKb9bn6MhayALE8qEuPQlLDbLKsHnswImEgImkHIDC2oGKO7lHY2OrhkPI2KPhR\/zs9+0UHVMmpuqbkLTyKY+hOxAgrhMw8eYDKQaPPQM2f3WRBgNnlhXm28BXD\/4SmItKQYuRmPwZJwsy+x9ZONM61Tnj6LSI1irXFRb0VD8Ax4oWA+cHLv\/TbCIiUSgMXOElYzQxdahwQBHJ+6e+pxNMGvTMdTDJtDl\/a2+uKwWgNN1lnOwyMcLxTXHgZpBnqXzlcC5mnDVwfa6\/hQOnBk0EfmOsnq\/wdkA40T7FSYNx8gPM2FiIiOqO5niWD+Mjm9UwCgxDe8wzjEWeAB0jgrc6jFJ5Fv3BWDJTXxRUOU4SWgFUH4v+lGUopwRMK6JVhj3Usl2Lkg\/kEEI5oTuAxqjvOh3L+WSuZOZU1tkd7YhWHR3yUGd5AUoMFzhkg8eYy5R5Yfa55Ivh+wbyzsJAv1GuhQN0zaZYK2P5CQcEbx5zTo0C4cYDuuMXvik2TvRHtMYDPlOOQTjdQB2M\/YxD4ed32CXPB2Mln9AVp0+Vg0f5LkGcb\/zaOYyISG94wxsSviUc4DOZ11HefQsH+DUX6MeS9wgD+jXzQN\/yyuXidSwgwgElI897B6XQTzga1RUA4b2C8tBlUFmVy6Hso43rOKwQ3LbC0v6ahkVEte+G6ZiUpbBi1rAZnjMvXYIuopU2IqoL3ToZUTLe7rvvPmZFYR8mdwjCOYmpYyNampRrFzQj6e0buGrC3wkMVidSMRs4VogEsu+ISBhS2R6aXDuNBvG9bUmzoxnSkG3uy9Oge9GLXsRb\/VSSPHwwZTlMpNNNLuZe4cp2OiwiqscZnPQSzuxMy8METHxPmeUBzyRA+MBzgEfb+U0KCoRBS0OmudI0xc0EyHVWF+1x8hf9MJM3v\/nNKa8MDGqrAHjtQD4OdNXjjAV5ln1Y4qBT3ttSrvGFaWKQlKSMy7KBcebv8VyHZfQjPPsZxrS+sieWBaT9XScU4XQCZh2TUDxmnRmgSSot8744DNM3P6WPm4F1wZ4Xbd7YyeFWlMZt\/uaWZfgugTZNORNmNVAqxMIGBQpETkvgYJgRLb5gTqC9+HydigJsvrSjo7klHj5w6hQzN8ey4iUck2epwDsoRcKAbRx14NfWrJzgbcYDfHFAvfCfiKhOrucwbgn2g1mR8niwCtEfcMzLrJARvNprnIir942wOqgDeuVwY5jf2LJI4ZdPyZscSHJyX1wdIqIKwlvt16ofwZTrJFJ+DvKJI\/iAcOB+N9oat67jwBUOzEUuKK00+CSlzVyXltIKBxCq2ojfWYELA+a78czfC0xUVuUy0IJwjIjquiKrT64v5cAZknkZuR+X5mdC2fzuJ13GNTj32GOPShDnMIKHkM0mhhxed2mMysZE6nH1byYz93RMiHpc+Y2xupxcTpgcT4AjGi1V5xJeBoc8CSsaIFxxvuXl21UOHS+e8BTmSog68QP39Qhwfq9aaBuaYrzC7PV4nIAfaDMBkWlEc8XoCRsvCpXMBP50Aoak\/IioVkTohkY0Stq6OOOAWYe\/GxBQlLMSh\/IkTwpbLquMR0uCOIfZF9eXJkQ2AYtTJ26vQGHJZkPM10pT+awhGAhzvFWUsE55wsPYuHCcuDRW+PWx\/AlN38aZE478dchMWji6cgHliVuCyY75RUQ174yVrFwZc+XYd7COICrTT4afoJGv+cQl\/Cl96FKn43LLLVe9kAMPeFhCe9CN5UJYr2A+W32g7ac+9anUjr54nLGS86RIR0T+rF5MMibL8YCpZkE\/ijjiodgaf7m\/suAbier6PwvejGTeE9DmDN4g3BjMvMlVFf3abkwbj9orjdU3voFXsSLksSZO37vmg67mnTDA2mU7xEEXCmLOS1ypFOUFgHD9aPXKwmB8EmzCAaFrjhp7yhQG1Is7LNCP5IVyuuWJh1rgKJ9Sqh36KyKSuuPhPQtLxHZCCyOnydFGuxVej0MwjN+g6cRMDFraGI1dp+c8dIwlu8bYe0KAHNfNjYi04447Vs9aYWYYUcbP7WGe0XFleRmH64i5gaLT1UOYCcIM6gCF7wzyQBtL9rIsE42piRkoYvGE04E0Nisvact83ENzklA9czjXYHUSmLbqW52Ym4Xl\/RLhMwVYKEraq++GG26YMO9O46Bed3RihivN0BgJ7c+KvI6fvx1oaDdR1EkdMl4\/bkQkFgv1l4f2SE8bpuXr\/7IvxdWBMLP6E25yAgy\/1PLFZdh4440r64zviNb4wRCtAIRhStwM9e8c3s0ljHK8Qw4gf0+WS0kwnsv8u9ERvr4r8fnxFm430E9ZKJhTVmja6OBNRIumZXqMHMPMYawbpfC00iXQI9qPB+VJyzVOjBfX3oT1CmWfSIP\/WjWrPyHvdCsBKg6\/8W2\/DZMXVoK2U+azBaOsl3umJS48Y5RLoYGb45XDBF4+WOA+ah5zeCI+TbDmNFx5oAMzv7qXc4QSm+scsWRfSN8PoA9a9SurjAlbaCxf5ThTd+N0VFiq8BVXXFGZHTS8DqQtcwnNRmXqlafh19OU38w7lttMShGdCaIeVmW0tJyeJsYMaiVS74R6Pdp9GyA25S21c565PV4xKbXqdumZDKxQ1UN6+wc6vh2u+jHHlGWx3euIiCXbrWxmjbK9\/IR8Ozork6mDsFYXdTIRhTERCwP6A243yFo8fP62uG0C9VEeK1zfJZpv4fJlVitpr76UDwI+Yiw9yvpLW7YBE8XchAOT2cRk4slCxsSm\/OS6YBDMaXCkycDsrQ6+mfFsEeQ0ZR3K8nM8N6J13F0e2iMfKzHWCUwWTjegxVoZSWf\/FnPx8ov9U+nUR72MIXuvXnkyf+BbhXFp8xiCeAokhUt4r6B\/9JPyuEzUOa25kffixAM40sDhUgC5+Vu81QihL6wXwEto7CVuNzoyS+q7Ep8\/04a\/E+gn40S89lFezRlt6wT6BD7Ae+xJ8gMHoSgs0kYsOR6UB49rnBjvzKfCMqBfSUd9ru\/lCSh6FDB+4MwHfsHPpXwbB\/IzVnzjS\/IRxoID+IWJtw3hu6yXhYCwDPZBCUUCzHyAm+OUY1GT8xVO8YtozWU8C63syYvLIA90cF3EFQ08LsdZ5Tu05htNjCVtBPUxZU6Khwe\/HejfbrIKjdFDWvnIT1nAItAipRxn6m6cjgpLiA00FJhpFDCoTc6IqMyHVtQEpXrap3NB36D3beXBnStgkjVZ54QAABAASURBVDJdZUE2V9rVtGN8ChjTrEVZ+XNmxGp2\/JQNxmRRoBGWk0XZJt+hUMCK0uEcphAZ0k6tzCKiOiTF1CLc\/tFcE5ba1SM0aHOIAqwhxjvFkFVE06zGFixYwNvANFGgEZbTRPim2N4p4OCCvUCm7Pr+lD0cJkiPYdvb7D3XBrOhwMykADNhrpm9MydpHQaMiBzcuNNAgUZYtiG6wcr8xxRiD4bdvA1aEzRFFIiIZH\/OPlN9f8qenz1fezhTVJ2mmIYCg1Ggx1RWkXgPsHfmEJK9wB6TN2iTRIFGWE4SYZtsGwo0FGgo0FBg7lCgEZZzpy+bljQUaCjQUKChwMQo0DF1Iyw7kqaJaCjQUKChQEOBhgItCgxNWLof5c6KgxaAX1irmOZvQ4GGAg0FGgo0FJi9FBhYWNp89tqO35pzQtHTU442+3V5wC\/MMX+XVL3NOnvJNDU1b0pZeingYQnzJSKq+6QRrZ+mc8+yXRxc4UsvxZqWNxSYWgoMJCwJSc8deVvQyzbuAnl+zBuFHgoH\/ISoC+Qe9\/XSiEu10k5tE5vSGgo0FGgo0FCgocDEKLCEsPROqrcqIxZruBGL\/Z4DIiS98Vo+E+QnpoR56Bvwe4pJ9VwW9xSZZ5GkdTTa1QxxGTwRF7G4nIgl\/Z6d8xNXnvfK6SbL9ci09wsjInF9dytLPLyI1k\/ytHtQuVv6YcYxf3tPNGJJGkaMH+aZOM9ZDbNOk5WX91EPPvjgZLxNRhm2FCJKmo31u1b0kpe8JHkOi7VFHTzx5xFqz4VFRKI0eqe2PubhmkO2LKQtwXNg8vDsXL424KK658ysKPOzd9LKQ97u4uUyPa\/mzU5llIBeHu+XV\/6JsDI++7Uh\/7JHDuvmukgfEYnbDa+MK+c8fxlHqfYeqzkf0aJ5O1qXabLf05geC8\/f47nl3DWHfY+XZjLjPZsY0Wqz8depLO+Y4oe5z7keWPfiU6c09XDjwT1OlsCIVpmPfvSjk18M8YJUHd\/Y80ZzRGtcH3TQQcnD9HU8Y8sYM9aUUY8f5BtP9UgJ4B8kj4mkWUJYmmDdGqcjPLxrknpkvJfCrTq9MejnWPwEDD9m7rmyXtJnHML3a1\/7WvKQev55oxzXuEsfBSh2Htj3mLp3PyeDAu5xdsuXkPL7hh7X9\/amxxPUyTvImYmwvBAGW265ZfJmZrf8MCjC3y8deH\/UO8Q5HwLSE2ibbLJJ8n4l4Sovrh8fMLcyLmXHYw4sO3AyeAHJQ9iEafkLODle+R55UD7GmMOnytUWwgINjz322GTO57JLWnuEH26O4+Jb+oBVCx8TNtsAX3vf+943brX9AAMa4Ye5z7l+Ekvf4s\/jZWIsemvV4qXEJ4T96IWXs+SZ8\/E2rvd8ucKM63e84x2p\/kC8fjA+4Xh3dq7cgV5CWJ588snaWAHtzIAEHtD1Cr1L4YhIAJ5yyiltH173sjytQiZexc+PAdMI\/GyS\/UtENzHh9As6SQfr7H7TzkV8D7VbzWc44ogjEqWm3laPI1utWOVnoEW2w\/MQcc5vJrp+XUE7rbRe8YpXJEJk2PX06wiEHPpERPL8mMew\/SyXMOCFFT8sjp5oTtM3Plln9t5776SeHnyHy7ICr1s9MRnbFgSjNDR+\/evb6pkQ499vv\/2qX9OB4yFqP\/nF79dqCAqrTUxNPwoHmBhBKL2zBlZqwkvw80vqXTLJMn6y\/epMWKhjt7LUsfxpJ7i+PUrPPxuB8ocvGj\/d6o9GfslHH+Gvxv5tt92W\/HSXdH52q50yIa4E88eYFGbrjEUKb\/YAiDCCt1zZehTEz9tZrJAHlFR4FENp+QGF7MQTT0x+qcMzlMLmAowRlrfcckuiVeSGlUwBo\/XLFH5VI8e3c2myzEEGe0QkvziBaCWuPOQlzzI8+wllAroEL8l7NDsLYWYapqqcZrpdzIl2ps40YK8ATVWdTC5mDysLQDNnRquXbyX\/29\/+NmG+gF+d63jGAY1VXjMVmI4oa9rBJGNiD7uuGA6mgD7MgehMYBE6TF7Mo6wlylcPggmjgn\/ve987YS6EE+bmW7jfNsTQO9VV\/sYQXMCsud5661WHfvxiT2aImJa84BDStkEIaKsNAjz\/ooZfN4ED9DdFFjO0NSJspoH6obN6UQwIEFYD\/ARt0EKcb0qHON+zGcxLr1Dhk4TVeG1xTkT\/w\/Mbuc6PWL0RoBYxfkPXGCVA4bQD\/FS8VbjzJFaIFjO22fy6iTTGoTnGD\/BcrrGFdyvX9+WXX57wDH4CnIKof3bfffc0l56gHCMsMdhMEMxhwYIF2t8X0GwxB4loSUxB\/BMFjMnekM6Ul28Mi98eg72GiEjMu8w3OjMikoGAScDD2OzDaFdEyz5vfw4T6mSjl64Eg4FpIaKV3i9iWOGWdTDoMNCcDsOLaO1lMrMoy4ouopUHs0mme06TXeHiI1q4mJyVQ84zIioFR9s8jQXUxU\/e5Dz8lBAlw0peHBygv\/0sVEQkdEiL\/klb4sE1IfSrn8kC6iVMXB3ky3xZ4ndKgzlkPIzdN4HjZ4KkAb7lWS+n\/i1tzku6bgAPfj2P8psAWkSShCY5jkJoVeZEqjHoWxyaYTDS1PFzOCYFtxNgOuXPfFlBGtvylm\/ZT0yOEa0Vr3xzGcafX6mAjyEKZyrGBOHZXxVvfwtd4QFjyqpBGt\/MsNpYli9c3\/vtVHWJiOTE+1lnnSWqLSiDWS\/j2\/PyY+XqVU9gfuUwioH65HTm6Uc\/+tFKcYBjjKGXVY157xS+cOCH5CNa\/EC8MOOfaVvbI6L6jVAM3dwR3yuwNkS05mNEdxdut3zVjeWBggvPXM2Kle86MEszzQsnICk+\/OApT3lKMhadoGYaNQ6EtwPj1u\/usiRQmMvf6kSnnEb\/Z3\/dxWMpaPrMeBeP1\/rtWfXCq4R1humJMZeM6YjW+DAXLd6cLYjoPC7mldW1UmNmFaYTCbqIqBipwW7Qpw7\/4OtE+y1QCNs3v\/nNiQakcjqx7AQ4\/YDVmtVOrgOthtCr50G7ZyrLjM6AoWkRaNrzwQ9+MBkcOR0NCFPVPpvV7SZwxjUomKZpTsIe+chHVvb6XrUnjAqDUla5gre\/xJxodSLfDPYlaNLic5gVpIGIkeWw8VyCEsOBpw1coD+0V5xJKqwO4vUtQKscL60w\/ZLDuBmfZlvii8tpCGjfdYCPcYmXT473rawyLMdNlqss9ZV\/RCQ0TOP8y\/jQ0JSbISIqr3y1s\/ro84++Y97KyXI+aG2FhU4EmT7BAOBhvIQqRdMKRhiAYzX+nOc8Z8x8ENcJ1N0ctHr9+c9\/nnL55jblRjr5cjNgnixUFMSMT1Hxu4aHHXZYRht1S6a97777JsotvqRsSA6MyMe3VScGJ3w8QDtvrL761a9OzNbwhTG12wO2Ohc2nUCBZlK1\/92pHup52WWXVdFWkXnxUAVM8A+aMqGyqMiKideCh78d6HeKlYWLcWYMsqTgc2jdK19sl\/dUhVl4OER3wAEHpGzRyONC2\/GdXJcxwhKhcoQG+6FQ3wanwW7QG\/zCgEwJGNoFDYQAy2YrQtcgZBICWeuRbjxwcjAiKg0youXS5PfZZ59qghJ+zA80q3peNBwmUKsGbaBR03yPP\/74lO3zBJ44nctuHxFVNg4elQylClz0Bw0oAjR9QUxENCgaiu9eQHmYCu3YIFNHgk9aHaWO\/MBAfP\/73z960ow\/p8GsSoEPvxtERKXwpJF\/GLpJMeIdzZugrDN38QA90YqfNoo5AX5hZbxvuMYFv\/6hNAH9JwxIk+vgOwMa8xtL0nAjWn0jT3UXPxWgfkBZEVHtzRPkxjUwiXJ94YCMz1+Hkr71dHXcTFurNysnfc3Mz2pDSGV8+3bGlN88JMyVT8BQxAhNeNLbA+UHH\/nIR5I+Mgd8W7Wbb\/Ix\/s1v81lcvXyrkLzFYvwzi0pH6Ea0+olFRZj0hLh9LQqzb4KK0op25rIVkPASCG+CVJg+p9z60WfMmBCxYso8Bg4wtpgLv\/e97\/mswN4weggXb+4zX4okBCid6klBNU7RRNx0AP5EyFidM3F2qwPaoSscVjMKg5V6RFQrZUIq0xtOryCNfOzL40V+zYdJHM\/PeeS6GVv4k8Ns4vSPuaouFHw3HlwTFDfTwUJOHY0P4+EHP\/hByuPf+R1jXjyY5w+AaIlvUtIUTCodg0Fj1HAQlLDInWXgsnX7pkVIB8\/EXWWVVRKhlkHHlgwD3iAgb6szq7pO6ZlWaFzqY1LDY9IxecA+++yTxKkP8w1hDkd7TVD+OhBkmJNwk405p9SChfcCTEUeaTBB1dFeRU5XatBWyDRwcS984QsThpbTOI2sDuJ6BQIRLoEDMGx9LizH8ZeAVgSbMHQ3iCKiUmL4hYmDA5cfUyPogPoKA8rI+HCB8DpIo2+EcwE\/UGduJ5A\/uih7PIAHv1Neysp15Cd0Mr2kwVwx\/hwGF564yQCMiaDKeTPVOixnDDu0RUEUZ9XBjwkYn0yu6iaOYucgB5o6pMfEKRxTJDT5uwFhjZHCIWyZRZXPfGt+C7dqy0o2RoPhCH\/0ox+dCG3nFVh78I2sKIrPQLA5MLLtttvmoMo1L7Vl5513TngLt76KrRDb\/NFHFIFMB4q2OaXuLDfa0iZZxyAmO3n1AnA7ZjQSob22jfABY38kqOt\/K0v8FpJVqBPWmU+gkQNiFi0ULDi9QlYEMz5Fxp57Lks464RxR6Ei4D\/0oQ8JTvpqueWWq6xs5oXDY0y0VeQs+EMho6QZD7bVsvnYuKEU5CaMCkvMDBM22DWYYDCodaCTp1nDMLFMAoLFoMZ0aGkGo86Ssb0RqyYEz8CUOAwCYkiEt0GCWSmvBEy8myCFq+40TsftCctsshLXDrQBo1K2eOlsxvP3C1bYmFVOh3lkv3pkBoAx5HB7lvohf9PkMMT8PZ5LKICI1gqJsATaExEdTYwZJ438y+lHvNX\/iBhNBw\/jSLV\/wowJE85kVF4NZcxnRIyugNOifxGxyJcqq8LoxxR7tJ9CSAjnvtA+yiR3MqtDUPotT2XlcrbZZptqr9B3RKQ6U7ZCcLiOmQkOsNeOyTHhY6iYHlOeA3XjXQMzZ9xflE99juEdK664oqhq1WrF6YPgxkv4zRdl8wO07HRSEo0powSDVSu8cs5IT1HHpI0t392AomN\/E466M7vxZ1A3Ajh\/zxbXuNPPeK3+oQipO8WHco3Z++4FLGoIWGMMvzdvCZE999wz5XysGG0DcOWJ9zusZiFCQfrWt76VKEXGJj5m0UIARURyRcViQLqZBOZybo96GcvkHj9ASy4YFZY+OoGJwPQn3uCntRqADni86EUvSswm+eg6HMJSofyDgMlrIJRgUlgi61R5WjYzofKXwERYTsocJy+dudZaayUmAyZiq+deHziQPudl36bUOHJ4L67B0wte3puFawByM+hMhxTy93huRFSCLaIleAx+kyGN\/CMEOtVJm8EIWjVhWB6YIAHJvtrjAAAQAElEQVS\/fMTBKQWhMWIFAsc40XdlvDQ9QZ9IylCu+o0H8OB3KgJdIlr0gkNzznTiz+MbDUBEVDROE\/yHdrnvCTqrRmNVfcus7fdFRLXKj4hkfyXHM0Ey1xoneVsBU7BXzuy41VZbJcwNvr19QpRw8t0J9KP5Lh5DtSqNWFw+E6I4kOufXWFrr702ZwyUiuKYiEUf5jFG7EoI6wVlkmUrC05bLFZXi9A7OvZy83xtxx\/QJpu+O2ZSRDi0E7G47RGd\/XCLpBP2qn8eexGRmLNZqPQ14cZ6oBCLmpLRC+sGaIoGaCHPzHNsv5X5UPZtJZnz5jcrmfpQzIxR1wTJC5YD+8F5jlmluuKVx2O3ukxlXET0PG97Epap9o+GgJkgEsCYnPKERitcf\/31eYcKGNSmm26a7PfljB0o0mn5m4uhAf4S2NJ1VhaO9mNorZhHNiGV+O38BqpwZg8mKf6pAMJmouVExOiqjaAEaeQfWkXEaFya4D\/MnpabJ4nxYSJbSRgzE8x+2pKjUy48IsbQK7c1IlKnf1bfOa4bHeSVGQqlFJNB05y2F5fwNi8AvzQYmLlpOwFTsyJRDmYnnrAkVPgnCmVbc15WGtnfybUiJUAjIjm8Rzhm3IhItktYlRxYEq599kj5ewV165eeveY9FXgUfQJNWVyWAX5gjqEbP0FW0k9Yr2C+UlTg4+15Ve67HeCH9s0tRFj8WOIsniIisT7a41RPlgz8tl0esyGsJ2FpUOZJp1H2Qkw+5gvH\/ZkMacHirEDz6s\/3sMEAyXma9DTO\/N3Jxbzdy8OIIlodyAxLa2WDN4E6pc3h9kkdd4cvjNY03iCCNyjQFnNa5pHs52I8\/U6EiEgEl\/TokNucw4TXISKqlYtweJQFDLcOwsUbJyXDtR+D+dnHjAjZzBqIiDECMXX5F9FqWylQM31zMrThj4iumixamV+pzb98wCJHmWtHH310sgeHzlYGVpQYV0Sk0vRNObJt4fCP9ISlfqQ0+rZawGD524G+zEolUybGp0333HNPdfjJalY6rpUJf7kdQsGGLzyD+mU\/l7Uk8w5jPu93issgD+3I3724Zd2lpSSU6Sijs0WAljQq21D3G4sWGPXwfr8jIpU8t55ef1h04MUe59B\/tvKcbyFwrUSFGbtw+1Vu6uVN5\/eosMT4mVbsK9Lg7DfkijFhZDOLzrJExxzd07FEt\/zPwpR2YXDmtMNyMR8TNGuV8nViq5cBUZqQdLw6Sg9M4vrpOuElaDON1gVem9fiaP3uRhkAvocN5eEHDNCEzmUwh\/Uz6PJKxgTKeWS3XVgZl9MSsGVb9TetE2B64gA86SNijFAQDtIk\/lNX+yiEwHgAD36n6kREdQgsx2tv9pftFJbzMSd8A+OVC7Q7f8PtRvOISGgqXQZpWDIcAslhrCQUVczf\/pR9OAzf\/vYJJ5xQCTDWFGVLYw7T6jEx31YoXhty4Mw3BYzbCeDncwvMsOZ9xiVkmUR9E955bDIJEqzC8Q914AcElmf3+DNQRFmP8rdL9u5LZgVMW23RMEFnHGVkfycXP8pz3jyi3Je49oTtj5Zh3fz2h42BXgBut7z6jcuLFOnq\/aAPMy9DS3wcXjuwz+l0MRyKecnvmebJA+nE47P87cCqkXKGRzroBcd4MF8ob\/itsLkAo8JSY52ERSiCwJNTBrQOoCnmPQ6ChzZKy7TkFp9NsAhiX1BcHWiy5WSB2wlM\/oioVjURLRcjoqXkTtUR9nM65VGGm+gOxQgzwDATk0ZHY0A6VhyGBvhLwNxARCT4eUPYfUtMoMQdlt9AzuV4qQWTUGf7QO6v0uR6LSsiKtTchupj5A8mLGzE2\/Z\/RFTH0UViumiXGURmYOIoLBFR9Zc8hcGzmuGiqXEifDaBfZzcHu3VHvUnlLLfuMw05PrOOHlcsX6ggXDjNiJ4qwNL+pFwzAxG3uZghbDoj3061y6c4DRPBduTolARoq4wOUAnHHMybmw3EJZW\/RGtg12UWytXeASn\/Sh+oA+56q8d\/KxFhIi+017zTf3FOeTmoJ82HnfccQmvEK5+TsnyW004ZciPfzgsQrGSn9WwqwbiSnBKFx8SZrxhwNoUEYmCgxcZi+K1M+fvW925gCKhTvgVfPTLfYm3mVP6xKlzeUozGyAikoM8ZT84eIU3OKCJztrhQBgezG\/fNEbSRSx+6J4wZRrF61mp0CT3jVOuwqS1F75gwQLeJcDcZmalgFG8Mp4xpy+MZfVaIuEsDZiX621AWill0wkNEUFpZZ6VgmdCISitJINJJC6DyZ\/jSpcd2+DMeBN1TTYCpZd8tIGQy5PFUXEar8lmwueBRyuz0uyWp5U3YQ7HZDYQ6zQQN1FQDiaZ62z1rs5MGhgYRtlvGfICOR2mGNFi3BFRCbscl119Tmj4NvgxdcAvDPPN9IuIBF84MFHgWhGUfY95ATgzGdDH\/k1ES9hgvAQbZq\/eEVHdyYoInxX99FFEC99cgY8OENCxpI+wOqALAVSGY2To7BpUZmLmmcMtFFUMzVjUD\/atKIdHHnlkwggJH0JKflYLhCw\/QSlNHg+sJpgchkfIwZEePzAWMWTzzaE4cZRWK01tsgIUBlinhPG34ynqZz7aFmFGhleCFaDVJMFYhtf9eJODduqW41i8CGvfBDEcdwcp6a4E5PMOaGcFi6FbkaObfpNuNoAH8N1v13f6Qd+pPx6h\/viaMaN9vttBt77J\/N64woMiWuO7ng9eSVmj3FBqIlp4VqNoj5\/qB+POfnREpH5O8dfLm+7vUWGpIkwaTJ1MjvzCAL+3LO13sEeXwBxjwMEDrpCU8dmPaOXAhtsvmIQGilOxzLERrc7pJR+v92AQ+VSegebVHMfoHUOXByZF46Qx+e4EW221VbK6Fm8lbf+Tf9jgGSwmWCtqeWu\/+2XCcjuE9woRMWYfztF7K5J8yCPV\/s2fPz+ZOPoOLuGRUdCPIAE57N7zV03P+uQ56envOT6tsPqi048jZZq09tQwyYwbyz8gPeuwP6VnHnJBWm7F1avgefdaLi3\/kEdW\/nZ\/HvHK96fnHHnTGNj4y39JKz9963boQwlDcwweQ80ZRrSUAuHalsO5voUTbr5BRAtf+yO6j1nCUh7SjQcRrbzUEWOzmsA0WR\/sI2FYVlReZDFnjHf9VuarPN9WjQS5uhNWLEHCgXFAsEZEInjNmTIvYyCPR35pMuAd9vqZjXPZ7jYywapbxitdgo3Vx+pV+hwnPQHruoKVM8Gc47isR0yChIVvoO6UnIhIxrlFAGYuTl3dVWVCRithswEiInl8gDmZ2R1d1Bt\/tYhgVi7pJq4dwGnH7\/WlMYBWxkO7tMLwesqbvi1NtVa0eCQ+im9Qqigo+otf2tkIY4SlBmBqtImFCxdW+x4azK9zaJ60iBJogjoHHrDBX8avuuqqybeOLJmtskpg25e+G1jNYO60wogWo5CH\/B0IkJbrW3gJEZEIH2YKeFY6Xiwy2DyPJwwYbBFR1VlewrhlnpgeASlOPq7OiIcnzMqj1KAMKuFAO8t6uX8kHMhT3mU8pmT1IF77mX4xAKYPeJijlQR\/hohI+jHv22GmOU7+OZwZBrNET1eA9K84aSOi+hUDKwuauScBTRzxQB1KgXC\/Beump+7\/i3Tfhzwirbbhy9MLv\/DHtPWJ96St\/+\/utMlBp1eHiwgcaZ+44\/+k539lYVp2hQem5R64anrR4VdUuC894bb0jI\/+cozww9Ae\/eI3pJedcHtae5t35WZMqYsZlXRDJ3QX3q4iwtVbW0HGj1g8ZqWDh44Zh5CUL8VOPKX1tttuG52HxgAQJo7fUX1mXlYhpkerK2ZZTMxVEavEiEj62Hg3Xo0zeMpQvlUeYekbuHNJIZM\/kFcWnhGRzJmcl3gH6LpZOuyJuXivbPhMwMynLDS+QX1emE\/MgXiPeCC9FS6hZ4yqax0osQ4rwQdWNNoDD\/\/RTu0Rp94WAYRDnrtcZcOfDjDO9I\/6AfyhUz0In1NPPTWhC1wrOS8m1WmDtuIBXlTmB3e\/\/fZLJZ3xyO23337Mnn2Zht+5CQqZ8UOBj1g8to1jCg1FxBiH7z67xcqKi+7jCpsu0L\/6GT3qvFqd0EgcQDthYAlhKbAdYJhMGia+Qy4EpH2cdrhTFebIMoZf\/oxMWTYzmIuxmHREJCYaR+QNrhKPn6mMBq59EVH9YK9v4eKnGqwEIqIy7RlwzGrqDdwxNaHUiaaN2fL3C\/LEfExQ+yAGUc4DUzFRMWZ3qJjkclw7d43N3pyWvd8D0t13\/Cudf+gb089es2L6+5\/OrFDv\/8inpIdtsUflt\/pc5VmvrvwlrjS+5917ufTwl70rwauQRv6stN7zknDx8OSd4ZTXrZauP+P4Eay589+pQkoKs5pVEKHnLiTgFyYOzl577ZX00Y477pgwOX1GyJjoBGhELEEYAuXCCy+shDAzOUWYIFkCsQloKNCBAlbxLBjGD7lQR6MsUkTwK2PRyeYFCxbU0WbVd8\/CkpnCKTSHDJjmrHjYvZkImTGZWxFlqlpvH4zmkldY9XLF09BtehPuBAMNkumHvb+sqz1HeTkAgdnA3XTTTZNv4eLr+U\/2txdWvFyiHHWl6dHYgGenMMh58+YlzJL5DF6\/wOyMoRKETCfaKn+auFc45M9clgVzp\/wJthXWfFIVfftfr05\/v\/C0yn\/j2SdVrj\/3Xa1llr3\/Ohum+zzwIYLSLVeeNyrorh8ReL5FiIfHD+6zcmuS\/eeu29Nt11wsaE4DpcXKTh8wX9HIWRQAvzBxcJjSrJDsCxGklJw5TZymcQ0FpokCPQtLpjymOqefrLbsKTgoYz\/AMtyqzQqHYPLixmSuOh2asBKywupEN6sheyVe+fFIMW3dfiQN3Ea\/VWlOa2XKpODwgiU4XPZ638K7lZPzGLbL5Ok0owNETN0EVy6D1mZ\/immUaTmHD+I68Wjf9a677kobbLBBokxQEjBscdo\/Xr533HxN+vVeT6lWk1zfndIQilaDVoa\/22\/TTmij4QTxMvdb9JTaLX9Ld918w2jcXPasssoqKQtG49aJU8BPWIqDgwYUWfPR3DRuhDXQUKChwHAp0LOwLItlsllnnXWSI9dMPyavF\/\/zqtMeXl51euk\/28PLPAbxW2ERgB5D8EIEE2S7fKwqXQ+xb1i+fk\/ge4iAJu4lE2mtrjChBz\/4wYlmHtEyW0VE9S2cAIUHfyrBfgLB5RBVNmeggfp72cV+1DDqYz\/Ja0jM1ZQJK0tmd0oGWg5SBiFXmltvPOcnXbOx57n8qmtVOP8pVpDLzn9QutcKD6jC77vqmmnDQ84fPeCTTbtV5Bz9Q\/gZww7wbL311olfWNlc89HYLsMaf0OBhgLDpcBAwrJeBZOX+ZJmW191WmmybxNsGHA9bT\/frky4z8NsSBhaBbVLzyRMwGD68+fPH4Pi1JY9Hyc8rX4Jevt2Vm\/2P0tkh5KYbsXDK+Pmkp8iwLSObpQc+djllQAAEABJREFUh370pXYP0k5CjFAj3OwzXvCl3UbNrZ3yW\/u1B1R7nuLvLFaQK6773NFwcSU84hXvTU\/6wA\/LoDnnv2tkxe\/ghjHfDswpitR0N5xCSZHjTnddmvLnJgUsfhzIAfxT3cqhCMuy0rTcdqtOGnGJN4ifKdIj7a6weHkkorUKrOflMrLDPY6QR4zFcZjF\/pzVrhUaPGZdYeLKvJg7mZcJ6fpF8RJvLvjd10I3bcF8DUj+QSDvT0rrYM5DX7Azb0cg8BwCygjX\/uLolE25ZV6XfvNDlan3tLc8Lv3zmssq9BXWePyY07NV4Cz\/Q3mxr+5krNU+cysFrx04M3D99dfP8hbPreo3rZmbFBi6sKyTKa86\/ZbdyiuvXI\/u65tJ1AkrR9G7JXRCC8MhuOt4EVHdNSQoae1wCUwHWyLGCtaIqK48WIE6UJPa\/CN0L7\/88ur4NiEzHUCjb1O1voIc7qHkSIRJW7Xbt\/TdL1xw2C6VULt0RLhJSxASiPx1EC4+hztBe8X3Ds6fKedljzOHE6QEKiTC2GlZ\/rkCthj8agMrDYXNQxSdgJLXbpzPFVo07WgoMFMoMCFhiUmb2A6GONwTEdVVh4glXaf2mJFmSsOHUQ+CklnYIRh33aYLKAYTac\/GG2+c8slbV1YoEJgwATqRfAk3wk8e7VaAj3nT51IpKK0Wz\/\/UTtDHhdtvuKq6pgIxn5bln+2gLx3iobDoCwfovH7VCdw7nqgSOhU0M1fyfb6pKG82loGf3jVidufOxvqrs7rPjjaobX8wIWHp8A6zqJckMNj+ip792BiAaxVeHHFBf7rAqnhQatrT9VSV+6VOCDsB7PQvpk0JmqyrCATlKhtuM1ptgvKcD28xan4djVjKPOju0rwXUByYi4g5QQFzZd99960uv8+JBk1CI5wVIGi4k5D9lGSp7rO9DZ0INbCwJBzdr6QBO1nqdRlaRSe44oorqldxOlVkmOEYPwHCJFrPV\/2E08bdT4Tr1CkmJa7E9y1cPLwyrvRHRGXaZQ6bDogYnKHuscceySEmq\/6DDz44aa+fHyM4CX9XUxyUSl3+eW7Os3OeomNWzahOxOZrH\/8pTrjCX\/nJm2e0av+xnaCU\/mkHnlmdfn3W5y5JTszmRPd50EMTE6zv269fyJkzYOyCiMH7dc4Qo0NDLr3utvT2Iy8eA3secVHa7WsXJG6OO\/PSf3TIoQluKNAfBQYWlvbxrr\/++kQDdvl\/2WWX7a\/kScT2pBIB5yBQvRh7lUxahKWTn\/CYkGnz9UMtvoWLh1fPa8DvGZPMqtFrLnfeeWdyfca9SpUjMAlOAtShEndaMW9x7cAjBB4jEMes6jQs\/yrP3jY5Ect\/2zWXpFsXnlu9zOOFnizo7rr1b+kPh76u7YrS3uQtl\/1O8upErBOzPgjNNV60K2+S\/soffLbyz4U\/FBQH0y688MLRX\/KYC+1q2tBQYLZTYGBhaVVG4GCiETNLA8bgPQXmorYVcNlJHmB2RcIpQm1ghvSTQgQr4V\/i+hYuHl4ZN9v9VotWjU5b+pkizwCWbSI4CVCC1EP57qCW8aWfULv82x8b3UN8xCveW60GufAItIsOfydv8jKPF3qqj5E\/y97vAempHzq5wrcyBVapVp8j0enSY\/arVp78BLF4+NIJu\/LEQyshzD8XICKSt4q9yOOdYgrbXGhX04aGArOdAgMLS2ZJr8h4UNl+xEwihBO49lL9CO2PfvSj0ardNbJ57qK9KyIueYsg7D0d5i6by\/lMr8K5fn5GuD09eMLnAmjL61\/\/+uqtXMqA38ps1y7CkiBlNSBYCdh2eMKuP+P4dMbbnzwq2IQBB3x+sctaowLNydW8qhQ\/HhDEXgW69rTjxqASwL957ybJIaIxEcP6mKZ8KCf2jD10zhxurFL+HLhqBx44p9RNU3WbYhsKjKGAbSgK+Lx5A4uWMfnNpI8Jtch+FwbqgIun2ZjtOoEJba9wqhrvMQS\/bOAKhN+yPOqoo5IDEwTA3nvvnZi6cl08qODJPi8SwYfL9SC18HxSNOPPdpeZ1ZuyVoxO8958881tmwTPL0N4p5ap3WqzLeKiwCzYXPPIUH\/SrrwKknHqrufwCN9F2VZOPV0pgCuEOfLHAYkf\/vCHyYtKuUnmVPnbsKXflsJUzqtcp8ZtKNCJApTxiJllbexU137CJyQsFWQVx7Rp5UED7gTMngSmNFMB6uX9TCsop3aZtqyivPnqJ34iFnemlRMt3mk9jAou94ADDkjCxU9FncsyrrrpX2n9fX5VwWkXt4TZN864tkRp\/HOQAvYsPeRvq6AX8BLVyhO8vzyLydhUfRop8Ndb70rjwTRWb+hFjyssmS5dyG8HhJAL+d5q3WyzzZJfw+gE3otl2myXz6BhVoz2dKz82uVhT9JjCFZO8DyBt+WWWyb3Iev46ma15UAPXK7fChRex83fHknQdpfHc9iw3B+fc3Vaafm70xr3ui39ZMR\/0VV\/TV\/56WVpv29eUP0kU1kOBmuPdv31108NTC4NBn2oodeZGxHJgTLl9AIEJdNXr\/k3eA0FhkWBD3\/n8lSHD50wwqOO\/1PiivvG6XNHwe8qLAlKT7353bJ24CSsX\/UAXtbxRFcn8CC4k6jt8pmtYZiUtntvdthteOZD\/50OecVK6YBXr55etd4yKW6\/sfrmr5e10korJYrDoYcemhqYXBr4cV1CbFgMRz5Mr4Cfcub5RabXXoC1pjHDolwDM5YCc6Ri4wpLqyZ7d1ZZDeyaGhosvTTwUpOV+zCFJSHpuUXAz+phG6DTdkY9fDK2N9yx9aMCfrquHZ\/DEw488MDkkF9EVK5v4e3wm7CGAnOBAl2FZW4gDfess85KDTQ0WJrHwNlnn52nxFBdJwdBztTD6Z3egq2HOyHLwpHTTtT1k2yeObzxxhvbZsXaJJ6lyME4+6ubbrpp8i1cfNuETWBDgVlOgZ6E5extY1PzhgIzmwKEpAcvAL99dj+X5pRrLzDMt2GZfz1A4epKJ6pZbVphexbRz3Ftt912yaE538K7pe2UZxPeUGA2UKARlrOhl5o6LlUUYI7tpcH2Kn\/2s58lB9h6we+EY59UPg6rfec730l+e7YdrqtErl452OaRiojWifKISL6FE6Dw2qVvwhoKzGYKNMJyNvfeHKl704zFFPCMpKcGmUMXhy7pc8jLvWH3ZCe6V+gQn3xst5xwwgnJHeMlS0yJUD733HPTWmutlexpljhO5XpjWDy8Mq7xNxSYCxRohOVc6MWmDXOKAkyZ2267bWonMAlG93\/XXHPNBM+1oYnuWTL\/evHKPeQtttii+pm9dgR1CImp1j6pl4VKHL+7udpqqyWC94YbbiijGn9DgTlBgUZYDqEbmbHce6RRYyaA3ypB3BCKmJQs1A0DdK90UgroM1P1cb0I\/QDBoH5AHOCfKfXts3k9oXsA48lPfnLyKIaHPkqB6a1jrym9613vSg7SvOUtb0m\/\/e1vk1VdT5l3QGI+dfWr+4+qp+SdZfRv90JLRCThxry50KGoJrihwKylQCMsJ9h19pcwEEyChk7TB7LFNDB\/TN53A90pQAB4GxX9rFwIju4p5l6sVeL73\/\/+tMsuu1QrRys+z9u97W1vq55oPOecc5LrIq53fOYzn0kOBs0mKlx++eXJXuugkNt6zz0pmVfjwQhWlQTeoGVOVTq8JMNUldlvOehYEXTkD\/+4MIKX\/9fLkjbHzQa3EZYT7CX34gwCzB3jYo4CfqXET4ARAGCCxSwVyTGKiEiEpceYrVTQFES0DpMsDYTQbg\/+e5bx5JNPTsyeWTA6eerd2Mc85jGzkhR+TcWcGRTMtVbD72krdI0hOBkyQxY+aJlTmQ6vmEh5k51W\/Vr0T23pj+5ozQX8Gb9et9l2EGxebkjjDkYBg8GKEoOr54Dpi8uDwsS1CmVGtIJi1mJuZGorB2HOx2CDDwfw57wyDrdXPGUyD8tLmdJJ3w16rXPGs5K2oq6Xod7qLxzwC1N2Titd9qsn2sIDwuG2A+2AI1\/An\/Nuhz\/MML+t+azPXTLmJ8ae9IEfTrgIq+qPfexjyZUMmXkA4PTTT09+g9TqU9hUgzpQXNC23h++hYuH16lufnTBvBgUzCd5R0RChzqIL8MiWkqWsEHLnMp0+n0qy+u3LPVLi\/6haTso+4B\/EXqlBJflteOZGXcmuo2wnGCvGCyYejthZ6BYYS6\/\/PJjSiG0CEx36qxCRd5yyy1JOD+QH4GGCcEB\/ATBIHi0OoJMnayCDXrf6q688UCZ49VZHvDUXX21T3nC1Fv9haOHcoWJi4hkFW41KQ\/x8CJajE5YJ1BWL3TqlH4i4X7kuvxtzZyX390kQAnSHNbNRYsMzPkeBHAy1VN2fh7uPe95T3USdccdd0znnXdeEpcBDmWhW\/7DiiMIvVvr3WRjoczXt3Dx8Mq40h8RbYWcedQLRCweExFRHUaK6OymFCn\/6yX\/6cQxVzJMZz26lR2xmJ4RkSLGgUz8Ebeeb0SMhM6e\/4MLy+VXTulBj10Mc92vvW36laaEyWMWVjUEEMGE+bVBr4Ky0CAgpMdcaFkYpTjgcIt8xcEBfklFWL946iINAbnCCiskeRBIQFxVqXH+qBMB1qnOOXlEJMJYfYF02qJ9uS2EqLYIUy846uY7IqqDIr5znp1c6eStPTlvZcpbWM67U\/qJhN97\/qpplWe9usri7jv+lc4\/9I3JT41de9pxVdiy93tAWmOzN1f+bn\/Qn9JgJQ1c+n\/BC16Q8rN2Tph++MMfrvbnHOZZd911R+Pg2L8kMLuVMaw4ip\/ynZqtl+lbuHh4wyqzyaehwEyhwMDCMkaE47yNPpiWFogFG7fts4iWcMCsMWgrHYIT42NmtXKqJyQUSmEQEZUAs0LAPLkgIqpTj\/LIIC9xJUR0x8t5EiQRi7U5dVAXeY4H8OBnvIixdc7htEfacf7O9SQgIxaXHdFKz3QHJ+P340oHIrq3v588e8Vddv6D0r1WeECFfvtfr05\/v\/C0yn\/jOT9JhKeP+6y8gDMuoFcGyojTqfVn7Tp9289E83ELGQKCMbDNNtuk6667Lh177LGVAJctpeWrX\/1qFW4lDE94Aw0F5hIF5g3amHtuuz7dc8HIhFla4IY\/dCUVBmFFZb8G8GOAVppWOGVizC1iseAQJwzTIdi4wApV+hIITXEl9IKnjHnz5nHGQMTYeoyJLD7UL2IsrjD1UOcCta0XLeoR0guTB7dfkA6M1\/5+851KfHShaFmNAT9x94Mf\/CD18tQdnGE+d9dLu70Hu\/POO6d99tmnerzgqKOOqlw\/tiDcz+X1kk+D01BgtlFgYGGZRoTHPRd8c0RgLh2gvb12bkRUK0VM0GqMsOxFoNTzZzK1B9QO5Jvxe8XL+NmNiGrPIX\/PVnfQ9k+kvbcuPDfdds0lVRb3eeBD0v3X2bDyr7Te89K8ey9X+W88+6TKnUt\/jLtPfepTad99903ugrt9cZ8AABAASURBVPqFFK6HEoSLn0vtbdrSUCBTYF72NG7\/FGACZGq1b9YudUSkefOWJDHBaUWUin\/yiogKPyKSf0yU3E4Q0R+ecsu81KEeVsaXfnjwy7CyzmV4O7\/09fCcPqLVjnr8eN8RrXTj0Wm8fAaN\/91+m6a\/\/+nMRDg+btfPVydiV9lwmyq7S7\/5oXTF9w6u\/H3\/GScB5csBnyuuuCJ9+9vfrg7\/jJOkr2iPHuhrq8h2CZmKP\/CBD1SPFMAzB\/baa6\/qkFY7\/NkY9tdb70onnXvTuHDpdbfNxuY1dR6AAkty8gEyWVqTzBsRhEyJmJe9yjodCAPhcCJajB0O5i6OHxAkTIlMuTlPfmHi4GTI+6HSy7dXPLjqibnlvNRNXfJ3NxeeMjOOeqmf8tU5h9dd5YJ62erBpCyt+Hq6Xr6lU756qE+ZpqRTGT5Mv0M+y9xvxbZZ3ne1tduG9xKI1nvvvXd1ajQiqtV\/xGKXsHK4Z8GCBWmPPfZInZS1XspqcNpT4G\/\/vCv9aERYjgd\/vfXf7TNoQuccBRphOYEujYjkRKksXP24+eabEyYNnHDMVxowt4iAVgFBAR+Tw+jhYvYZLyKSAzHC5EHQwJOGK46giOgdT96YsLLkoWx7oVWFevgzXp07ZUEY1svWHu1SH3ERi2nTKZ924RG9tb9d2omGEZTrved76b6rrlkd6KmfhrXCfMybPjdQMX75Y\/\/990\/6Pz9YzrzpZCx65kyf\/\/znJyu6+fPn56DGnToKNCUtZRRohOUEO9zKxlUFJ01lRRABqzBh4uCIy+CbkCA0CFYM0P6m8IzjZK0wQtFlfXjydHCIsBwEz96edPJStjqUZYrrBPDgSyd9uzp3SpvbEhGVMqE9Ob24Tul6CZe+Fzr1klc\/OPYo7VVKc8uV56XrzzieN116zH7pn9dcVvlXWPNJiVCtPnr8Q4H4yU9+klZaaaV0zjnnpMsuuyxttNFGiWD0+o343\/\/+92m99darhOn2228\/p8yfPZKpQWsoMOUUaITlEEiO8buDOH\/+\/JQP4zgRK0xcuyIIUjjwM7Ov4xFQ4uAA+RMOg+JZnRDe8lI24Sd\/9azn2e67W50jonqnVH4RS64U622BJ6wshxKgXjk8YmyeEa3ven3hy0+7QCc6lWVN1H+fBz002auUz+3XL+QsAa6WuGKyRESXAMLQKVd3Ld1ZpBxtuOGGiYD0WEFEJOFHHnlkFXbEEUd0ya2JaiiwlFNgiM1vhOUQidlktXRSoNf7lP1QhwIQ0VI6vAPrwQuHenIewghUJlsr\/RzeuA0FGgpMDgUaYTk5dG1yneMUuOncn6a7bv1b1coV1nh8WvnpW1f+R7zqA9U+pg9XS1wx4e8VIlr74FaY9oml80gB87dfH\/ENIlovHf3pT39K9qGFNdBQoKHA5FGgEZaTRtsm47lMAULwyhMPrZrIHFu\/OkKQXnT4O6v4fv4wsz\/ykY9Mv\/rVr5KrIdI62MM8\/bOf\/SxlAeqAlpd0xDfQUKChwORToBGWk0\/j0RIiWntu9tciWia20cgZ6omYfXWeKlK6R\/mb924yusLM5bp7+Ytd1koEag7r1Y2ItOWWW6aFCxcmplaPqK+88srVb1gedthh6YMf\/GA6\/\/zz02677ZZOOumk9LjHPS4RpL3m3+A1FGgoMBgFGmE5GN2aVHOEAhNtBoFIMHpEPYPHCiaS7wYbbJC8huNU9c9\/\/vPqNShCkpK17777psc\/\/vHpi1\/8Ylp22WWre5YORk2kvCZtQ4GGAuNToBGW49OowWgoMOUUeMtb3pKuv\/765DctnVp+6lOfmk477bT0yle+MnlU\/cUvfnF1tcTvQ0555ZoCGwoshRQYWFiuNG9e2nS55ZYaWGeZZZbC4dE0eTooQCj6uasVV1wxPeMZz6h+skw9mGW\/8Y1vVI+sf\/\/736\/MtMLnPjQtbCgw\/RQYWFjed2RvZfmlCDp1lWP7jvXXwXuZ4rzC0iltv+EOdzj5CPjbpRcuHvC3w1nawvSBhxC4M73td955Z7I3aU\/SIZ6ZXt+mfnOHAp\/98VVpPPBe7txpcX8tGVhY9lfM3Md2itGl\/Qxe3rHn5Im62cCk53IPEUBgNrTRWKF4uS7C\/Dob6tzUcW5Q4NLrbk\/jwd9uvatjY+d6RCMsh9DD80ZM0t6I9bJMBocxMDtP1IEhFNNTFhHN6dWeCDVDkShbT3jCE9IZZ5wxenVkhla1qVZDgaWKAo2wnMTu9gqL7LM5lMs86gFzZkErCA+KZ2Hq4rnH2IUDfmHy6ARWIvKAK59chnL4AT+TsNUVPHkzE7fLu47jW1p5yKtTPYQrH578Ab\/L9eKANitfnX1n8C1cfA4bLy91kb+6qaP0yqy3S7x8cxm+leFbP0gDpEcP+YqfTthpp52SE67rrLNO2nHHHZMfWD7++ONTO\/BD0VNtrnWdJSKW+DWUiEivfe1rp5N0s7psSjeY1Y2Yw5WfG8JyBnYQZoz5MsdmoZmribn7eSyrUEzRBMHwMHXXAbwHKi4ikjDm3Jy2dDF2zF9Z0iirjC\/9ypSXlYu8lelbeMbjJ0AiIsGBK3\/hGaeTqz2EtjpJC\/gJtJyeqVpdSwEqP9\/Ctd13L3nBA\/LWDnVVZr1dwpUbsbhN6qWdyrX6Rzt9JB99Jt\/pAn29++67p\/POO6\/66a3DDz88veY1r0nbbLNNW9hll10SQT9V9UUzh49YUjzi\/vrXvz6V8OxnP3uqqjLnyolo\/YpOt3k85xo9ixrUCMshdBZGj2FZoWTwLRwjxsDLYiIiCcfIgTgCgh\/Dx9z5\/UqItBgUnBJKhg8Psy\/j2\/mVSTiUeSsXrroS2AQWEzIcDFF9lAWnE4iXVr1zWuk92i6MAIKDCagnAVfmpQ7CAbxe8irTd2uXPEFEVPcS+a1agfahhzrKg4vW6lDmP5V+\/b3pppuOEUClMKr7X\/WqV03pr47Yg\/ey0MYbb5y+8IUvLAGve93rppJcTVkNBaaMAo2wHBKpMVoCIkNmypgLxlwWQ2hgijksorXPSDDlMG5EJHjtmLdVEMbeq6BUJkiL\/kVElbe6yZ8L1D8iUv5HeGpL\/m7nSgciIhF8hGEG+OL+85\/\/VGY7+ak34SyOK43wiEglrvCcDxd+jucH2gT4QUSMaVdq8w9NI6JaualLRiEw0TNicftz3FS5xtHb3va2JYRQO8EkbP\/9909+ZWWq6nfttddWPxv2iEc8ojIVT1W5c62cv956V\/rwCZePgQ99+7K03\/F\/StwctzSfPp1pfd4IyyH0COZrlULYZbDCAgSRlRK3l6IIA4KBMGTCxMwJlDK9MDjC+HvJdzwceUVEJWhS8S8iUimMUpt\/0gImRObNEnI9xUtKKPLnenN9CxfPD3rJC\/4goL+sKNEajVkDuLmug+S5tKT5y1\/+kuwLP\/axj11amjxp7fzrP+9KS8CIEC3DJq3wJuO+KdAIy75J1nsCQsaqLAuEbikJB0zbvh9hY1UlLcbeLp1VkFUIQYzpt8OZ6jCrMr8n2Q6yMMw0IZjUTzuFAd8Zeskr4w7iWkF7UxUd1U0foTsgrAfJcxhp9KUxUNaB\/1vf+lbyg89rrLFG5X7961+vVsbDKHNMHuN8XHrppRWGt2uf\/vSnV9YCfedFoYsvvriKa\/40FJiLFGiE5QzoVQzSSpLww8AJG\/t9VqvthCUhisFbHak+gcmdCERE9YsWVrFlPhi1+pVhdX9Ey2xJ4NTj6t8Rrb1DuICw1O6IVh4RLVdcPe2wvyOienc1C2b0VJ+pKLveFjS2uv32t7+dNtlkk+Se5bHHHluZpQ888MC09dZbp9\/\/\/vfJz3RxnTp92ctelgjWel6T+X3mmWdW4+Sggw6qHnc\/5phjkn3KH\/\/4x+lpT3ta+s1vftO1+Msvv7xqk\/YOAjnze+5JVT2Mz24wglUlgdNLefAkmOz8W2XcM24bRhCgjjj39ES3XP8qUZ9\/pB0XijzHxR0h4kg3jaao01\/60chZ4Jk3C+o4a6tocGC8NO+IlhBo1xgCysAphQY8aQF\/O5Cv07RWaaAdTq9h8gJWuOqS0+U25O92rnQEuLTaUuI4Teuwk3xyOEGvjHzwx3eO6zevnK4fF62YEgnGMl07xaSMnyw\/2hCUZV\/fcMMN6dWvfnX6+Mc\/nj796U8nJv1DDjkkWb1xff\/whz+srpNMVr3q+VLorLxXWmml9Mtf\/rKq17bbblvtrxKa2mC\/tZsAv+SSS5JxMiigVate7YWH8Qcng3EGX3gvZUoHn5DlHw\/6zb\/s43Z5q2cZfveIwGnVJ5V06+gv88\/penXLcjv51S\/n1w0nx5X4dfpPpK65DlPpNsJyCNQ2IDASgiGDgz2Yh8lEoEV0FpaYdEQkaQkQzJxfHuNVjzmRgLG6VI\/x8DvFq0NeWam3gS1P\/k5pcnhE68i78jFKbZBe\/bnar44ZX1mEq3Zyfee4iP7yyum6uRGR1E29TNBcJsav39SDCwhu8d3yG3YcGhkn+vKlL31pOuuss5Lfqtxoo43Su9\/97nTNNddUQnHXXXdNfuuS621YypWVqHYNu07t8mPp8DYtQe5h9xLHz4qp+7nnnlsJ9DKu9Hv4XTsHhTxWIqLaSzeuShBffke05p2wXsqEl0b+RUTb\/MWXENFf\/uXYKvPJ\/nr95y3KPyJSL\/Uv8099\/st16OaqX862Ex6cHMef8ev1n0hdc55T6c6bysLmclkYLqaXwaoF42VO5XZru4HFFGhgYdiYOM3M6oGgwegx03Z5SEPIwZ8o08R87eEpi7CWnwNLvQxqadVXW7RBenWSnzaUdY+IauILky6ixXB8A2G95gV\/PEB\/bdAeCkBEJPQWLgy9uSaz+kaMrc94+U8kHq0J8NyP6ik\/v2HpZ7l8P\/rRj05PfOITBY+C37Fcd911k71D9R+NmCaPeq6\/\/vrVPqoTs52qERE9CSHjqB1ELO6biKj2TCM6uylFyv\/a5VcPi1iMHxEpojukFCn\/q+fV7jtiMX5EpIjuMIKQs++JbhFR4Ttt+\/YjL07jQXnaNiJGihsHqtxbfyLGwRXfQq3+1ukREVX4bPkzr9+KNvhjKUCY2GNsBxiyAZJTRERlTiMIIsYOFMyGYM35wBFGm58\/f351SjWifXrCRTq4EWNxIsZ+j1cXeSlPfvZPfUuDmUeMrbPwEtRXvaUF8snpSzx+4XAIKN91GC+viN7bpe65XvokIip6EozqkCHTL03hP8KSMqSOEWPp63rGQx7ykPSgBz1oiWsaBL2xR1BSzKaqyhQgFoN25VGQhOs7bgMNBeYSBQYWlv8csaVfeNddaWmBG+6+ey71+xJtwQTtLVphlZHCQcMAS6osnf4LL7wwWfFuvvnmiZAuqWDF\/rvf\/S6tssoqae211y6jGn9DgTlBgYGF5Y0jwuPCf\/87LS2gvbOnx\/uvqZWNVTATKmBWxgCtIsRZCfaf69xMsdVWWyX7hsOAnXbaqTIJr7baaumGhhsVAAAQAElEQVQNb3hDYnKdqVRbsGBBdQL2F7\/4RTrppJPGVPN73\/te+tGPfpRcIXG9ZUxk89FQYA5QYGBhOQfa3jShoEBEVE\/wLbfcctVJOysHwpKQZLokMAv0pdpLwB135RPT9\/\/9omRf8fW77ZMOu+456TcXPjj9bc1XV2HCe4G99torMb8TRO9973urtPX9yZlCbHvj2sSk7Zk9J18d+Nlhhx2SU7GPecxj0vve977UWCFmSo819RgmBRphOUxqzvK8IiLZt7NXmffx7Is1gnLJjl3+37ePBl55079G\/VcV\/tHADp4y2MEwB2OcfM1w\/fXXJ4d\/2sXBFV7mMRV+p2A98u6e5+c+97lEaP7v\/\/5veuc735lOP\/309LCHPWwqqtGU0VBgyinQk7Ck6Trp1sD6qaHB0kuDVVdddXSCrrHicqP+7CFAS8GZw3txf\/WrX1XXQhzoybDeeuulq6++OrWLc4VEeC95Dxtn9dVXT1\/84hcTU70DSu6sfuxjH6sOrw27rCa\/hgIzhQJdhaUTd0wvO++8czr00EMbaGiwVI8BZlLzwbwoJ3C5miz9JU7jn60UaOrdUKBFgXGF5SqrrJJoku3A5Wm\/accc4woAnCOPPDK94x3vqMx5vh0Qef3rX5+++tWvJpq5sLkC3snUfqdFD\/jprWnfr\/8pvfPoP1fuMef8O606f35Fu5WXWabyf+XEq6r43P577rNSess3b0ynnXZVWvFf\/6pwc1x2hUsPVl\/9viM48yrYZ599UgmHHvrJKnz11eelq\/54aVUHeavTgV89L\/3yqmWSOglTrvwf\/F\/\/lb76uVNHw8Vfedv9qrRcbdIeuED9pecXzw9H\/tIqQ5gypeM\/8pc3J\/H8QNpMF20D82+4YaTuq1c08i1ee\/nF5e\/Lr0\/p0C\/+pqKZ8sStNG9edbXi2ENOSuoif6AsOCd95aejNBeWcW656m9VvbRDmDrD9w3voktuSd\/79rmjaXPbLr7639VPfbWmz5J\/rS6XDO0cYk74ySsrtEFAWnl0LqGJaSjQUGAYFOgqLBVAi7aPVQd3wwhJd6u8MmKvAo7J6+1KJyt9P+EJT6iYmdNy9liEzRWwyvAWptdWzrrqznTRwr+nqy65Nt377v+kHZ62crr3f\/5TKQ3L3+c+lf+2f9yRzr7qjnTT7fNa4csvP\/J95wjcke6VcUfCSvoIlw9YfvkHp2X+\/veRtJGOP\/5jabvtnp822mjdyv3Up95ehS+\/\/LLp\/\/39nqoeZ4\/U6ew\/31n5b7xtXgLC5H\/vu++ufgfxqkuuS9895x8jdVCPO9Ntt9xRtePGkTqeNZL+xttiJN\/lK7jo8r8nIP19RuopL3leNdJmLhCmzLNH2sm\/9uoPTDf98fLR\/KUdhRG6oM3\/q9rUKkN7q\/iROH7lVG0f+c51ky+QThxF7R9X35jOGqmvtOgrHqjb2SN1yWHqCEe7\/nj5zSm3s4V3Z\/UtHbwbb4+Ret9Rtd238GWXWda0qCCvIq+88V\/Vd\/6Tw\/N34zYUaCgwuRSYitzHFZadKnHLLbckjyI\/6UlPqu5edcJzMs4hEacrp\/LydKf6TGb4\/Psuk970vNXTZ3daJ7Xb03royD7X8v8Zy1jVRzh3fFgm\/efW5dOd19xRob70pc9Lb33rdol71U3\/TlfdtOxI\/IpV3GT8KeuZBUIZlsvU9nbtzPGDuvOXX2Y0abtyRyMLz\/z7LhZuRXDlve9IX9QFXRUx8mf+SF+OOF3\/11eR80fKmox2d61EE9lQoKHAlFBgYGHphCRB6LQe81Gn2oqDA1eaTnhzJXy7DRcfApmMNv371n+l2y65KV3\/zV+nU444Mx36nevSN864O2150A3ptIvv1bXILOA6IZUCiBAZD18+ZZpO+FMlQOrll3W7ctEp1TJM\/UGZ7hunXysolYK5jK8imz8NBRoKLHUUGFhYuhvmAvWJJ56Y\/GxPJ8oxyzq1B1eaTnizNnxRxaeaof5nRGhedN6N6ZARYXnljcuOrCr\/vagm\/TlWR\/W6C8u5PGBktZT9pUuYlt9lHqVA6uQv006FP9cv1ye79bJPu\/jmJG7DR82vR435hpMDct6+0S4LZt8NNBRoKDA3KDCwsHR5\/XWve131e3pe7Tj++ONT3czKTLvDDjukyy67LDkII83cINtwWjEMpoo5nz7C4Os1YmIsw0qGXjL6+9dMm\/NH9jzLdO38ZV4brj0\/qQO8Mtx3hrK8HDYZbq5Hmff8kfb1urLN\/fHo+a1f4Zs\/QouctlPbyrIaf0OBhgJzlwIDC0sk8UbknnvumW688ca0zTbbVD+k+\/Wvfz1deeWV6bGPfWxac801k6exnIZ9+ctfLsmchgeMMNdeGpiZcsZ9QA\/7Yxm3dDuVN3\/A\/Mq8JyIcCMd2gqvMf1B\/p3znL\/\/\/RrOs03f5f9+e6ivhjJzbSbnIfvXP8eO5OQ28xz\/0fpwGGgo0FJiDFJiQsHRS9sADD0x+W49wLOnjorJfTTj22GPTYYcd1vW4fZlurvs7MfuJtDsLh3ZMft0RBj6sMuWfV1rd6uuAT7f4QeOUn9OW\/hzWzs2KQ7vVdzv8dmHom4XiQ1dabgmUHLdERA8B5skGG2yQdttttwrbNSQv99x8883Vd\/OnoUBDgZlBgQkJS02IiOrx5PPPP7960cMzXF4d8a7oJZdckl7xildUv8MGd65Cr8xyjTaMdibRpKzfeG2qxxMo7dqShcuwBHa7MjqFEahW3w+6428p169sozrV21HPC85pF92czr7gmjFR8u6kOIyXZ5mRX3nxvF0WjgTlU57ylFHhWeLOeX\/TwIYCM5gC84ZVN6de\/\/GPfyQHes4555zkG3gSa1hlNPmMpYBVE2Y+NrTzVxYYdYycBwFQj2sXVuJYRWah0Sn\/jA83+yfLrQuq9RasUBVVD5\/f4eBShbzoz\/zCnJ3buCiqo5Np2RGhFuGEuJPi9vUJylr0UvP511vvSmde+o9xYakhSNPQGUeBCQtLZqNjjjkmPfjBD04rrbRSetrTnpZ22WWXRFN2t\/LFL35xOuSQQyrhOeNaP80VqjPwQavTLp9OTHs84WclaP+un7qUeZb+nMf85ZfJ3kl1Owm0Z6y+zOghpF4q0E2od4tbd43+9yxXXHHF5HHyX\/7yl9Uc8i6sPX97\/xGRIrrDwx72sGRl2ku7ZjLOpdfdnr5xxrXjwqXX3TaTm7E01G2pbeOEhKXTr29605vSq1\/96nTDDTekBz3oQWP2Jv0uIo3ZfsxPfvKTOU\/kciXSrbHthFs3\/H7j2gmnTmWWB2PKcjqtEglhcZ3ykweBCY9\/2NBJIM5fftmORXWL65ioiMj7nf\/91PsXocPxWlUefPDB1U9cPfShD62ehJSz8wC+xwO\/Hem1LGkaaCjQUGDyKDAhYfnlL3+5+vWBLbbYIl111VWJGclP9uTqWm0effTRyZWRr3zlK9XPDeW4mebaY3VYyc9TRUTi+hY+zLq2EyITZebd6vf4kdVOJwFTT7fGisvVg\/r6bidAJ5pnWYEyr3LvscThr9eDEpNpQJDDyUDwZ3+9b5RXhrW7e1mmz\/n0666yyiqJdcaK8ne\/+10iAM0j3+OB0+Yrr7xyv0VOCN+cMDfMkYjJmysTqmSTuKHAkCkwsLBkYvXDr07B+rkej1e3q5vn8DbbbLN07rnnVqbZdji9hk0WnhWy1a\/f5HMdxmPwm266afUbfcLFdyu7V4ZZZ9Td8uw1rjSZYu7d0tWFSDfcHFfWuR8BldPPVPcBI6bhLEDHq2On\/s2CtB7f6ZrKeOWI994wQfnsZz\/b54wDc8GcGHSuzLgGNRVqKNAjBQYWlnPpbVg\/XvulL30pfeQjH0mHH3542m677ZIVsW\/hP\/3pT3siZylYekoww5By\/QmBXgVrTpOb0k5gW9nl+Kl0uwn3XI9e2plxyrZ2yrtUXnIZ\/bjz589P+++\/f\/Lgh3SU0lNOOSV59AN4uF+YuOmAYc2V6ah7U2ZDgYlQYGBhmU\/xeffVqddOlRAHx96MNJ3wpitc3Y477rjqcMUrX\/nK6kCFukRE8s2UTIDCEz5MmMgKRD3mj6yOuBMBL\/iUK6x2wq6efxYeOXxsPVL1XFyOK91S2JThw\/TXV3nyrtdPWDcYpJ7SzO+yb9qtvHZxTpa\/9a1vTSussELaZJNNqkc\/PPzhAJ0wq8+\/\/vWv7ZJOWpg5MF1zZaKN6vW0LbxBypKul9O88GZi\/oPUaWlLM7Cw9M6r915n+9uwTu0yEa+11lrJAaVyANgLWnvttSfdhDzsldf8NtcivHmqbb0IQ3jDAAJkGPn0m8dElZBcXr3+U0W7v\/\/97+llL3tZdYqcgklYegUL8AuzBeIOM9xc38l2Z8JcGbSNf\/vnXeOetHUa99Lrbh+oiF7zP\/OyfwyUv0TqNx5MJH9lNNCZAgMLS4d2mIpMVtdDmIjsZ5RFzYa3YWnwNHSHKu53v7FH\/\/202GqrrZY8tOC0b9m20p9PS7Y7AFLiZX9m5jldDp8Mt87gn9HhgfC6YMh16dQmK7icZn5xH1HYoM\/35TK7ufMXraaVU+KVdSjD+UsalH5xvYJy57dRQuqrbPm1CxPeKxxxxBGJ6d8pcy\/8\/OxnP0tf+MIXKuAXJg4O3F7zLfEG8Q9jrgxSbpOmocBMoMDAwlLlHYaZ7W\/DYjyYADNxRGjWKEREEu6VFddgRiNqHi+8YN6dBEsNfYnPQdPVM1KHdmEYfQ4v99qctLz8ssty1LhumX83gVCWJ9MHLN+61lGmFz4M0H8f\/vCHE7eeXy4vu\/X4Tt8Zf\/6iesObXygEvkvoRosSrxe\/\/UiKJ6vNJz7xifRf\/\/VfSyQTJg7Od77znfTPf\/5zCZzJCEDjic6VyahXzvMfN\/4l\/fo7hyZuDptNrnrP5vqj9Vxog3a0gwkJS3fBHCFfmt+GXXbV9dKyD1kv3X3Ltemss8+u4JRTTkmngFNPrVzhp4z4b77+L1UfMKW89Wt\/TEyj\/1mU7uyRtFUa6QoQLj0Q7\/vss85Kl156abr7jluq\/PyRzymL0okXBu7zz8s5o3Dpeb+q6gT31JE6\/e1vrTdI1V8YGEUe8dx1zTmj+AsXLhwJSSn\/5mNO87e\/\/q0K9+fuW1t0qPwjbZNfrucVVywczUu4NgFtqr5H2sV\/1oibw7UT7TLIV1vvuvqcBPfUn\/+8Oph12aUtoX\/g\/y5MB35\/YYJz2UhbpYeX0+X2y3\/+olVqjuOCU085terHm669wmcFj1npnjF1l7+Ikhb3FP0hbhC4ZdGPqnuowH55pzzEwfnTn\/6UCLBOeDMt3JjT13U497enpasu\/M24cMpI35wyMs7b4f\/5j2emX3\/3s+mCX5wwmg+8TvjtyptO\/CXr\/5s0VfVBo3b0qIfl+nTCr7ehxJemhMxPZtoY7VSfCQlLmUYsvW\/DLliwID1n81ciQ\/rjjz6fXviqV1WwzS67pC233DK9bLvtqsMZT3\/xiyv\/K7bfWkvaYwAAEABJREFUKl1534ckKxECU8K7RwTKM1\/5yvSi17++wrUnVYLwnK\/wbd74xrTtO96RXn7AAel5e+2WLvyvNWVTCWvxyt18zz3Tuf+8JREoyv7hX1v7MBj8e9\/40qoceDu\/9a3ppD9fkG69\/eZ0wbf2Gw3\/w7z7VnnCf97mm1fh8n7ze\/ZJN977AZWQh\/CnM75TxX325yf7rOCLH9kjKfP80z6bzjv8TUn63T6+d7rul59On33rJhW+MPlpN9hu332r8KdutFFFB648tP2J22xT0Q4N0POEU7+ebvvd4RW+dG\/ce++q3OOvuT7d9v+WG63bHRf\/MO2w9WYVneBdc9MVieBXLlDuD0bSSKydZ559XLrl9r9XQnaPXV9VtUHZF112ehX2mU+9p2pLrns7Wmz8pv+u+uOdmy+QbQMFBcyVjTfeOO27qK\/1QQk7b\/uidNxHth8X4En3spe8YAnck77w7uTfGSccMhrXDb9deZON\/87XbVmNXW2ol9+u\/t3w6+l9Tzb+R3fbumP9lV9vQ4mvzSUcfvjhumvWwISFZbuWelHEqrNd3EwLc7GaWctJPyd3y\/r5Fi4eXhnHjwF88SO7p2+\/YZX0w8+\/K33\/pz9dAk4++eQEctyRb354hS8N+M7b1xuNh1eHnI4r7tgf\/zhlEJbzK\/MR\/qV9Nkk5jF9ZQB4ADveoYz6bjnjTw9OPjj54tB6ff+s6o3WEk+E7x34u5fLk9X8f37FK8\/UvvL\/CP+5VaTSfE444KP30e8dU8eqLPjmfuis+h+V6+c7+0lXfnJd0OU4d2tUNDkCL\/\/3AplV95A2+vN8WVb21Rdu4QFyGbx6wfYWT6ZPD4dfLUxdh2YxrjPQLxtnjHve45Pm7hQsXdkwuDg5caToiDjFCOeaCOWFulFn7Fi4eXhnHb6589atfHUP\/TMvGPXmppMtrX\/taQ2PWQCdh2XMDvA37zW9+M9k\/ude97pVWWWWV6hqGvb6nP\/3p6ecjZjITqecMpxjR5H7gAx+Y\/vKXvyyx92MvSLh4eO2qhjE+60mPThtvvHFPALeEjXtM1wkv59UpPofDy\/5e3E74wjOU+Qgrv6fDrw4Zei0ffq+4dTxpM+Q43+3GSa9hyy23XHItxDORO+ywQ3JIrp5WmDg4zg1IU8eZjG9zwFwwJ8yNsgzfwsXDK+Oyn8DMdGrcjXviFxtPkD\/M5PTGQx4bs8GdkLB0Enbbbbet7iNedNFFyZF2p0cB\/69+9au00Yhpba+99kr1k7IzhTjz589P6667blJ\/z\/WV9fItXDy8Mq7xNxSYLAq43+uaiKfs1lxzzUQAOa0N+IWJg+NE+mTVo56vOWAumBPmRhnvW7h4eGVc428oMBcoMCFh6XUbJ\/fWW2+99Pvf\/756+\/XPf\/5zAkwywsQddNBB6Vvf+taMpJcVsMve1113XfJD1XkVzGU2Es5cAG\/GNaCp0JykgC2MT33qU+nTn\/50cvfXKVRvLwN+YeLgwJ0qIpgDzVyZKmo35cw0CgwsLB1x9\/QVLfeEE06oVmcRMdq+iKjCxME54ogjkisYowgzyMOUtfPOO6d99tmn+gWVo446qnI\/9KEPJeHPfe5zZ1Btm6osDRTwRqwXfChrN954Y7r66qsr4BcmDs5U06KZK1NN8aa8mUKBgYXlLYuOuD\/zmc9MCxYs6NgecXDOP\/\/8tnfhOiacwgjaOS193333TT\/84Q\/Ta17zmso94IADknDxqfnXUGAwCkwoVUSkFVdcsfrprlVXXbXyRyxWSieU+QCJzQVzopkrAxCvSTKrKTCwsDRpvHjD3Mpk2YkK4uDAlaYT3nSH09I\/8IEPVAJdnZm77LUKn+66NeU3FJhJFDAnmrkyk3qkqctUUGBgYUnbff7zn5+8IHL66ad3rKtNf0fD4UrTEbGJaCjQUKChwHRRoCm3ocA4FBhYWEZE2m+\/\/dKLXvSi6pLqBz\/4wWpPxapMmfY0jznmmCpupZVWSg7JeGP1mmuuScDpOddO4M5GuOCCC5LfHIyI6pdK+IVNV1vcubNyj2jVJ2Kx+7CHPayieVk3dVXniBYev7ASJ\/uFi48YHzen6dX9wx\/+UB1isf\/dLs3tt9+evBLl7l5EJK5v4XV8YeLgRAwPt15O8z0zKIDXfPSjH03ecP7d7363RKX6GQ9LJJ6kADwvX7WLiOROure1L7744lT\/NxPr77Wo3XffPbEURkTl+hY+G+pfr2M\/3wMLSwLPMXGnYV0LYZbJV0YiovppIY89E5DnnXdeWn\/99dNDHvKQUXjKU56SCMx+KjtTcP2m4IYbbpiuuOKKak\/THo5nx4SJm456qgtztx+tdqWgBPf2mM5yvdRRXaVRd9Cp\/v3g5vx7dV098kPCDq20S2Ncie\/lh4YnC7ddvZqwmUEBY9MhvHa16Wc8tEs\/GWGEu5sBrgbhhX5k\/t3vfnf68Y9\/nJ73vOcl92ZzuTOx\/uarX8PBL7jqz83f4ie5\/jn7aXEHFpY0Ive+HvrQh6ZBQFp5TEurJ1CoFTPm7b7bqaeemt72trdVcMYZZ1T34d7\/\/vcnGuEEihgoqdWfOnlg+wuLfqEiu\/vvv3\/Kd9\/6qX8\/uP1W2i+9\/Pd\/\/3f16xqd0lptup70kY98JHkaa7vttktHH3109RascL+6kdNOFm7Ov3FnFgUwZmcKOv3AQT\/jYapaxvrzvve9L1FkTzrppGQ8+xEANwVctzN3CVT1mYn197Nw5lw5H81L38LFqzuYifVXr4nAwMLSbz26GO2XKwYBaeUxkcpPR1qrZILRYwxO+uY68AuzP3vOOefk4ClxXclRpteT3MHrVmg\/9e8Ht1uZZRxm4GemPAJuv\/vxj398GT3qt0ru9YeGJwt3tDJT6NGWHXfcMREEd9xxxxSWPHuKMoY+\/\/nPJ2N+s802W6LiaNjr2Fki8SQFqPNXvvKVasth7733TrZMclEveMEL0pOe9KRkS8Itg5lYf3U988wz0\/LLL5\/UNyIEVVtQvoXnsysztf5VhSfwZ2BhOYEyZ2XSXOlzzz033XnnndXgzmHZ3WCDDao4ODlsKlwrQG+FEtj2ErqVqW691r8f3G5llnHM8jvttFO1h+oOLlN9GZ\/9N998c1L+WmutVTGYHM6lZK299tpVPDwwGbjKmmq46aabEkXSvUrPR051+bOhPEzbXiWhYy+9Xud+xkM97WR961dmYzyCJa4sxx67NlG0PRU4E+uvvo985COTlbxnDX1nuOGGGypr2uqrr14FzdT6V5WbwJ+hC0uDgi37Pe95T7KCYHufQP1mXFKraPt\/9QGvolZ24miIvqcKDFZCyBODLqtjshGRvNfr5SRaba5LP\/XvBzfnP56rji996Uur5wW32GKLSjNtl8aBAaZa5vq6AuBAh\/1xbdb2ycJtV6\/JDrM1MVVvvU52WyYjf4rhe9\/73oRxv+ENb2g7fvoZD5NRx3Z5GqfGq7Mbv\/71r5N3syOiOuBjD5MZNi36NxPrr2rm6yqrrJL22GOPalWPr1BSfd\/\/\/vdPW2+9NbTqJ+N6nbtVglnyp29hmU9z0fif9axnJWaD3Nb\/+7\/\/S5jb9ttvX+0reflmnXXWqQibcWa7a1BHRPUObqr9IwgiItlPqUVN6ufChQuTwel3RR3a8UyfvRB7pwawE6IGtkr0U\/9+cOXdC\/gdxk9+8pMpa6Gd0rjniml4Yi2iZfLJuBFjf5R7Me4ySzDPiMFxc3lT6To5bi\/XiUn7snNN2ZwoLZlf7f05KNPpKlo\/42Gi9ek1vRWZ7RL7ehtvvHFiBXJbYNddd00sLM94xjOSbQ\/5zcT6qxdeftppp1UnYJ\/4xCdWPPAJT3hC5dqaEgZvptZf3SYCfQlLE\/ctb3lL9XD6pZdemqw8aHoqcMkll1Qb1wbFYx\/72IRZGxTwnMbEeOE1MHwKZJPdxz72sZQPDni6zy++0MAdKvCo\/fBLbnIcNgWYsJwS91qPlRMrgZOTlNB2QGGFP+x6zMT8WGwcVkMXr4LNxDqOVycCEW90UM0ZB2\/8EppMmw7K2O8bL4\/piqeIW11eeOGFiQXLvjDXSXpKufjpqttUlNuXsDzllFOSE5a0X4QiIE1qFWV6dZ3EapL2gVkzwzKZ\/PGPf6weL4DXwPAp4JcnHAZxSjdi8SrM\/UpXeuxR2g8ZfslNjsOmAGuA1YdH03Pe5pXvdkBhZe3JuLPBHaSO6OKaBcsE3hKxeJwPkt90pbE14s55xOL6u6vu0Rb8VX9OV926lYv+FkoEJeFOyBOQXLzFz8aJh9ctn9kc17OwZMajDXEddUaofKLLspsJFiFcpWC\/5o+ItNNOOyV2bicf\/ead8NkMzIdocPfddy\/RDGHi4CwROU0Bj3rUo5J9VC8pqYK6qaO6+i5BmDg4wrm+hfsuQZg4OGX4sPwOPTjsQNNWTpmvb+Hi4QF+YeKGhVvmM1V+h5cc9mAt6AXgSjNV9ZuuclxR8G6z1dd47e1nPEx1ewh7e+5lueancNsOeOlMrP\/Cka0ep13d4\/aIQln\/pz3tacmpZFfpPK4wE+tf1ndQf8\/CknnIY+js1k9+8pPHlEcbwowJxcc97nFj4gxsJxct1Q2GMZGz8IMpjPZks75efWHi4NTjJvvb3nG7FYYVJ\/N5VmzUTR3VtV4nYeLgiOP6Fu67BGHi4JThw\/ITfu6NMk\/VlSzfwsXDA\/zCxJV18C1cPDzAL0xcN9wybqr8DviYMyw2vQBcaaaqftNRjq0eq23K0JZbblntS0dE5Vpt2vrBkwghL\/nMxD4mDAmRXug3M+o\/tqZobM+VVXG55ZYbE+lcgUdm8B99NBPrP6bCA370LCwRyt6Ie3x1YhGimLWTXvZXBqzLrEjm1SL7SO32AIWJgzNVjdEvND2Kintn9XKFGcCbbLJJFaVu6qiuVUDxR5g4OIK5voX7LkGYODhl+LD8HlGQNyXMuCvz9S1cPDzAL0zcsHDLfKbL77DYj370o+QXcAgGSqv+pOljTtNVr6ksF7\/Zd999k62fOrh6hFnby2T5evjDH576GQ9T1Q5CxmGYdmOUMmC\/D281j2di\/Ql6QtDPw1GSS7qx5jgVS2nTFzOx\/mV9B\/X3LCy7FeAotPj11lsvGdj8GRDSpEZEp0Vz+Gx1XaJ37Jumi2Hldti\/dQqVUEKHHD7ZLnozgdD8nBS0isxlqt\/BBx+cnFzO99H6qX8\/uLnMYbnGS68\/NDxZuMNqyyD5sMI4PIHxvPCFL0zvete7kr0ijIpi+opXvCLZIyr7e5ByZkMa\/bvxxhtXVxNs\/5RASaK0eS7OqpPVAH6vY2eq2m+eOujIIkOwl\/3mUJ67tVtttVX1HOhMrD8Lkj7wNN+JJ544hmy2AoRttNFGybbPTKz\/mAoP+NGzsFxhhRUSrQ0Dpu3m8tjYHRuOiOrR9ByeXadgPcUmbf2+XMapuTP6Uxto+a5qEJo2uIFTiRiZB+XtQUxlI1zVcbDqi1\/8YrLKPOqoo5I3MyTemQcAABAASURBVGmyDoU4JZvvhfZT\/35wJ6O9m2++edp5552TAx1WENrF1Tbh2pzLnSzcnP9UuuaXNzcPOeSQapVEMC5YsGC0Cnm\/+HOf+1xyj3Y0ovGMUqCf8TCaaJI97lN66q6cp854EKKPeMQjqvuLEVHVYqbVnwAk5NXTeHRIyYJB\/Z1MFu8wYeZ9M63+FVEn+KdnYYlx0iyYCzAtK0ZlMxHZJ7AvaSUiLAMcphGCxSawPYUcN5tdT7U56YuBeegbuKLhFLC4qW6bA1XeYiS0Ha9\/zWtekwxcg5gZFuMt66SOvda\/H9yyjGH47bN6pJkJzuEO7eJSVoSLz+XwCxs2bs5\/Kl3WAG9temfYIZ9jjz22+oWbXAf3C60y7Wk6bMcUn+Mat0WBfsZDK8Xk\/1WnQw89NH3mM5+pnrYznik8BI9rXk6v51rA7XU85zST7aofvuEeMCWNkM\/1P\/vss5N941yHmVj\/XLdB3Z6FpQKYNtjemYSYG31b1RCKNH2b2PCAwxM0EafXpLEiED5XwAVcK2ptBwb7Yx7zmGlrHo2O2c7BFfVh+nZCuVOd+ql\/W9whtdVYUl+aaDviaRfBz4IBj+vdVOF1fGGTgVsvZzK\/tU+\/MWl1aqfyafjmoGtZVqLClkYwfvAab6vW29\/PeKinnaxvQoT5PM9T17qsNO1V1sucifV3+l197bOaj7n+wmdD\/et17Oe7L2HpsQFaLuHnmHD+ea5ddtml+uWNXLB7YZg08xmTkYMJvnN84zYUaCjQngJWiQ4p0eJZc9pjpeokKNOXwz7mWGr+NRRoKDCpFOhLWKrJc57znOpHnglLzzS5NnLYYYeNeUXf\/iazpEeD4e25557V5Ja+gYYCDQU6U8DKg5C0WiIIO2GKgwNXmg54TXBDgYYCQ6JA38JSuSan05VOb+WDI8IzmMA\/+clPEjMlvIjWpnWOb9yGAg0F2lPA6VcnPCmi3\/3ud9sjjYSeddZZ1W+BwpVmJKj531CgocAkUmAgYTmJ9WmybiiwVFOAafXNb35z4jp1uPvuuycPergahDCuHHgNy\/UJ10sctoArroFZSoGm2rOCAo2wnBXd1FRyaaKAU8wOUbjk7USku2vOB9jysJfpJ85cCXIq2P3CpYk2TVsbCkwXBRphOV2Ub8ptKNCBAhGRdthhh+o3P60yy1exvKSy4447Jqdg3\/72tzdnATrQsAluKDBkCqRGWA6bok1+DQWGRAEPebiXl68ZOKrvzrKXotxrHlIxTTYNBRoK9ECBRlj2QKQGpaHAdFHAHiVh6fEPP+HkWok7tNNVn6bchgJLKwUaYTkLer6p4tJHAYd3PCXmTVEXvr0P6xECD3+4mnXggQcmzysufZRpWtxQYHoo0AjL6aF7U2pDgY4U8J6yt4Y9iwbJM5JOvdrH9HIPIfnOd74zecZwaX69B20aaCgwVRRohOVUUbopZ45TYHjN+9KXvpT85JEnIj1\/x\/\/lL385HX744cmv2\/iZJILSO7ne5hxeyU1ODQUaCnSiQCMsO1GmCW8oMA0UIBy9DevHdD2M7zcE69VYeeWV0xe+8IUEx5USaeo4zXdDgYYCw6VAIyyHS89Jzc1PbUVEdV0gYnzXnTzv9PpVA7\/44tdhJrWCQ8r8pptuSn7t5H3ve19yArSfbHNal\/n7TdtPOZOFm9+GXWeddZJfF+lUjjg4DvxI0wmv13C\/WhPRfUx5mWvTTTdNfjRgNtIWLWYDTEdfOGXtKhKrxWTT6Mgjj0z24VlJcll+Hcl4johEGfQGebsx5n6xp1Q78YbJnP\/zcmUbd+ZTwE9xeV6wBPfu1HzevHlptdVWS2XcGmuskVxsFz9bwAT5n\/\/5n2S19IY3vKFSDPqpOyHy4Q9\/OLnUj+n0k3Ym4DrQg1l499X7r53qhE7i4UrTCa\/fcD+S4B6n310swX6p8XfSSSclB42+\/e1v95t1g98nBaaqL4w1r0WxUkz2o\/x+QnCPPfZIfiFprbXWqihij97PfRnHBKnx5efL6vPXmGdRsWdvDz8iqvTln8mc\/42wLCk9w\/1+3cUrLiXYx1JtB0JcVC\/jfvGLX1RaGhwTot3PGEk7k+CXv\/xlZWKk5RL8g9Rt4403rn4E249E0zQHyWO60lB+Xve61yXvwvrNQAyiXV1OP\/30lJ+9k6YdziBhflnIwSJMqQRjyNjyk3sY6nvf+940FauQQdowV9JMVV8YYxSvyaaba1D77bdfcqJ7p512Gi3OtoNVpnG33Xbbpc9+9rPJobbPf\/7zqbSaXHrppcnePUHrDvJoBjXPZM3\/RljWCN18Th8FaIyecGM+9qTboDXx0D\/Nk9n5uOOOGzSbKUmnzT\/4wQ8SrT6DFRyT1LbbbpuY0MXTvpnU3bV0pWSTTTZJfsgAzlTdu2SlsNr0c3sXXXRROu+886aERk0hS1JgNvaFsUsBpPSziORWUfy8UuWkt7AHPvCByfin\/OfT3gT6IYccksSVghZ+HSZr\/jfCsk7pOfiN4ZZ7lpgugSR84cKFacstt6zMtRGRHve4x6VTTz21osJpp52Wnv70p1emUJPzla98ZcK0q8jiD2ZNO8y4EZEWLFiQjjnmmCSuQO3qtW\/h8r2TnrTPEtm9Qz\/0Kzwiqjopw31Dq+YSl\/9pT3taWn\/99dPXvva1lCec8JkGN998c8I8ttlmm5TBKdgsiI444oj0ohe9qDKvYyiEJA2cln700Ucnv+pj33Kq2sVU9qAHPajaSyboy3L1db\/j4IILLkgveclL0r3uda+qT+2L2m\/W32Xe\/O1wCe9yTF5++eXVfpix4TEH6Uq48cYbkxXbqquumi677LLRKHOCNcMqPSKq+jANlnlnZPPG\/DnllFOqvfWIqLZAKDERMea3fXMartPLtkv8+DPmL2wi0K0v9I1xwtQZ0ZovaIzW6JjLZep0b9ecZzkwxrQNPTIOxYiAkj4iUrc+ymnqrpXrV77ylcrSZTzX48vviKh+SEAaVgxxfmVHer+NXApace1gMub\/vHYFLcVhS1XTmTydqPRTavYIPOBtT8GKxT0+5gxmEKs0Zo9vfvObya9dlMIH0ybETMLf\/OY3aYsttqjeNfUrGZj+m970pgSnF8ISrnAJhBJfeQSoVafJjEHaYxGunlZXmEOZBtN7wQtekNRJ+8q4meS\/z33ukzBlbRoEpJXHVLWJcvX73\/++0vAx4lyufut3HFhlMLcRsE94whOScbbKKqskj8c\/+clPTldccUWVPcHyta99rTLNwX3GM56R0MpBD9ds5KGfIROSz3\/+86u0LAvCSqCQXXjhhcnYtqcvThjm+olPfKI6VKUeyvjGN75RlZnzhpuBEH3xi1+cHIyxn2slZOxb+fz0pz9NhHLG5WrD97\/\/\/UopNV4jQvCEoFNfmBebb775qNDWHrDmmmsm9Ntwww3T2WefXZVtq2PHHXdM9kcJQ3O2HFPws9JJych0r\/dRlVmXP345hyIsL3O4C2qliBGUfk2HcsH\/8Y9\/PD360Y9Om222Wbeko3GTMf8bYTlK3qXPQ7O2qsSU7Ek55bjrrrsmqx2D8+tf\/3qi0dknwHjsi2IcZ5555iixaKaYjIl4\/vnnV3tt8jI5nvvc51YHbaQfTdDB40APoWYimRQlGiaDAdknUx97aU7L2efYYIMNktUEKNPw26PFpE488USfMxL8FuX+++9f7dNqV78grTwmu3GUplNGVlJWv8YHZYWwyuX2Ow7012677ZZcjfnxj3+cjCnjhCDDkI0fV2fkT5g5FAKXtUM90MlY8E6uVShhYIUdEYkFJCKS3wTV\/\/LI8L3vfa9ixsY9ZnzrrbcmK1mnLD\/wgQ8k5auHMpjFy7xzHlwrHuPLvDDe\/X7vxiN75RQ9efz2t7+FNgpXX311Mo6lIdxHIwbwjNcXrBHmCzqqi\/YAfnvO+k\/bFL3eeuslK1CCcJURRcW8z2MKTdBGeeiM3pnuTubrI6vkuqIq3zpIS7EgqK2Iy3iKCfrYkxSuH+E\/8YlPrJQXfIHQtqp0gAdOL4DW+n9Y878Rlr1QfY7iLL\/88snKL69MIiJ5Vk1zDTTXBCJaGrB9NMLS4MuTg2vyCHMClXYtLYD\/yU9+MmHkVgW0XeGd4Nprr63MYlYr0pR4tHjf2QSbfIyAiWMiEdL20UaCxvxnTtJGZqd2ptoxyM1HRQHmOGa5iKjMohEt1xghCC6++OJkNU9BImwkGmQcWGUwkzrxTKmSD7DfhCnav6LMEVYYNQYvHGOFByIisYgQ3CwihK5wTNZD8yeffHLChIUBqz1ChDKmLcJYVyiJVpYEsvKFA9YM80PeP\/vZzwSNAYdRjPMciB4sL+YDwcjNcYQqpdRefJkmx7dzB+kLFh1zguB74xvfmMr2RLS2WZSV5xR\/J3DIjEKKvugcERVqRFSnWfEK\/UiZqSK6\/Pn1r39dxbZTFKzQzXsHd4466qhEYTfO1J\/FgkVJv1tVqre+i4hkbh900EEdLVfDnv+NsKy6cOn8w\/RiJdeu9YQPU0YZR7Mvv+1rMMm5ssKEW8bxG6z2QAmrhSN7o8I6AcZpdSkv+6slHkbGHGMyWdlYxYwnfKXXNm20irGCEDZbAJOy0tlqq60Sc2EnoMDQxIfVLvTacccdKzOnlclGG21UZW3PmumN0mFVQXhWESN\/BhkHDnWMJK3MoREtJuwbsFJov5Whco0f5T3nOc8RPQYIKGNCIOHIpVS1M8Va7VldEc5538uJcUKNCbUuxCIi5TKtHOWdISISgZ6\/s2usGsOEcu4X+WsLJZBgyLjjuYP0BQFC4OgTq0Z1sEr+7Gc\/m4wlJtbxyhXP9CkdP\/qiM38G\/WG\/\/M4770yEcw5v51qZWjVKQ4jXcdy5JJitOAll+fn2W60UBkqQ\/WR74iwIlGO\/xsMyQYHKFoh6vsOe\/42wrFN4Kfp++MMfnqwiBm0y4Ubzx0CZSK0SSjDAaYK33HJLcnigWzlWJ5iKSRkxlnli2MxDBCbzEcaG8Vgh0CzVoVveJj6zWTecmRRHGaCsfPCDH6zM2sxhnQBdMZFh1Z85jlmOxQBgmJiVcWLlxRxWL6vDOEh5LNTHAcXFKgtjz0Krnmf+Ni4wfnuBDuXk8NKVh3Fj9amvxREK9uAIKeMKWO2JIzQiWmPM+BRmhZTrW7pMmhFRPTOo3nABxl9XJoUvWLAgEdQOxeR9QYoiGhLSxiy8XmCQvpCv1Zh5oc8oDlZiVmvMkQQwnPGAgLMSj4hkZV\/SJPuNDflQmLmdQJ\/gERGRzOHU5t+jHvWoSujqJ4qzMxDSMAmb7xuPmLh\/9atfJUqLQ31vfvObk\/vYzOkO\/qhrm2yrIOUPY\/7Pq3Jr\/jQUmAAFMCn7Nh4CqANGMYGsq6QRkTBqex6Yl8lv0mFI73jHO1J5GKRKMIv\/YMgYHYanbcxRTImdwF4fYTGZTabhE6AYzs4771wdmmpX3mSPg3ZllmH3vve9qwM0wigbzLFWm2hnhWVxZIT9AAAIIElEQVS1Z3sBwCnBCqY+dn27y4qBl7jd\/BGtPVM4hLO0TJDqwERLqIsbFMbrC+Xttddeydhxatn2iPIJkzvuuKPan+ynbPlpB1rUgQLQT1794qK9MxLmPnOyN5KtPikR8kJLBxTxGMqXsMmERlhOJnXneN40a6ZZJlirOxOrEzid140cNPWISLRAebTDZSZjprF\/xPzjsMd6662XHDSg\/bZLI8ykIlz5ZzpYhTMbv\/zlL08f\/ehHk0M0VlSdgKBkqpzsdjk0wxxHIDrUUT720O84YGZnqrS\/Roh1q7txoe0UJSvMdrgYpXEDNyIqFGPF\/qA4+4VWeZQrZlD1rZBG\/jABjjjJyVzjrhNYRbkyAXc8oLzZvyecrZKYFJnRmczHS9tLfLe+MHYcfiMorcL22Wef6nqL\/f2IFm3GlNHhg1CyCkVTAqsTXYRTlDtkUwWbf\/ocLoWrChznj3FhVel0LoEInQIZER1Xp6nDP+UPY\/43wrIDgZvg8SmAUTtIYQ8BUxo\/RWcMzBMTw1yYXzImoehFGwzTYYwcTkDY9PdGpDBMglsCbZ5GzdzcK6Mr00+HnwatrpiLST4ddWhXpnrZQ8VArWZd2ch4\/Y6DiKju70pPCGGi\/BmYdZ1yxiT5rRKZYylJGSe7hKSVj2\/43AwEI3M9QegUrDY4lJLjufYYufKu10P4IIBGTL2EM4FpD45Z0R7aIPnV02hHp75AL4ormpkzZVrt81xhGdbJb+yhJ7pbmXbC6yWc4LW\/Ky8H+XpJ4znFG264obIoqYs02q0NpcC1BYEXZBx4GYY9\/xthmSnbuH1TAFNnlrPacCGb9l9m4hAHAUjLdmCjjKv7bfw72IHB0CpzvH0n5jSTDIM24XIcTdP+nm+Ck1sCAW71giESPmXcTPVbATAzO4hSrt5mQn0x4D333LOqij1kp0R9DDIOnLTGzO2LEibyAZih+7YYtBOSxoUL8YSelYYxBQ\/A9Zao+7\/2AglH4RkwaH1v3xKeRzPqpzFdZSAUDjvssCQfeeb0xprxjRnvt99+Obgnl3DE3NUZ0x\/W3cpceKe+MN9Yeyg0uX+k0S7t007f7cAWgLmc41g39BGFtKS7eHPd4SfzU38J6wb6Qbx9R243sD9vK8K+s3Zm3HXXXbd6\/i63i6Lkiom7opTtjJfdYc\/\/aRSWuUmNO5spwCTkBRp7QxgbjdoBAAwIIzL5MByMq1s7TXLMzHuj9ulK3O233z45HMHc44StF1SsNgnhr33ta1UcZlSm4T\/llFOq6w+Ej+\/ZABGR7Dmhm5N\/7VbM09WOiKgO7ehbK3YChMKiPv2OA4LQPqhVkHHiAIdx49K6wyjMzwSNFQNT+8EHH5za4RKkhIO8sklVfQATIlMs0zawZ0iwi8tAOXFS1AEiY8iYMra0x0rQ\/URhTgjnNL24mDwlz5NtBDToJV2vOBHt+4K51xykJFIgHYDRHvTUPtsYFA+ChHBUHkWS9YUANOY8LkFR00eemGPp0Uf6XR\/J31w3583L8V7kUYZ+RWMmcUqIsHZAqFOglMncH7HYdMzaYM\/2\/e9\/f3UvmRBn4lZnq\/l6fsOe\/42wrFO4+e6LArRnE8o+iRNtBq+DAE7I0fRpklYjEYsHfacCaJLyY1YzaTKe\/SdmGff7IiJ5LMEJOJqlO1jKhJPxuZg5E5if+jLJhc0WwKS9CmPVbLUdEZXQj1jSpTx02subjPYSLk4+2wNi3vRurXL0W7\/jwMrFs36YrSscxo39Z8LSSkbb5B0Raccdd6zeojWmxGVcipg8MFG4dfCKk1UHRo2x1+N922O0pykvp2ONLaswY8o1GeXlusDvBaQlqOFyffMPE9r1BeXCHVjzghLB\/EyhtF1iLoojBEsLTkSkvffeOxFGcCge7riqq7um7lGiuzmN7uabuU55NS97aRthrS9YDFwjkXc7EEdBcS1EPUscio65r57uYLrKRKFi1Srx+Cdj\/jfCEmVnMTg4Q7DQogymdk0xqGlq+RQg0wrNsl2anJ808iqBxqksOGU4M5WLy\/YuxQN7jYQe80tElOgd\/ZgWTdgpOCfcSkQTH+Oy8pQ\/gGMVRjMucfndqaPVY7K9TGZpZgq4N0Zr1sapqpM+VV67MVGvg6P89ooAf44fZBxYgbmOIi\/lW+0Quu1WCnCNKWMr42Le9RVlrg\/XiokAtFKyQhTWDuQhL+XLGxhfFDRjr0xjbpTzqYwr\/VZQVnEETRk+nn+ifWFFbV7Yv9QOtEVjc5GAZaKlYFHEcl0IP+Zw+FalpYJZpzscc51Cp6ycRzeXEHfdQ96uf3XCJSDtNeI17XCsaCky6iAv15IoanXcyZj\/jbCsU7n5njYKGPQeHrDHY3IPWhFMyqVlyoETnIPmMx3pMDhvkmKy7sVZPWMMnYDSQ\/mZjro2ZXamAEWNOdHerP3UzphLT4wVIWWYFYrSO1ktn6z53wjLyeqxJt+BKOAaiifGCLtBJ5RTs043MhfSpAeqyDQlchncAScrNs97Wa2NX5UGYyZQwGqTadkq1V1ATNvK1KpqJtRvuutAGXaKl4l0Mn86b7LmfyMsp3sENeWPoUBEJHfDCAnmOCuqMQjjfHgGz8seTET2wsZBn3HRjtm7ijHjKtZUaFwK2A6wynf9iZLmJ9Qc8hk34VKEQIkwP+01jndCfhCyTOb8b4TlID3SpJlUChAW9kRMqoje9jtzhexPehjbEfnZqNE7FWyPy+sok8FMMp0ad3IokBU81gGWgbKUxt+igNPz9pHtT7ZChvd3Muf\/\/wcAAP\/\/h+R\/fgAAAAZJREFUAwChU0v7FK04vQAAAABJRU5ErkJggg==","height":186,"width":447}}
%---
%[output:90b9d771]
%   data: {"dataType":"text","outputData":{"text":"\nFigure saved: MATLAB_Validation_Figure.png\n","truncated":false}}
%---
%[output:6eee28ad]
%   data: {"dataType":"text","outputData":{"text":"Script complete.\n","truncated":false}}
%---
