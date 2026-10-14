%% Live SSVEP FBCCA -> ROS 2 /ssvep_command Publisher
% Drop-in replacement for read_ssvep_ros2.m. Identical LSL/ROS plumbing,
% channels, voting, cooldown and class mapping -- only the classifier core
% is changed from standard CCA to Filter Bank CCA (Chen et al., 2015,
% J. Neural Eng. 12(4):046008, https://doi.org/10.1088/1741-2560/12/4/046008).
clear; clc;

%% ================= EEG SETTINGS =================

srate = 500;
window_sec = 2;
window_samples = srate * window_sec;

freqs = [8 12 15 18];

% SSVEP frequency -> meaning (used only for console logging)
CUP  = 18;
LIPBALM = 15;
LOTION = 12;
HOME = 8;

corr_thresh   = 0.55;   % NOTE: FBCCA scores are weighted sums of rho across
margin_thresh = 0.05;   % bands, so they live on a different scale than single
                        % CCA rho. Re-tune these two on a calibration run --
                        % see the comment at the decision block below.

decision_rate = 0.25;
cooldown      = 1.5;

channel_indices = [2 14 13 19 16 17 18];
n_channels = length(channel_indices);

vote_buffer = [];
vote_required = 4;

%% ================= FBCCA SETTINGS =================
% Number of sub-bands. Chen et al. use up to 10; 5 is a good cost/accuracy
% point for a real-time loop. Each band m spans [m*8, 88] Hz.
Nb = 5;

% Sub-band weighting w(m) = m^(-a) + b, fitted constants from the paper.
fb_a = 1.25;
fb_b = 0.25;
fb_w = (1:Nb).^(-fb_a) + fb_b;

% Number of harmonics in the CCA reference set (was 2; sub-bands isolate
% harmonics so 3 now helps rather than adding broadband noise).
n_harmonics = 3;

% Upper edge shared by every band; lower edge steps up per band.
fb_high = 88;             % Hz, must be < srate/2 (250 Hz) -- fine.
butter_order = 4;

%% ================= FREQ -> SSVEP CLASS INDEX =================
% scan_and_reach.py maps class indices to AprilTag IDs via freq_to_tag.
%   8 Hz  -> 0  (HOME)
%   12 Hz -> 1  (Lotion)
%   15 Hz -> 2  (Lip Balm)
%   18 Hz -> 3  (Red Cup)
freq_to_class = containers.Map( ...
    {8, 12, 15, 18}, ...
    {int32(0), int32(1), int32(2), int32(3)});

%% ================= CONNECT ROS 2 =================

disp('Initialising ROS 2 node...')

node = ros2node("/ssvep_matlab");
cmd_pub = ros2publisher(node, "/ssvep_command", "std_msgs/Int32");
cmd_msg = ros2message(cmd_pub);

disp('ROS 2 publisher ready on /ssvep_command')

%% ================= CONNECT LSL =================

disp('Connecting to LSL...')

lib = lsl_loadlib();

result = {};
while isempty(result)
    result = lsl_resolve_byprop(lib,'type','EEG');
end

inlet = lsl_inlet(result{1});

disp('LSL connected')

%% ================= BUILD FILTER BANK =================
% One Butterworth bandpass per sub-band, designed ONCE here (not per loop
% iteration) so per-decision latency stays flat. Band m: [m*8, fb_high].
nyq = srate/2;
fb_b_coeffs = cell(1,Nb);
fb_a_coeffs = cell(1,Nb);
for m = 1:Nb
    f_lo = m * 8;                       % 8, 16, 24, ... Hz
    f_hi = min(fb_high, nyq - 1);       % guard against >= Nyquist
    [bb, aa] = butter(butter_order, [f_lo f_hi]/nyq, 'bandpass');
    fb_b_coeffs{m} = bb;
    fb_a_coeffs{m} = aa;
end

%% ================= BUILD CCA REFERENCES =================
% ref{i} is the harmonic stack for frequency freqs(i): sin/cos pairs up to
% n_harmonics. Shared across all sub-bands.

t = linspace(0, window_sec, window_samples);
ref = cell(1, length(freqs));
for i = 1:length(freqs)
    f = freqs(i);
    R = [];
    for h = 1:n_harmonics
        R = [R; sin(2*pi*h*f*t); cos(2*pi*h*f*t)]; %#ok<AGROW>
    end
    ref{i} = R';   % window_samples x (2*n_harmonics)
end

%% ================= BUFFERS =================

buffer = zeros(n_channels, window_samples);

lastDecision = tic;
lastCommand  = tic;

disp('System running')

%% ================= MAIN LOOP =================

while true

    [chunk,~] = inlet.pull_chunk();

    if isempty(chunk)
        continue
    end

    chunk = chunk(channel_indices,:);

    buffer = [buffer chunk];
    buffer = buffer(:,end-window_samples+1:end);

    if toc(lastDecision) < decision_rate
        continue
    end

    lastDecision = tic;

    %% ---- pick analysis channels (reference subtraction) ----
    % data = buffer(2:7,:) - buffer(1,:); % Comment this if using mock data
    data_raw = buffer(2:7, :);            % 6 x window_samples (occipital set)

    %% ---- FBCCA ----
    % For each candidate frequency, sum weighted squared canonical
    % correlations across sub-bands:  score = sum_m w(m) * rho_m^2
    fb_scores = zeros(1, length(freqs));

    for m = 1:Nb
        % Filter the (unfiltered) data into sub-band m, zero-phase.
        sub = filtfilt(fb_b_coeffs{m}, fb_a_coeffs{m}, data_raw')';

        for i = 1:length(freqs)
            [~,~,r] = canoncorr(sub', ref{i});
            fb_scores(i) = fb_scores(i) + fb_w(m) * r(1)^2;
        end
    end

    [max_val, idx] = max(fb_scores);

    tmp = fb_scores;
    tmp(idx) = -inf;
    margin = max_val - max(tmp);

    %% ---- decision ----
    % IMPORTANT: max_val here is a weighted sum of rho^2 over Nb bands, so its
    % scale is NOT comparable to the old single-CCA rho. Run one calibration
    % session, log the printed scores for true vs. rest, and set corr_thresh /
    % margin_thresh accordingly. Starting points below are placeholders.
    if max_val < corr_thresh || margin < margin_thresh
        vote_buffer = [];
        fprintf('Rest  score %.3f  margin %.3f\n', max_val, margin)
        continue
    end

    detected = freqs(idx);
    fprintf('Detected %d Hz  score %.3f  margin %.3f\n', detected, max_val, margin)

    %% cooldown
    if toc(lastCommand) < cooldown
        continue
    end

    %% Accumulate votes
    vote_buffer(end+1) = detected; %#ok<AGROW>
    if length(vote_buffer) > vote_required
        vote_buffer = vote_buffer(end-vote_required+1:end);
    end

    %% Publish on stable vote
    if length(vote_buffer) == vote_required && all(vote_buffer == vote_buffer(end))
        vote_buffer = [];

        class_idx = freq_to_class(detected);
        cmd_msg.data = class_idx;
        send(cmd_pub, cmd_msg);

        switch detected
            case 12
                fprintf('12Hz -> Lotion  | published class %d\n', class_idx);
            case 15
                fprintf('15Hz -> Lip Balm | published class %d\n', class_idx);
            case 18
                fprintf('18Hz -> Red Cup  | published class %d\n', class_idx);
            case 8
                fprintf('8Hz  -> Home     | published class %d\n', class_idx);
        end

        lastCommand = tic;
    end

end
