%% Live SSVEP FBCCA → ROS 2 /ssvep_command Publisher
% Filter Bank CCA (Chen et al., 2015, J. Neural Eng. 12(4):046008,
% https://doi.org/10.1088/1741-2560/12/4/046008).
% Frequency set: 16/19/23/29 Hz — off-alpha (8-13 Hz avoided) and fully
% harmonic-clean (no fundamental and no harmonic up to 3rd coincides).
%
% BANDED REFERENCES: FastLED.show() blocking causes non-uniform per-group
% frequency shifts (observed ~-0.25 Hz on 19 Hz group; last group, 29 Hz,
% worst-affected). Instead of a single reference at the nominal frequency,
% each class is tested against several references spanning +/-0.5 Hz, and
% the best-matching offset per sub-band is kept before FBCCA weighting.
% This absorbs the drift without hard-coding per-group shifted frequencies.
clear; clc;

%% ================= EEG SETTINGS =================

srate = 500;
window_sec = 2;
window_samples = srate * window_sec;

freqs = [16 19 23 29];

% SSVEP frequency → meaning (used only for console logging)
CUP     = 29;
LIPBALM = 23;
LOTION  = 19;
HOME    = 16;

corr_thresh   = 0.3;   % NOTE: FBCCA scores are weighted sums of rho^2 across
margin_thresh = 0.005;   % bands — a different scale than single-CCA rho.
                        % Re-tune on a calibration run (see decision block).

decision_rate = 0.3;
% decision_rate = 0.1;
cooldown      = 1.5; % For cubes
% cooldown      = 0.05; % For screwdriver

channel_indices = [2 14 13 19 16 17 18];
n_channels = length(channel_indices);

vote_buffer = [];
vote_required = 4; % For cubes
% vote_required = 1; % For screwdriver


%% ================= FBCCA SETTINGS =================
% Number of sub-bands. Chen et al. use up to 10; 5 is a good cost/accuracy
% point for a real-time loop.
Nb = 5;

% Sub-band weighting w(m) = m^(-a) + b, fitted constants from the paper.
fb_a = 1.25;
fb_b = 0.25;
fb_w = (1:Nb).^(-fb_a) + fb_b;

% Number of harmonics in the CCA reference set.
n_harmonics = 3;

% Sub-band lower edge starts at the lowest fundamental (16 Hz) and steps up;
% upper edge shared. 100 Hz covers the 29 Hz 3rd harmonic (87 Hz), < Nyquist.
fb_low_base  = 14;        % Hz, lowest stimulus fundamental
fb_high      = 100;       % Hz, must be < srate/2 (250 Hz)
butter_order = 4;

%% ================= BAND-CCA SETTINGS =================
% Offsets tested around each nominal frequency, in Hz. Keep this list short:
% every extra offset multiplies canoncorr() calls by (Nb x n_freqs) per
% decision window, and decision_rate = 0.25 s is already tight.
% [-0.5 0 0.5] is a reasonable default given the observed drift magnitude;
% widen/densify (e.g. -0.5:0.25:0.5) only if real-time timing allows.
band_offsets = [-0.5 0 0.5];

%% ================= FREQ → SSVEP CLASS INDEX =================
% scan_and_reach maps class indices to AprilTag IDs.
%   16 Hz -> 0  (HOME)
%   19 Hz -> 1  (Lotion)
%   23 Hz -> 2  (Lip Balm)
%   29 Hz -> 3  (Red Cup)
freq_to_class = containers.Map( ...
    {16, 19, 23, 29}, ...
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
% One Butterworth bandpass per sub-band, designed ONCE. Band m: [m*16, fb_high].
nyq = srate/2;
fb_b_coeffs = cell(1,Nb);
fb_a_coeffs = cell(1,Nb);
for m = 1:Nb
    f_lo = m * fb_low_base;              % 16, 32, 48, 64, 80 Hz
    f_hi = min(fb_high, nyq - 1);        % guard against >= Nyquist
    if f_lo >= f_hi
        f_lo = f_hi - 2;                 % cap higher sub-bands into range
    end
    [bb, aa] = butter(butter_order, [f_lo f_hi]/nyq, 'bandpass');
    fb_b_coeffs{m} = bb;
    fb_a_coeffs{m} = aa;
end

%% ================= BUILD CCA REFERENCES (BANDED) =================
% ref{i,k} is the harmonic stack for freqs(i) + band_offsets(k): sin/cos
% pairs up to n_harmonics. Shared across all sub-bands.

t = linspace(0, window_sec, window_samples);
n_offsets = length(band_offsets);
ref = cell(length(freqs), n_offsets);
for i = 1:length(freqs)
    f0 = freqs(i);
    for k = 1:n_offsets
        f = f0 + band_offsets(k);
        R = [];
        for h = 1:n_harmonics
            R = [R; sin(2*pi*h*f*t); cos(2*pi*h*f*t)]; %#ok<AGROW>
        end
        ref{i,k} = R';   % window_samples x (2*n_harmonics)
    end
end

%% ================= BUFFERS =================

buffer = zeros(n_channels, window_samples);

lastDecision = tic;
lastCommand  = tic;

%% ================= SCORE PUBLISHER =================
% Publish every decision window on /ssvep_scores so the RECORDER can log it
% alongside the raw EEG and cue markers — one file, one timeline, no offline
% join. Keeps the classifier experiment-agnostic: it emits raw decision values,
% not trial/experiment context. Payload is a std_msgs/Float32MultiArray:
%   [score_16, score_19, score_23, score_29, winner_hz, max_val, margin,
%    passed, published]
score_pub = ros2publisher(node, "/ssvep_scores", "std_msgs/Float32MultiArray");
score_msg = ros2message(score_pub);

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
    data_raw = buffer(2:7,:) - buffer(1,:); % Comment this if using mock data
    % data_raw = buffer(2:7, :);            % 6 x window_samples (occipital set)

    %% ---- FBCCA (banded) ----
    % score(i) = sum_m w(m) * max_k rho_{m,i,k}^2
    % For each sub-band and each class, take the best-matching offset
    % within the +/-0.5 Hz band before applying the FBCCA weight.
    fb_scores = zeros(1, length(freqs));

    for m = 1:Nb
        sub = filtfilt(fb_b_coeffs{m}, fb_a_coeffs{m}, data_raw')';
        for i = 1:length(freqs)
            best_r2 = 0;
            for k = 1:n_offsets
                [~,~,r] = canoncorr(sub', ref{i,k});
                r2 = r(1)^2;
                if r2 > best_r2
                    best_r2 = r2;
                end
            end
            fb_scores(i) = fb_scores(i) + fb_w(m) * best_r2;
        end
    end

    [max_val, idx] = max(fb_scores);

    tmp = fb_scores;
    tmp(idx) = -inf;
    margin = max_val - max(tmp);

    %% ---- decision ----
    % max_val is a weighted sum of rho^2 over Nb bands, NOT comparable to
    % single-CCA rho. Re-tune corr_thresh / margin_thresh on a calibration run.
    if max_val < corr_thresh || margin < margin_thresh
        vote_buffer = [];
        fprintf('Rest  score %.3f  margin %.3f\n', max_val, margin)
        score_msg.data = single([fb_scores(1) fb_scores(2) fb_scores(3) ...
            fb_scores(4) 0 max_val margin 0 0]);   % winner=0 rest, passed=0, pub=0
        send(score_pub, score_msg);
        continue
    end

    detected = freqs(idx);
    fprintf('Detected %d Hz  score %.3f  margin %.3f\n', detected, max_val, margin)

    %% cooldown
    if toc(lastCommand) < cooldown
        score_msg.data = single([fb_scores(1) fb_scores(2) fb_scores(3) ...
            fb_scores(4) detected max_val margin 1 0]);   % passed=1, pub=0 cooldown
        send(score_pub, score_msg);
        continue
    end

    %% Accumulate votes
    vote_buffer(end+1) = detected; %#ok<AGROW>
    if length(vote_buffer) > vote_required
        vote_buffer = vote_buffer(end-vote_required+1:end);
    end

    %% Publish on stable vote
    published = 0;
    if length(vote_buffer) == vote_required && all(vote_buffer == vote_buffer(end))
        vote_buffer = [];

        class_idx = freq_to_class(detected);
        cmd_msg.data = class_idx;
        send(cmd_pub, cmd_msg);
        published = 1;

        switch detected
            case 19
                fprintf('19Hz -> Lotion   | published class %d\n', class_idx);
            case 23
                fprintf('23Hz -> Lip Balm | published class %d\n', class_idx);
            case 29
                fprintf('29Hz -> Red Cup  | published class %d\n', class_idx);
            case 16
                fprintf('16Hz -> Home     | published class %d\n', class_idx);
        end

        lastCommand = tic;
    end

    % Publish this passed window (published=1 if it fired, else 0 = held by voting).
    score_msg.data = single([fb_scores(1) fb_scores(2) fb_scores(3) ...
        fb_scores(4) detected max_val margin 1 published]);
    send(score_pub, score_msg);

end
