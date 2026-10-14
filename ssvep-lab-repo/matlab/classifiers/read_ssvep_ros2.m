%% Live SSVEP CCA -> ROS 2 /ssvep_command Publisher
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

corr_thresh   = 0.55;
margin_thresh = 0.05;

decision_rate = 0.25;
cooldown      = 1.5;

channel_indices = [2 14 13 19 16 17 18];
n_channels = length(channel_indices);

vote_buffer = [];
vote_required = 4;

%% ================= FREQ -> SSVEP CLASS INDEX =================
% scan_and_reach.py maps class indices to AprilTag IDs via freq_to_tag.
% Publish a std_msgs/Int32 class index. Adjust mapping to match the
% Python-side dictionary.
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

%% ================= FILTER =================

[b,a] = butter(4,[5 45]/(srate/2),'bandpass');

%% ================= BUILD CCA REFERENCES =================

t = linspace(0,window_sec,window_samples);

ref = cell(1,length(freqs));

for i = 1:length(freqs)

    f = freqs(i);

    ref{i} = [ ...
        sin(2*pi*f*t)
        cos(2*pi*f*t)
        sin(4*pi*f*t)
        cos(4*pi*f*t) ]';

end

%% ================= BUFFERS =================

buffer = zeros(n_channels,window_samples);

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

    %% reference + filter

    % data = buffer(2:7,:) - buffer(1,:); % Comment this if using mock data
    data = buffer(2:7, :);
    data = filtfilt(b,a,data')';

    %% CCA

    corrvals = zeros(1,length(freqs));

    for i = 1:length(freqs)
        [~,~,r] = canoncorr(data',ref{i});
        corrvals(i) = r(1);
    end

    [max_val,idx] = max(corrvals);

    tmp = corrvals;
    tmp(idx) = -inf;

    margin = max_val - max(tmp);

    if max_val < corr_thresh || margin < margin_thresh
        vote_buffer = [];
        fprintf('Rest %.3f\n',max_val)
        continue
    end

    detected = freqs(idx);

    fprintf('Detected %d Hz (%.3f)\n',detected,max_val)

    %% cooldown

    if toc(lastCommand) < cooldown
        continue
    end

    %% Accumulate votes
    vote_buffer(end+1) = detected;
    if length(vote_buffer) > vote_required
        vote_buffer = vote_buffer(end-vote_required+1:end); % keep last N
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
