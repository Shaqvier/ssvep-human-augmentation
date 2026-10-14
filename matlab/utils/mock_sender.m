%% Mock EEG LSL Sender
% Simulates SSVEP EEG data at specified frequencies

lib = lsl_loadlib();

n_channels = 20;
srate = 500;
freqs = [8, 12, 15, 18]; % Your SSVEP frequencies

% Create stream info and outlet
info = lsl_streaminfo(lib, 'MockEEG', 'EEG', n_channels, srate, 'cf_float32', 'mockuid123');
outlet = lsl_outlet(info);

disp('Streaming mock EEG... Ctrl+C to stop');

t = 0;
dt = 1/srate;
active_freq = freqs(2); % Change to test different SSVEP classes

fprintf('Streaming at: %d Hz\n', active_freq);
while true
    sample = zeros(1, n_channels);
    clean = sin(2*pi*active_freq*t) ...
        + 0.8*sin(4*pi*active_freq*t) ...  % stronger 2nd harmonic
        + 0.6*sin(6*pi*active_freq*t);     % add 3rd harmonic to match CCA
    for ch = 1:n_channels
        sample(ch) = clean + 0.3*randn(); % same signal, small noise
    end
    outlet.push_sample(sample);
    t = t + dt;
    pause(dt);
end
