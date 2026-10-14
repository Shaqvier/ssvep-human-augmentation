function analyze_recording(matfile)
%% Offline analysis of one continuous SSVEP recording.
% Loads a recordings/exp<ID>_<stamp>.mat produced by record_ssvep_continuous.m,
% slices it into per-trial epochs using the marker table, and reports:
%   - per-trial spectra at the four stimulus frequencies
%   - target-vs-flank SNR
%   - a re-run of FBCCA on each epoch (so you see what the classifier saw)
%   - cue-locked average across ALL channels (the trigger-coupling test)
%
% Usage:   >> analyze_recording('recordings/exp1_20260715T143022.mat')
%          >> analyze_recording        % picks the newest file in recordings/
%
% Needs no LSL, no ROS, no hardware. Run it anywhere you have the .mat.

if nargin < 1
    d = dir(fullfile('recordings', 'exp*.mat'));
    if isempty(d)
        error('No recordings/exp*.mat found. Pass a path explicitly.');
    end
    [~, newest] = max([d.datenum]);
    matfile = fullfile(d(newest).folder, d(newest).name);
    fprintf('Using newest recording: %s\n', matfile);
end

L = load(matfile);
eeg = L.eeg; fs = L.fs; markers = L.markers;
fprintf('\n=== %s ===\n', matfile);
fprintf('Channels: %d   Samples: %d   Duration: %.1f s   fs: %g Hz\n', ...
        size(eeg,1), size(eeg,2), size(eeg,2)/fs, fs);
fprintf('Experiment id: %s\n', string(L.exp_id));

%% ---- Marker summary ----
fprintf('\n--- Markers (%d total) ---\n', height(markers));
types = unique(markers.type);
for k = 1:numel(types)
    n = sum(strcmp(markers.type, types{k}));
    fprintf('  %-18s %d\n', types{k}, n);
end

cues = markers.sample(strcmp(markers.type, 'cue_onset'));
ends = markers.sample(strcmp(markers.type, 'trial_end'));
cue_src = markers.info(strcmp(markers.type, 'cue_onset'));
if ~isempty(cue_src)
    fprintf('  cue source: %s\n', strjoin(unique(string(cue_src)), ', '));
end

if isempty(cues)
    warning(['No cue_onset markers — cannot slice trials. If you ran with ' ...
             'USE_TRIGGER_CHANNEL=true, TRIGGER_CHANNEL_INDEX was probably ' ...
             'wrong (silent failure: recording works, no cues detected).']);
    return
end

%% ---- Channel setup ----
% Edit if your montage differs. This is the 32-ch CACS-32 order from the
% actiCHamp LSL dump.
labels = {'Fp1','Fz','F3','F7','F9','FC5','FC1','C3','T7','CP5','CP1','Pz', ...
          'P3','P7','P9','O1','Oz','O2','P10','P8','P4','CP2','CP6','T8', ...
          'C4','Cz','FC2','FC6','F10','F8','F4','Fp2'};

% actiCHamp APPENDS the trigger channel at the end when it is armed, so a
% recording is 32 ch (disarmed) or 33 ch (armed, trigger last). EEG indices
% 1..32 are identical either way.
TRIG_IDX = [];
if size(eeg,1) == numel(labels) + 1
    labels{end+1} = 'TRIG';
    TRIG_IDX = numel(labels);
    fprintf('\n33 channels: trigger detected at index %d (EEG 1-32 unchanged).\n', ...
            TRIG_IDX);
elseif size(eeg,1) ~= numel(labels)
    warning('Recording has %d channels but label list has %d. Labels may be wrong.', ...
            size(eeg,1), numel(labels));
    labels = arrayfun(@(k) sprintf('ch%d',k), 1:size(eeg,1), 'uni', 0);
end

occ  = [14 13 19 16 17 18];      % connected posterior electrodes
refi = 2;                        % Fz
% IMPORTANT: these match the live classifier's channel_indices =
% [2 14 13 19 16 17 18]. Do NOT resolve by label — the 32-label list describes
% the CAP LAYOUT, not which sites are gelled. On this rig indices 12 (Pz) and
% 21 (P4) are FLOATING (std 1279 / 1778 vs ~60 for connected channels) and
% feeding them to CCA produces spurious structure.
occ_names = labels(occ);
fprintf('\nOccipital set (connected): %s  (indices %s)\n', ...
        strjoin(occ_names,' '), mat2str(occ));
fprintf('Reference: %s (index %d)\n', labels{refi}, refi);

% Flag any selected channel that looks like a floating input.
sdev = std(double(eeg(occ,:)), 0, 2);
if any(sdev > 5*median(sdev))
    bad = find(sdev > 5*median(sdev));
    fprintf('*** WARNING: idx %s std %s vs median %.0f — possible floating input\n', ...
            mat2str(occ(bad)), mat2str(round(sdev(bad)')), median(sdev));
end

%% ---- Channel character dump ----
% Identify the trigger EMPIRICALLY rather than trusting its assumed position.
% The trigger row is near-zero variance with very few distinct values and a
% large max; EEG rows are noisy with many distinct values.
fprintf('\n--- Channel character (trigger = low std, few unique, large max) ---\n');
fprintf('%4s %-6s %10s %10s %10s %10s %8s\n', ...
        'idx','label','mean','std','min','max','n_uniq');
for k = 1:size(eeg,1)
    nu = numel(unique(round(eeg(k,:), 3)));
    flag = '';
    if nu < 20 && max(eeg(k,:)) > 10*std(eeg(k,:)) + mean(eeg(k,:))
        flag = '  <-- looks like TRIGGER';
    end
    fprintf('%4d %-6s %10.3g %10.3g %10.3g %10.3g %8d%s\n', ...
            k, labels{k}, mean(eeg(k,:)), std(eeg(k,:)), ...
            min(eeg(k,:)), max(eeg(k,:)), nu, flag);
end

freqs = [16 19 23 29];

%% ---- Per-trial analysis ----
% Prefer cue_onset -> trial_end. If trial_end markers are missing (e.g. the
% block was aborted, or the robot never completed a run), fall back to
% cue-to-cue slicing so every cue is still analysable.
if numel(ends) < numel(cues)
    fprintf(['\nNOTE: %d cues but only %d trial_end markers — falling back to\n' ...
             'cue-to-cue slicing (last trial runs to end of recording).\n'], ...
            numel(cues), numel(ends));
    bounds = [cues(:); size(eeg,2)];
    starts = bounds(1:end-1);
    stops  = bounds(2:end) - 1;
else
    starts = cues(1:min(numel(cues),numel(ends)));
    stops  = ends(1:min(numel(cues),numel(ends)));
end

n_trials = numel(starts);
fprintf('\n--- Per-trial (%d trials) ---\n', n_trials);
fprintf('%5s %8s %8s %9s %9s %9s %9s  %s\n', ...
        'trial','start_s','dur_s','p16','p19','p23','p29','argmax');

for i = 1:n_trials
    s0 = starts(i); s1 = min(stops(i), size(eeg,2));
    if s1 <= s0, continue; end
    % NO Fz subtraction: BrainVision Recorder already references before
    % streaming, so subtracting again would be double subtraction.
    ep  = eeg(occ, s0:s1);

    % Power at each stimulus frequency, averaged over occipital channels
    p = zeros(1,4);
    for f = 1:4
        acc = 0;
        for c = 1:size(ep,1)
            [pxx, ff] = pwelch(ep(c,:)', [], [], [], fs);
            acc = acc + mean(pxx(abs(ff-freqs(f)) < 0.5));
        end
        p(f) = acc / size(ep,1);
    end
    [~, am] = max(p);

    fprintf('%5d %8.2f %8.2f %9.3g %9.3g %9.3g %9.3g  %d Hz\n', ...
            i, s0/fs, (s1-s0)/fs, p(1), p(2), p(3), p(4), freqs(am));
end

%% ---- SNR at each frequency (whole recording, occipital mean) ----
fprintf('\n--- Target/flank SNR (whole recording, occipital mean) ---\n');
ep_all = eeg(occ,:);          % already referenced upstream — do not subtract Fz
for f = 1:4
    snr_acc = 0;
    for c = 1:size(ep_all,1)
        [pxx, ff] = pwelch(ep_all(c,:)', [], [], [], fs);
        tgt   = mean(pxx(abs(ff-freqs(f)) < 0.25));
        flank = mean(pxx(abs(ff-freqs(f)) > 1 & abs(ff-freqs(f)) < 3));
        snr_acc = snr_acc + tgt/flank;
    end
    fprintf('  %2d Hz: %.2f\n', freqs(f), snr_acc/size(ep_all,1));
end

%% ---- Low-frequency contamination check ----
% Sub-band 1 is [16,100] and is the ONLY band containing fundamentals, so
% anything leaking up from below 16 Hz inflates every class score.
fprintf('\n--- Low-frequency contamination (occipital mean) ---\n');
[pxx, ff] = pwelch(mean(ep_all,1)', [], [], [], fs);
bands = {'0.5-4 (delta)',[0.5 4]; '4-8 (theta)',[4 8]; ...
         '8-13 (alpha)',[8 13]; '13-16 (below b1)',[13 16]; ...
         '16-30 (stim)',[16 30]};
for k = 1:size(bands,1)
    r = bands{k,2};
    fprintf('  %-18s %.4g\n', bands{k,1}, mean(pxx(ff>=r(1) & ff<r(2))));
end
fprintf(['  If delta/theta dominate the stim band, unsubtracted drift is\n' ...
         '  leaking into sub-band 1 -> every class score inflated.\n']);

%% ---- CUE-LOCKED AVERAGE: the trigger-coupling test ----
% Average all channels around cue onset. A transient on FRONTAL channels
% (far from the LED strip) indicates conducted coupling from the trigger lead.
fprintf('\n--- Cue-locked average (trigger-coupling test) ---\n');
pre = round(0.25*fs); post = round(0.25*fs);
valid = cues(cues > pre & cues < size(eeg,2)-post);
if isempty(valid)
    fprintf('  Not enough margin around cues to build epochs.\n');
else
    stack = zeros(size(eeg,1), pre+post+1, numel(valid));
    for i = 1:numel(valid)
        stack(:,:,i) = eeg(:, valid(i)-pre : valid(i)+post);
    end
    avg = mean(stack, 3);
    base = mean(avg(:, 1:pre), 2);
    avg = avg - base;                       % baseline-correct

    t_ms = (-pre:post)/fs*1000;

    % EEG channels only — the trigger would swamp the uV scale.
    eeg_rows = setdiff(1:size(eeg,1), TRIG_IDX);
    figure('Name','Cue-locked average (EEG channels)');
    plot(t_ms, avg(eeg_rows,:)'); xlabel('ms from cue onset');
    ylabel('uV (baseline-corr)');
    title('Cue-locked average — look for a transient at t=0'); grid on;
    xline(0,'k--');

    % Trigger channel plotted separately: sanity check that the pulse really
    % sits at t=0 (confirms cue markers are aligned to the TTL edge).
    if ~isempty(TRIG_IDX)
        figure('Name','Trigger channel (sanity check)');
        plot(t_ms, avg(TRIG_IDX,:)); xlabel('ms from cue onset');
        title('Trigger channel — pulse should sit at t=0'); grid on;
        xline(0,'k--');
    end

    % Quantify: peak deviation in the 0-20 ms window, per channel
    w = t_ms >= 0 & t_ms <= 20;
    pk = max(abs(avg(:,w)), [], 2);
    frontal = {'Fp1','Fp2','Fz','F3','F4'};
    fi = find(ismember(lower(labels), lower(frontal)));
    fprintf('  Peak |deviation| 0-20 ms after cue:\n');
    fprintf('    occipital (%s): %.3g uV\n', strjoin(occ_names,','), mean(pk(occ)));
    fprintf('    frontal   (%s): %.3g uV\n', strjoin(frontal,','), mean(pk(fi)));
    eeg_pk = pk(eeg_rows);
    [mx, mi] = max(eeg_pk);
    fprintf('    max any EEG channel: %.3g uV (%s)\n', mx, labels{eeg_rows(mi)});
    fprintf(['  >>> A clear transient on FRONTAL channels = conducted coupling\n' ...
             '      from the trigger lead (frontal sites are far from the LEDs).\n' ...
             '      Occipital-only = suspect the LED strip / visual response.\n']);
end

fprintf('\nDone. Figures opened for inspection.\n');
end
