%% SSVEP Offline Analysis — CCA + FBCCA
% Analyses all BrainVision (.eeg/.vhdr/.vmrk) recordings from the
% SSVEP_Offline_Experiment.ino paradigm.
%
% METRICS COMPUTED
%   - Per-trial CCA and FBCCA classification (correct / wrong / no-decision)
%   - Accuracy (%) per run and pooled across runs
%   - ITR (bits/min) — Wolpaw 2000 formula, threshold-gated
%   - SNR per frequency (signal power at target / mean of flanking bins)
%   - Responsiveness — latency (s) from stimulus onset to first correct
%     sliding-window detection within each trial
%
% PARADIGM FACTS (from SSVEP_Offline_Experiment.ino)
%   Frequencies  : 16 / 19 / 23 / 29 Hz   (class 0/1/2/3)
%   Trial sequence (20 trials, balanced): [0,2,1,3,2,0,3,1,2,3,1,0,3,2,0,3,1,2,0,1]
%   Timing        : 1 s cue (TTL at cue onset) → 4 s stimulus → 1 s blank
%   Epoch         : TTL + 1000 ms (stim onset) to TTL + 5000 ms (end of stim)
%
% CHANNEL CONVENTIONS (32-ch actiCHamp, 1-indexed)
%   Loaded indices : [2 14 13 19 16 17 18]  (P3, O1, Pz, O2, P4, Oz, ref-ish)
%   Analysis set   : channels 2–7 of the above (6 occipital channels)
%   This matches read_ssvep_ros2_fbcca.m exactly.
%
% USAGE
%   1. Set DATA_DIR to the folder containing your .vhdr / .eeg / .vmrk files.
%   2. Run. Figures and a results table are displayed; CSV is written to DATA_DIR.
%
% REQUIREMENTS
%   MATLAB R2020b+, Signal Processing Toolbox, Statistics & ML Toolbox.
%   BrainVision files are read with the included bv_read helpers (no EEGLAB
%   needed); functions are appended at the bottom of this file.

clear; clc; close all;

% ════════════════════════════════════════════════════════════════════════════
%% 1. CONFIGURATION — edit these
% ════════════════════════════════════════════════════════════════════════════

DATA_DIR = '.';          % folder with your .vhdr/.eeg/.vmrk files
                         % '.' = current directory; change to full path if needed

% ── Paradigm ──────────────────────────────────────────────────────────────
FREQS        = [16 19 23 29];          % Hz, class 0..3
TRIAL_SEQ    = [0,2,1,3,2,0,3,1,2,3,1,0,3,2,0,3,1,2,0,1];  % class index
N_TRIALS     = 20;
CUE_MS       = 1000;    % ms: TTL fires at cue onset; stimulus starts after this
STIM_MS      = 4000;    % ms: analysis window duration

% ── Channels — selected by ELECTRODE NAME (robust across montages) ────────
% The script looks these names up in each file's .vhdr channel list, so it
% adapts automatically even if channel order differs between recordings.
% These six are the standard occipital-parietal SSVEP set for your CACS-32 cap.
ANALYSIS_CH_NAMES = {'Pz', 'P3', 'O1', 'Oz', 'O2', 'P4'};
% Optional reference channel subtracted from the analysis set (set '' to skip).
% Your decoder used buffer(2:7,:) without subtraction for real data, so '' here.
REF_CH_NAME = '';

% ── Sliding-window classifier settings ────────────────────────────────────
WINDOW_SEC   = 2.0;     % analysis window length (s)
STEP_SEC     = 0.25;    % sliding step (s) — matches decision_rate in decoder
N_HARMONICS  = 3;

% ── FBCCA settings (Chen et al. 2015) ────────────────────────────────────
NB           = 5;       % number of sub-bands
FB_A         = 1.25;    % weighting exponent
FB_B         = 0.25;    % weighting offset
FB_HIGH      = 88;      % upper edge (Hz) — shared by all sub-bands
BUTTER_ORDER = 4;

% ── Threshold sweep for accuracy/ITR curves ───────────────────────────────
% Thresholds are applied independently to CCA rho and FBCCA score.
% Trials with max score < threshold → "no decision" (excluded from accuracy
% but counted as 0 for ITR calculation).
CCA_THRESH_RANGE   = 0.10 : 0.05 : 0.90;
FBCCA_THRESH_RANGE = 0.05 : 0.05 : 1.50;

% ── Thresholded confusion matrix ──────────────────────────────────────────
% The operating threshold is auto-picked as the LOWEST swept value whose
% accuracy (over decided trials) beats 25% chance, while still deciding at
% least this fraction of trials. Adds a 'No decision' column for abstentions.
CM_MIN_DECIDED_FRAC = 0.50;   % require ≥50% of trials decided when picking thr

% ── SNR window ────────────────────────────────────────────────────────────
SNR_FLANK_BINS = 2;   % number of flanking FFT bins each side of target

% ── Output ────────────────────────────────────────────────────────────────
SAVE_CSV = true;   % write results table to DATA_DIR/ssvep_results.csv

% ════════════════════════════════════════════════════════════════════════════
%% 2. FIND FILES
% ════════════════════════════════════════════════════════════════════════════

vhdr_files = dir(fullfile(DATA_DIR, '*.vhdr'));
if isempty(vhdr_files)
    error('No .vhdr files found in: %s', DATA_DIR);
end
fprintf('Found %d recording(s):\n', numel(vhdr_files));
for k = 1:numel(vhdr_files)
    fprintf('  %d. %s\n', k, vhdr_files(k).name);
end
fprintf('\n');

% ════════════════════════════════════════════════════════════════════════════
%% 3. PRE-BUILD REFERENCES AND FILTER BANK (shared across all files)
% ════════════════════════════════════════════════════════════════════════════

% Will be rebuilt per file once srate is known; placeholder here.
refs_cca  = [];
fb_filts  = [];

% ════════════════════════════════════════════════════════════════════════════
%% 4. MAIN LOOP — one iteration per recording file
% ════════════════════════════════════════════════════════════════════════════

all_results = struct('file',{},'n_trials',{},'cca_acc_raw',{}, ...
    'fbcca_acc_raw',{},'cca_peak_itr',{},'fbcca_peak_itr',{}, ...
    'cca_peak_acc',{},'fbcca_peak_acc',{},'snr_16hz',{},'snr_19hz',{}, ...
    'snr_23hz',{},'snr_29hz',{},'cca_latency_s',{},'fbcca_latency_s',{});   % grows as we process files

% ── POOLED accumulators (all trials from all files combined) ──────────────
POOL.true_class    = [];   % 1 × N  (0-indexed)
POOL.cca_scores    = [];   % N × n_freqs (best CCA rho per freq per trial)
POOL.fbcca_scores  = [];   % N × n_freqs (best FBCCA score per freq per trial)
POOL.cca_latency   = [];   % 1 × N
POOL.fbcca_latency = [];   % 1 × N
POOL.snr_epochs    = [];   % N × n_freqs
POOL.src_file      = {};   % 1 × N  (which file each trial came from)
POOL.trial_spec    = [];   % N × spec_len (channel-averaged power spectrum)
POOL.freq_axis     = [];   % 1 × spec_len (Hz; set on first file)

for fk = 1:numel(vhdr_files)

    fname = vhdr_files(fk).name;
    fpath = fullfile(DATA_DIR, fname);
    [~, fstem, ~] = fileparts(fname);
    fprintf('══════════════════════════════════════════\n');
    fprintf('Processing: %s\n', fname);

    % ── 4a. Load BrainVision file ────────────────────────────────────────
    [eeg_full, srate, ch_names, markers] = bv_load(fpath);
    fprintf('  Loaded: %d ch × %d samples @ %d Hz\n', ...
        size(eeg_full,1), size(eeg_full,2), srate);

    n_ch_full = size(eeg_full, 1);

    % ── 4b. Resolve analysis channels by NAME ────────────────────────────
    ch_idx = zeros(1, numel(ANALYSIS_CH_NAMES));
    missing = {};
    for ci = 1:numel(ANALYSIS_CH_NAMES)
        hit = find(strcmpi(ch_names, ANALYSIS_CH_NAMES{ci}), 1);
        if isempty(hit)
            missing{end+1} = ANALYSIS_CH_NAMES{ci}; %#ok<AGROW>
            ch_idx(ci) = NaN;
        else
            ch_idx(ci) = hit;
        end
    end
    if ~isempty(missing)
        warning('  Channels not found in %s: %s — skipping file.', ...
            fstem, strjoin(missing, ', '));
        continue
    end
    fprintf('  Analysis channels: %s  ->  indices [%s]\n', ...
        strjoin(ANALYSIS_CH_NAMES, ' '), ...
        strjoin(arrayfun(@(x) num2str(x), ch_idx, 'UniformOutput', false), ' '));

    data_occ = eeg_full(ch_idx, :);   % n_analysis × T

    % Optional reference subtraction
    if ~isempty(REF_CH_NAME)
        ref_hit = find(strcmpi(ch_names, REF_CH_NAME), 1);
        if ~isempty(ref_hit)
            data_occ = data_occ - eeg_full(ref_hit, :);
            fprintf('  Referenced to %s (ch %d).\n', REF_CH_NAME, ref_hit);
        else
            warning('  Reference channel %s not found — using raw.', REF_CH_NAME);
        end
    end

    % ── 4b-i. Sanity check & clean non-finite samples ───────────────────
    n_bad = sum(~isfinite(data_occ(:)));
    if n_bad > 0
        pct_bad = 100 * n_bad / numel(data_occ);
        fprintf('  [data diag] %d non-finite samples in analysis channels (%.3f%%).\n', ...
            n_bad, pct_bad);
        % Replace NaN/Inf by linear interpolation per channel so filtfilt is happy.
        for ch = 1:size(data_occ,1)
            x = data_occ(ch,:);
            bad = ~isfinite(x);
            if any(bad) && ~all(bad)
                x(bad) = interp1(find(~bad), x(~bad), find(bad), ...
                    'linear', 'extrap');
                data_occ(ch,:) = x;
            elseif all(bad)
                data_occ(ch,:) = 0;   % whole channel dead — zero it
            end
        end
    end
    % Report basic amplitude stats — if these are absurd (e.g. 1e6 µV) the
    % binary format / reshape is wrong, not a NaN gap.
    fprintf('  [data diag] analysis-channel range: [%.1f, %.1f] µV, median |x| = %.1f µV\n', ...
        min(data_occ(:)), max(data_occ(:)), median(abs(data_occ(:))));

    % ── 4c. Find stimulus-onset sample for each trial ───────────────────
    % TTL fires at CUE onset; stimulus starts CUE_MS later.
    trig_samples = bv_find_triggers(markers, srate);
    if isempty(trig_samples)
        warning(['  No triggers found in %s — skipping this file.\n' ...
            '  Run  type(''%s.vmrk'')  to inspect the marker file format.'], ...
            fstem, fstem);
        continue
    end
    if numel(trig_samples) < N_TRIALS
        warning('  Only %d triggers found (expected %d) — will analyse available trials.', ...
            numel(trig_samples), N_TRIALS);
    end
    n_trials_this = min(numel(trig_samples), N_TRIALS);
    stim_onset_samp = trig_samples(1:n_trials_this) + round(CUE_MS/1000 * srate);

    % ── 4d. Build references and filter bank (once per srate) ────────────
    win_samp  = round(WINDOW_SEC * srate);
    step_samp = round(STEP_SEC   * srate);
    stim_samp = round(STIM_MS/1000 * srate);
    nyq = srate / 2;

    % CCA reference signals
    t_ref = (0:win_samp-1) / srate;
    refs = cell(1, numel(FREQS));
    for fi = 1:numel(FREQS)
        R = [];
        for h = 1:N_HARMONICS
            R = [R; sin(2*pi*h*FREQS(fi)*t_ref); ...
                    cos(2*pi*h*FREQS(fi)*t_ref)];
        end
        refs{fi} = R';   % win_samp × (2*N_HARMONICS)
    end

    % FBCCA filter bank
    fb_b = cell(1, NB);
    fb_a = cell(1, NB);
    fb_w = (1:NB).^(-FB_A) + FB_B;
    for m = 1:NB
        f_lo = m * 8;
        f_hi = min(FB_HIGH, nyq - 1);
        [fb_b{m}, fb_a{m}] = butter(BUTTER_ORDER, [f_lo f_hi]/nyq, 'bandpass');
    end

    % Broadband pre-filter for CCA (5–45 Hz, same as decoder)
    [b_bp, a_bp] = butter(4, [5 45]/nyq, 'bandpass');

    % ── 4e. Per-trial analysis ────────────────────────────────────────────
    % Results arrays for this file
    cca_scores_all   = nan(n_trials_this, numel(FREQS));  % max rho per freq
    fbcca_scores_all = nan(n_trials_this, numel(FREQS));  % max score per freq
    true_class       = TRIAL_SEQ(1:n_trials_this);        % 0-indexed class

    % Responsiveness: latency (s) to FIRST window that correctly classifies
    % (no threshold applied — best-case detection time)
    cca_latency   = nan(1, n_trials_this);
    fbcca_latency = nan(1, n_trials_this);

    % SNR accumulator: 6 ch × n_freqs × n_trials (will average across ch+trials)
    snr_epochs = nan(n_trials_this, numel(FREQS));

    % Spectrum accumulator: store the channel-averaged power spectrum per trial
    % so we can later average within each true class (diagnostic figure).
    spec_len   = floor(stim_samp/2) + 1;          % one-sided FFT length
    trial_spec = nan(n_trials_this, spec_len);    % power spectrum per trial
    freq_axis  = (0:spec_len-1) * (srate / stim_samp);   % Hz

    for tr = 1:n_trials_this
        cls = true_class(tr);   % 0-indexed
        true_fi = cls + 1;      % 1-indexed into FREQS

        s0 = stim_onset_samp(tr);
        s1 = s0 + stim_samp - 1;

        if s1 > size(data_occ, 2)
            warning('  Trial %d: epoch extends beyond file — skipping.', tr);
            continue
        end

        epoch = data_occ(:, s0:s1);   % 6 × stim_samp

        % ── SNR (over full 4 s epoch, averaged across channels) ──────────
        snr_per_ch = zeros(numel(FREQS), size(epoch,1));
        ch_spec = zeros(size(epoch,1), spec_len);   % per-channel one-sided power
        for ch = 1:size(epoch,1)
            full_fft = abs(fft(epoch(ch,:))).^2;
            sig_fft  = full_fft;
            ch_spec(ch,:) = full_fft(1:spec_len);   % keep one-sided for plotting
            freq_res = srate / stim_samp;   % Hz per bin
            for fi = 1:numel(FREQS)
                target_bin = round(FREQS(fi) / freq_res) + 1;
                flank_bins = [target_bin - SNR_FLANK_BINS : target_bin - 1, ...
                              target_bin + 1 : target_bin + SNR_FLANK_BINS];
                flank_bins = flank_bins(flank_bins >= 1 & flank_bins <= length(sig_fft));
                snr_per_ch(fi, ch) = sig_fft(target_bin) / mean(sig_fft(flank_bins));
            end
        end
        snr_epochs(tr, :) = mean(snr_per_ch, 2)';
        trial_spec(tr, :) = mean(ch_spec, 1);   % average across channels

        % ── Sliding window classification ─────────────────────────────────
        offsets = 0 : step_samp : (stim_samp - win_samp);

        best_cca_score   = zeros(1, numel(FREQS));
        best_fbcca_score = zeros(1, numel(FREQS));
        cca_first_correct   = NaN;
        fbcca_first_correct = NaN;

        for oi = 1:numel(offsets)
            ws = offsets(oi) + 1;
            we = ws + win_samp - 1;
            win_data = epoch(:, ws:we);   % 6 × win_samp

            % CCA
            data_filt = filtfilt(b_bp, a_bp, win_data')';
            cca_rho = zeros(1, numel(FREQS));
            for fi = 1:numel(FREQS)
                [~,~,r] = canoncorr(data_filt', refs{fi});
                cca_rho(fi) = r(1);
            end
            [~, cca_pred] = max(cca_rho);
            best_cca_score = max(best_cca_score, cca_rho);

            if isnan(cca_first_correct) && (cca_pred - 1) == cls
                t_win_end = (offsets(oi) + win_samp) / srate;
                cca_first_correct = t_win_end;
            end

            % FBCCA
            fb_score = zeros(1, numel(FREQS));
            for m = 1:NB
                sub = filtfilt(fb_b{m}, fb_a{m}, win_data')';
                for fi = 1:numel(FREQS)
                    [~,~,r] = canoncorr(sub', refs{fi});
                    fb_score(fi) = fb_score(fi) + fb_w(m) * r(1)^2;
                end
            end
            [~, fb_pred] = max(fb_score);
            best_fbcca_score = max(best_fbcca_score, fb_score);

            if isnan(fbcca_first_correct) && (fb_pred - 1) == cls
                t_win_end = (offsets(oi) + win_samp) / srate;
                fbcca_first_correct = t_win_end;
            end
        end

        cca_scores_all(tr,:)   = best_cca_score;
        fbcca_scores_all(tr,:) = best_fbcca_score;
        cca_latency(tr)        = cca_first_correct;
        fbcca_latency(tr)      = fbcca_first_correct;

        fprintf('  Trial %2d | class %d (%2d Hz) | CCA max %.3f (pred=%d Hz) | FBCCA max %.3f (pred=%d Hz)\n', ...
            tr, cls, FREQS(true_fi), ...
            best_cca_score(true_fi), FREQS(cca_scores_all(tr,:)==max(cca_scores_all(tr,:))), ...
            best_fbcca_score(true_fi), FREQS(fbcca_scores_all(tr,:)==max(fbcca_scores_all(tr,:))));
    end

    % ── 4f. Threshold sweep → accuracy + ITR curves ───────────────────────
    % CCA
    [cca_acc_curve, cca_itr_curve, cca_dec_rate_curve] = ...
        threshold_sweep(cca_scores_all, true_class, FREQS, CCA_THRESH_RANGE, WINDOW_SEC);

    % FBCCA
    [fbcca_acc_curve, fbcca_itr_curve, fbcca_dec_rate_curve] = ...
        threshold_sweep(fbcca_scores_all, true_class, FREQS, FBCCA_THRESH_RANGE, WINDOW_SEC);

    % ── 4g. Summary stats for this file ──────────────────────────────────
    % No-threshold accuracy: just argmax, every trial decided
    [~, cca_pred_idx]   = max(cca_scores_all,   [], 2);   % n_trials × 1
    [~, fbcca_pred_idx] = max(fbcca_scores_all, [], 2);

    cca_correct   = (cca_pred_idx'   - 1) == true_class;   % 0-indexed match
    fbcca_correct = (fbcca_pred_idx' - 1) == true_class;

    cca_acc_raw   = 100 * mean(cca_correct);
    fbcca_acc_raw = 100 * mean(fbcca_correct);

    % Peak ITR from sweep (highest ITR point on the curve)
    [cca_peak_itr,   cca_best_ti]   = max(cca_itr_curve);
    [fbcca_peak_itr, fbcca_best_ti] = max(fbcca_itr_curve);

    cca_peak_acc   = cca_acc_curve(cca_best_ti);
    fbcca_peak_acc = fbcca_acc_curve(fbcca_best_ti);

    % SNR: mean per frequency across trials (dB)
    mean_snr = mean(snr_epochs, 1, 'omitnan');
    snr_db   = 10*log10(mean_snr);

    % Responsiveness
    mean_cca_lat   = mean(cca_latency,   'omitnan');
    mean_fbcca_lat = mean(fbcca_latency, 'omitnan');
    pct_cca_detected   = 100 * mean(~isnan(cca_latency));
    pct_fbcca_detected = 100 * mean(~isnan(fbcca_latency));

    fprintf('\n  ── Results: %s ──\n', fstem);
    fprintf('  Raw accuracy (argmax, no threshold):\n');
    fprintf('    CCA:   %.1f%%  |  FBCCA:  %.1f%%\n', cca_acc_raw, fbcca_acc_raw);
    fprintf('  Peak ITR (threshold-optimised):\n');
    fprintf('    CCA:   %.1f bits/min @ %.1f%% acc (thresh=%.2f)\n', ...
        cca_peak_itr,   cca_peak_acc,   CCA_THRESH_RANGE(cca_best_ti));
    fprintf('    FBCCA: %.1f bits/min @ %.1f%% acc (thresh=%.2f)\n', ...
        fbcca_peak_itr, fbcca_peak_acc, FBCCA_THRESH_RANGE(fbcca_best_ti));
    fprintf('  SNR (dB, signal/flanking-bins, mean across trials):\n');
    for fi = 1:numel(FREQS)
        fprintf('    %2d Hz: %.1f dB\n', FREQS(fi), snr_db(fi));
    end
    fprintf('  Responsiveness (first correct window end-time from stim onset):\n');
    fprintf('    CCA:   %.2f s  (detected in %.0f%% of trials)\n', ...
        mean_cca_lat, pct_cca_detected);
    fprintf('    FBCCA: %.2f s  (detected in %.0f%% of trials)\n', ...
        mean_fbcca_lat, pct_fbcca_detected);
    fprintf('\n');

    % ── 4h. Store for pooled table ────────────────────────────────────────
    row.file           = fstem;
    row.n_trials       = n_trials_this;
    row.cca_acc_raw    = cca_acc_raw;
    row.fbcca_acc_raw  = fbcca_acc_raw;
    row.cca_peak_itr   = cca_peak_itr;
    row.fbcca_peak_itr = fbcca_peak_itr;
    row.cca_peak_acc   = cca_peak_acc;
    row.fbcca_peak_acc = fbcca_peak_acc;
    row.snr_16hz       = snr_db(1);
    row.snr_19hz       = snr_db(2);
    row.snr_23hz       = snr_db(3);
    row.snr_29hz       = snr_db(4);
    row.cca_latency_s  = mean_cca_lat;
    row.fbcca_latency_s= mean_fbcca_lat;
    all_results(end+1) = row; %#ok<AGROW>

    % ── 4h-ii. Append THIS file's per-trial data to the pooled set ────────
    POOL.true_class    = [POOL.true_class,    true_class];
    POOL.cca_scores    = [POOL.cca_scores;    cca_scores_all];
    POOL.fbcca_scores  = [POOL.fbcca_scores;  fbcca_scores_all];
    POOL.cca_latency   = [POOL.cca_latency,   cca_latency];
    POOL.fbcca_latency = [POOL.fbcca_latency, fbcca_latency];
    POOL.snr_epochs    = [POOL.snr_epochs;    snr_epochs];
    POOL.src_file      = [POOL.src_file, repmat({fstem}, 1, n_trials_this)];
    % Spectra: only pool if length matches (same srate/epoch across files).
    if isempty(POOL.freq_axis)
        POOL.freq_axis  = freq_axis;
        POOL.trial_spec = trial_spec;
    elseif numel(freq_axis) == numel(POOL.freq_axis)
        POOL.trial_spec = [POOL.trial_spec; trial_spec];
    else
        warning('  Spectrum length differs (srate/epoch mismatch) — not pooling spectra for %s.', fstem);
    end

    % ── 4i. Per-file plots ────────────────────────────────────────────────
    fig = figure('Name', fstem, 'NumberTitle', 'off', 'Position', [100 100 1400 900]);

    % -- Plot 1: ITR vs threshold curves --
    subplot(2,3,1);
    plot(CCA_THRESH_RANGE,   cca_itr_curve,   'b-o', 'MarkerSize', 4); hold on;
    xline(CCA_THRESH_RANGE(cca_best_ti), 'b--', 'LineWidth', 1);
    xlabel('CCA threshold (rho)'); ylabel('ITR (bits/min)');
    title('CCA: ITR vs Threshold'); grid on;
    legend(sprintf('Peak %.1f bits/min', cca_peak_itr), 'Best thresh');

    subplot(2,3,2);
    plot(FBCCA_THRESH_RANGE, fbcca_itr_curve, 'r-o', 'MarkerSize', 4); hold on;
    xline(FBCCA_THRESH_RANGE(fbcca_best_ti), 'r--', 'LineWidth', 1);
    xlabel('FBCCA threshold (score)'); ylabel('ITR (bits/min)');
    title('FBCCA: ITR vs Threshold'); grid on;
    legend(sprintf('Peak %.1f bits/min', fbcca_peak_itr), 'Best thresh');

    % -- Plot 2: Accuracy vs threshold --
    subplot(2,3,3);
    plot(CCA_THRESH_RANGE,   cca_acc_curve,   'b-', 'LineWidth', 1.5); hold on;
    plot(FBCCA_THRESH_RANGE ./ max(FBCCA_THRESH_RANGE), ...
         fbcca_acc_curve,  'r-', 'LineWidth', 1.5);
    xlabel('Normalised threshold'); ylabel('Accuracy (%)');
    title('Accuracy vs Threshold'); grid on;
    legend('CCA', 'FBCCA');

    % -- Plot 3: SNR bar chart --
    subplot(2,3,4);
    bar(FREQS, snr_db, 'FaceColor', [0.3 0.6 0.9]);
    xlabel('Frequency (Hz)'); ylabel('SNR (dB)');
    title('SNR per frequency'); grid on;
    xticklabels(arrayfun(@(f) sprintf('%d Hz',f), FREQS, 'UniformOutput', false));

    % -- Plot 4: Confusion matrix (no threshold) --
    subplot(2,3,5);
    C_cca = confusionmat(true_class, cca_pred_idx'-1);
    imagesc(C_cca);
    colormap(gca, parula);
    colorbar;
    xlabel('Predicted class'); ylabel('True class');
    title(sprintf('CCA Confusion Matrix (%.0f%%)', cca_acc_raw));
    set(gca, 'XTick', 1:4, 'YTick', 1:4, ...
        'XTickLabel', arrayfun(@(f) sprintf('%dHz',f), FREQS, 'UniformOutput', false), ...
        'YTickLabel', arrayfun(@(f) sprintf('%dHz',f), FREQS, 'UniformOutput', false));
    for r=1:4; for c=1:4
        text(c, r, num2str(C_cca(r,c)), 'HorizontalAlignment','center', ...
            'FontWeight','bold','Color','k');
    end; end

    subplot(2,3,6);
    C_fb = confusionmat(true_class, fbcca_pred_idx'-1);
    imagesc(C_fb);
    colormap(gca, hot);
    colorbar;
    xlabel('Predicted class'); ylabel('True class');
    title(sprintf('FBCCA Confusion Matrix (%.0f%%)', fbcca_acc_raw));
    set(gca, 'XTick', 1:4, 'YTick', 1:4, ...
        'XTickLabel', arrayfun(@(f) sprintf('%dHz',f), FREQS, 'UniformOutput', false), ...
        'YTickLabel', arrayfun(@(f) sprintf('%dHz',f), FREQS, 'UniformOutput', false));
    for r=1:4; for c=1:4
        text(c, r, num2str(C_fb(r,c)), 'HorizontalAlignment','center', ...
            'FontWeight','bold','Color','k');
    end; end

    sgtitle(sprintf('%s  |  CCA %.0f%%  FBCCA %.0f%%', fstem, cca_acc_raw, fbcca_acc_raw), ...
        'FontSize', 13, 'FontWeight', 'bold');

    % Responsiveness subplot (new figure — latency histogram)
    fig2 = figure('Name', [fstem ' — Responsiveness'], 'NumberTitle', 'off', ...
        'Position', [200 200 700 400]);

    all_lat = [cca_latency', fbcca_latency'];
    edges = WINDOW_SEC : STEP_SEC : STIM_MS/1000;
    subplot(1,2,1);
    histogram(cca_latency,   edges, 'FaceColor', 'b', 'FaceAlpha', 0.7);
    xlabel('Latency (s from stim onset)'); ylabel('Trials');
    title(sprintf('CCA responsiveness\nmean=%.2fs (%.0f%% detected)', ...
        mean_cca_lat, pct_cca_detected));
    grid on;

    subplot(1,2,2);
    histogram(fbcca_latency, edges, 'FaceColor', 'r', 'FaceAlpha', 0.7);
    xlabel('Latency (s from stim onset)'); ylabel('Trials');
    title(sprintf('FBCCA responsiveness\nmean=%.2fs (%.0f%% detected)', ...
        mean_fbcca_lat, pct_fbcca_detected));
    grid on;
    sgtitle([fstem ' — Time to first correct detection']);

end  % file loop

% ════════════════════════════════════════════════════════════════════════════
%% 5. POOLED SUMMARY TABLE
% ════════════════════════════════════════════════════════════════════════════

fprintf('══════════════════════════════════════════\n');
fprintf('POOLED SUMMARY — all %d recording(s)\n', numel(all_results));
fprintf('══════════════════════════════════════════\n\n');

T = struct2table(all_results);
disp(T);

if SAVE_CSV
    csv_path = fullfile(DATA_DIR, 'ssvep_results.csv');
    writetable(T, csv_path);
    fprintf('\nResults saved to: %s\n', csv_path);
end

% ════════════════════════════════════════════════════════════════════════════
%% 5b. COMBINED ANALYSIS — all trials from all files as ONE dataset
% ════════════════════════════════════════════════════════════════════════════

if ~isempty(POOL.true_class)
    nP = numel(POOL.true_class);
    fprintf('\n══════════════════════════════════════════\n');
    fprintf('COMBINED ANALYSIS — %d trials pooled across %d file(s)\n', ...
        nP, numel(all_results));
    fprintf('══════════════════════════════════════════\n');

    % ── Raw accuracy (argmax, every trial decided) ───────────────────────
    [~, cca_pred_P]   = max(POOL.cca_scores,   [], 2);
    [~, fbcca_pred_P] = max(POOL.fbcca_scores, [], 2);
    cca_corr_P   = (cca_pred_P'   - 1) == POOL.true_class;
    fbcca_corr_P = (fbcca_pred_P' - 1) == POOL.true_class;
    cca_acc_P    = 100 * mean(cca_corr_P);
    fbcca_acc_P  = 100 * mean(fbcca_corr_P);

    % ── Threshold sweep → peak ITR on the pooled set ─────────────────────
    [cca_acc_curveP, cca_itr_curveP] = ...
        threshold_sweep(POOL.cca_scores, POOL.true_class, FREQS, CCA_THRESH_RANGE, WINDOW_SEC);
    [fbcca_acc_curveP, fbcca_itr_curveP] = ...
        threshold_sweep(POOL.fbcca_scores, POOL.true_class, FREQS, FBCCA_THRESH_RANGE, WINDOW_SEC);
    [cca_peak_itrP,   cca_btiP]   = max(cca_itr_curveP);
    [fbcca_peak_itrP, fbcca_btiP] = max(fbcca_itr_curveP);

    % ── SNR pooled (mean per freq across all trials) ─────────────────────
    mean_snrP = mean(POOL.snr_epochs, 1, 'omitnan');
    snr_dbP   = 10*log10(mean_snrP);

    % ── Responsiveness pooled ────────────────────────────────────────────
    mean_cca_latP   = mean(POOL.cca_latency,   'omitnan');
    mean_fbcca_latP = mean(POOL.fbcca_latency, 'omitnan');
    sd_cca_latP     = std(POOL.cca_latency,   'omitnan');
    sd_fbcca_latP   = std(POOL.fbcca_latency, 'omitnan');
    pct_cca_detP    = 100 * mean(~isnan(POOL.cca_latency));
    pct_fbcca_detP  = 100 * mean(~isnan(POOL.fbcca_latency));

    fprintf('\nClassification accuracy (argmax, no threshold):\n');
    fprintf('  CCA:   %.1f%%  (%d/%d)\n', cca_acc_P,   sum(cca_corr_P),   nP);
    fprintf('  FBCCA: %.1f%%  (%d/%d)\n', fbcca_acc_P, sum(fbcca_corr_P), nP);
    fprintf('\nPeak ITR (threshold-optimised):\n');
    fprintf('  CCA:   %.1f bits/min @ %.1f%% acc (thresh=%.2f)\n', ...
        cca_peak_itrP,   cca_acc_curveP(cca_btiP),   CCA_THRESH_RANGE(cca_btiP));
    fprintf('  FBCCA: %.1f bits/min @ %.1f%% acc (thresh=%.2f)\n', ...
        fbcca_peak_itrP, fbcca_acc_curveP(fbcca_btiP), FBCCA_THRESH_RANGE(fbcca_btiP));
    fprintf('\nSNR (dB, mean across %d trials):\n', nP);
    for fi = 1:numel(FREQS)
        fprintf('  %2d Hz: %.1f dB\n', FREQS(fi), snr_dbP(fi));
    end
    fprintf('\nResponsiveness (first correct window end-time from stim onset):\n');
    fprintf('  CCA:   %.2f ± %.2f s  (detected in %.0f%% of trials)\n', ...
        mean_cca_latP, sd_cca_latP, pct_cca_detP);
    fprintf('  FBCCA: %.2f ± %.2f s  (detected in %.0f%% of trials)\n', ...
        mean_fbcca_latP, sd_fbcca_latP, pct_fbcca_detP);

    % ── Per-class accuracy breakdown ─────────────────────────────────────
    fprintf('\nPer-class accuracy:\n');
    fprintf('  Freq    CCA      FBCCA    n\n');
    for fi = 1:numel(FREQS)
        mask = POOL.true_class == (fi-1);
        n_cls = sum(mask);
        cca_cls   = 100 * mean(cca_corr_P(mask));
        fbcca_cls = 100 * mean(fbcca_corr_P(mask));
        fprintf('  %2d Hz   %5.1f%%   %5.1f%%   %d\n', ...
            FREQS(fi), cca_cls, fbcca_cls, n_cls);
    end

    % ── PER-FREQUENCY SPECTRUM DIAGNOSTIC ────────────────────────────────
    % For each target frequency, average the power spectrum across all trials
    % of THAT class, zoom around the target, and mark the target bin (green)
    % and the flanking bins used in the SNR denominator (red). This shows at a
    % glance whether a weak SNR is a missing peak, a shifted peak, or tall
    % flanking bins (intermodulation/noise inflating the denominator).
    if ~isempty(POOL.trial_spec)
        figure('Name','COMBINED — per-frequency spectra','NumberTitle','off', ...
            'Position',[140 100 1400 900]);
        fr = POOL.freq_axis;
        for fi = 1:numel(FREQS)
            f0 = FREQS(fi);
            mask = POOL.true_class == (fi-1);
            spec_avg = mean(POOL.trial_spec(mask,:), 1, 'omitnan');

            % Convert to dB for display (relative to median, robust).
            spec_db = 10*log10(spec_avg / median(spec_avg));

            subplot(2,2,fi);
            % Zoom window: ±6 Hz around target.
            win_lo = f0 - 6; win_hi = f0 + 6;
            sel = fr >= win_lo & fr <= win_hi;
            plot(fr(sel), spec_db(sel), 'k-', 'LineWidth', 1.2); hold on;

            % Mark target bin.
            freq_res = fr(2) - fr(1);
            tb = round(f0 / freq_res) + 1;
            plot(fr(tb), spec_db(tb), 'go', 'MarkerSize', 10, ...
                'MarkerFaceColor', 'g');

            % Mark flank bins (SNR denominator).
            flank = [tb-SNR_FLANK_BINS:tb-1, tb+1:tb+SNR_FLANK_BINS];
            flank = flank(flank>=1 & flank<=numel(spec_db));
            plot(fr(flank), spec_db(flank), 'rs', 'MarkerSize', 8, ...
                'MarkerFaceColor', 'r');

            % Mark the other three targets if they fall in the window (to spot
            % cross-frequency leakage sitting in this band).
            for fj = 1:numel(FREQS)
                if fj==fi, continue; end
                if FREQS(fj) >= win_lo && FREQS(fj) <= win_hi
                    xline(FREQS(fj), 'b:', sprintf('%dHz',FREQS(fj)), ...
                        'LabelOrientation','horizontal');
                end
            end

            xlabel('Frequency (Hz)'); ylabel('Power (dB re median)');
            title(sprintf('%d Hz class  (SNR %.1f dB)', f0, snr_dbP(fi)));
            grid on; xlim([win_lo win_hi]);
            if fi==1
                legend('spectrum','target bin','flank bins', ...
                    'Location','best','FontSize',8);
            end
        end
        sgtitle('Per-frequency averaged spectra (green=target, red=SNR flanks)', ...
            'FontSize',13,'FontWeight','bold');
    end

    % ── COMBINED FIGURE ──────────────────────────────────────────────────
    figure('Name','COMBINED — all recordings', 'NumberTitle','off', ...
        'Position',[120 80 1500 900]);

    % ITR curves
    subplot(2,3,1);
    plot(CCA_THRESH_RANGE, cca_itr_curveP, 'b-o', 'MarkerSize',4); hold on;
    xline(CCA_THRESH_RANGE(cca_btiP), 'b--');
    xlabel('CCA threshold (\rho)'); ylabel('ITR (bits/min)');
    title('Combined CCA: ITR vs Threshold'); grid on;
    legend(sprintf('Peak %.1f', cca_peak_itrP), 'Location','best');

    subplot(2,3,2);
    plot(FBCCA_THRESH_RANGE, fbcca_itr_curveP, 'r-o', 'MarkerSize',4); hold on;
    xline(FBCCA_THRESH_RANGE(fbcca_btiP), 'r--');
    xlabel('FBCCA threshold (score)'); ylabel('ITR (bits/min)');
    title('Combined FBCCA: ITR vs Threshold'); grid on;
    legend(sprintf('Peak %.1f', fbcca_peak_itrP), 'Location','best');

    % SNR bar
    subplot(2,3,3);
    bar(FREQS, snr_dbP, 'FaceColor',[0.3 0.6 0.9]);
    xlabel('Frequency (Hz)'); ylabel('SNR (dB)');
    title(sprintf('Combined SNR (n=%d)', nP)); grid on;

    % CCA confusion (pooled)
    subplot(2,3,4);
    Cc = confusionmat(POOL.true_class, cca_pred_P'-1);
    imagesc(Cc); colormap(gca, parula); colorbar;
    xlabel('Predicted'); ylabel('True');
    title(sprintf('Combined CCA (%.0f%%)', cca_acc_P));
    set(gca,'XTick',1:4,'YTick',1:4, ...
        'XTickLabel',arrayfun(@(f)sprintf('%dHz',f),FREQS,'UniformOutput',false), ...
        'YTickLabel',arrayfun(@(f)sprintf('%dHz',f),FREQS,'UniformOutput',false));
    for r=1:4; for c=1:4
        text(c,r,num2str(Cc(r,c)),'HorizontalAlignment','center', ...
            'FontWeight','bold','Color','k'); end; end

    % FBCCA confusion (pooled)
    subplot(2,3,5);
    Cf = confusionmat(POOL.true_class, fbcca_pred_P'-1);
    imagesc(Cf); colormap(gca, hot); colorbar;
    xlabel('Predicted'); ylabel('True');
    title(sprintf('Combined FBCCA (%.0f%%)', fbcca_acc_P));
    set(gca,'XTick',1:4,'YTick',1:4, ...
        'XTickLabel',arrayfun(@(f)sprintf('%dHz',f),FREQS,'UniformOutput',false), ...
        'YTickLabel',arrayfun(@(f)sprintf('%dHz',f),FREQS,'UniformOutput',false));
    for r=1:4; for c=1:4
        text(c,r,num2str(Cf(r,c)),'HorizontalAlignment','center', ...
            'FontWeight','bold','Color','k'); end; end

    % Responsiveness histogram (pooled)
    subplot(2,3,6);
    edgesP = WINDOW_SEC : STEP_SEC : STIM_MS/1000;
    histogram(POOL.cca_latency, edgesP, 'FaceColor','b','FaceAlpha',0.5); hold on;
    histogram(POOL.fbcca_latency, edgesP, 'FaceColor','r','FaceAlpha',0.5);
    xlabel('Latency (s)'); ylabel('Trials');
    title('Combined responsiveness');
    legend(sprintf('CCA %.2fs',mean_cca_latP), ...
           sprintf('FBCCA %.2fs',mean_fbcca_latP), 'Location','best');
    grid on;

    sgtitle(sprintf('COMBINED (%d trials)  |  CCA %.0f%%  FBCCA %.0f%%', ...
        nP, cca_acc_P, fbcca_acc_P), 'FontSize',14,'FontWeight','bold');

    % ── THRESHOLDED confusion matrices (with 'No decision' column) ───────
    % Pick the operating threshold = lowest swept value beating 25% chance.
    cca_thr   = pick_threshold_above_chance(POOL.cca_scores,   POOL.true_class, ...
                    FREQS, CCA_THRESH_RANGE,   CM_MIN_DECIDED_FRAC);
    fbcca_thr = pick_threshold_above_chance(POOL.fbcca_scores, POOL.true_class, ...
                    FREQS, FBCCA_THRESH_RANGE, CM_MIN_DECIDED_FRAC);

    C5_cca = thresholded_confusion(POOL.cca_scores,   POOL.true_class, numel(FREQS), cca_thr);
    C5_fb  = thresholded_confusion(POOL.fbcca_scores, POOL.true_class, numel(FREQS), fbcca_thr);

    % Accuracy over DECIDED trials only, and abstain rate.
    cca_decided   = sum(C5_cca(:,1:end-1), 'all');
    cca_correct_d = sum(diag(C5_cca(:,1:end-1)));
    cca_acc_dec   = 100 * cca_correct_d / max(1, cca_decided);
    cca_abstain   = 100 * sum(C5_cca(:,end)) / nP;

    fb_decided    = sum(C5_fb(:,1:end-1), 'all');
    fb_correct_d  = sum(diag(C5_fb(:,1:end-1)));
    fb_acc_dec    = 100 * fb_correct_d / max(1, fb_decided);
    fb_abstain    = 100 * sum(C5_fb(:,end)) / nP;

    figure('Name','COMBINED — thresholded confusion','NumberTitle','off', ...
        'Position',[160 120 1200 480]);
    subplot(1,2,1);
    plot_thresholded_cm(C5_cca, FREQS, cca_thr, 'parula', ...
        sprintf('CCA  |  %.0f%% acc on decided, %.0f%% abstain', cca_acc_dec, cca_abstain));
    subplot(1,2,2);
    plot_thresholded_cm(C5_fb, FREQS, fbcca_thr, 'hot', ...
        sprintf('FBCCA  |  %.0f%% acc on decided, %.0f%% abstain', fb_acc_dec, fb_abstain));
    sgtitle('Thresholded confusion (threshold = lowest beating 25% chance)', ...
        'FontSize',13,'FontWeight','bold');

    fprintf('\nThresholded confusion (lowest threshold beating chance):\n');
    fprintf('  CCA:   thr=%.2f  ->  %.0f%% acc on decided, %.0f%% abstain\n', ...
        cca_thr, cca_acc_dec, cca_abstain);
    fprintf('  FBCCA: thr=%.2f  ->  %.0f%% acc on decided, %.0f%% abstain\n', ...
        fbcca_thr, fb_acc_dec, fb_abstain);

    % ── Write combined summary CSV ───────────────────────────────────────
    if SAVE_CSV
        combined = struct();
        combined.dataset       = 'COMBINED';
        combined.n_trials      = nP;
        combined.cca_acc       = cca_acc_P;
        combined.fbcca_acc     = fbcca_acc_P;
        combined.cca_peak_itr  = cca_peak_itrP;
        combined.fbcca_peak_itr= fbcca_peak_itrP;
        combined.snr_16hz      = snr_dbP(1);
        combined.snr_19hz      = snr_dbP(2);
        combined.snr_23hz      = snr_dbP(3);
        combined.snr_29hz      = snr_dbP(4);
        combined.cca_latency_s   = mean_cca_latP;
        combined.fbcca_latency_s = mean_fbcca_latP;
        Tc = struct2table(combined);
        csv_path2 = fullfile(DATA_DIR, 'ssvep_results_combined.csv');
        writetable(Tc, csv_path2);
        fprintf('\nCombined results saved to: %s\n', csv_path2);
    end
end

% ════════════════════════════════════════════════════════════════════════════
%% 6. POOLED PLOTS (across files)
% ════════════════════════════════════════════════════════════════════════════

if numel(all_results) > 1
    files = {all_results.file};
    x = 1:numel(files);
    w = 0.35;

    figure('Name','Pooled: Accuracy','Position',[300 300 800 400]);
    bar(x - w/2, [all_results.cca_acc_raw],   w, 'FaceColor', 'b', 'FaceAlpha', 0.7); hold on;
    bar(x + w/2, [all_results.fbcca_acc_raw],  w, 'FaceColor', 'r', 'FaceAlpha', 0.7);
    xticks(x); xticklabels(files); xtickangle(30);
    ylabel('Accuracy (%)'); ylim([0 105]);
    legend('CCA','FBCCA'); title('Raw classification accuracy (no threshold)');
    yline(25,'k--','Chance','LabelHorizontalAlignment','right');
    grid on;

    figure('Name','Pooled: Peak ITR','Position',[350 300 800 400]);
    bar(x - w/2, [all_results.cca_peak_itr],   w, 'FaceColor', 'b', 'FaceAlpha', 0.7); hold on;
    bar(x + w/2, [all_results.fbcca_peak_itr],  w, 'FaceColor', 'r', 'FaceAlpha', 0.7);
    xticks(x); xticklabels(files); xtickangle(30);
    ylabel('ITR (bits/min)'); legend('CCA','FBCCA');
    title('Peak ITR (threshold-optimised)'); grid on;

    figure('Name','Pooled: SNR','Position',[400 300 800 400]);
    snr_mat = [[all_results.snr_16hz]', [all_results.snr_19hz]', ...
                [all_results.snr_23hz]', [all_results.snr_29hz]'];
    bar(x, snr_mat, 'grouped');
    xticks(x); xticklabels(files); xtickangle(30);
    ylabel('SNR (dB)');
    legend(arrayfun(@(f) sprintf('%d Hz',f), FREQS, 'UniformOutput', false));
    title('SNR per frequency per recording'); grid on;
end

fprintf('\nDone.\n');

% ════════════════════════════════════════════════════════════════════════════
%% LOCAL FUNCTIONS
% ════════════════════════════════════════════════════════════════════════════

function itr = calc_itr(acc, n_classes, window_sec)
    % Wolpaw et al. (2000) ITR formula.
    % acc: proportion correct (0-1); n_classes: number of targets.
    % Returns ITR in bits/min.
    % Reference: Wolpaw JR et al., IEEE Trans Rehab Eng, 2000.
    %            https://doi.org/10.1109/86.847823
    acc = max(0, min(1, acc));
    if acc == 1
        B = log2(n_classes);
    elseif acc == 0
        B = log2(n_classes) + 0*log2(1) + (1-0)*log2((1-0)/(n_classes-1));
        B = 0;
    else
        B = log2(n_classes) + acc*log2(acc) + (1-acc)*log2((1-acc)/(n_classes-1));
    end
    trials_per_min = 60 / window_sec;
    itr = B * trials_per_min;
end

function [acc_curve, itr_curve, dec_rate_curve] = ...
        threshold_sweep(scores_all, true_class, freqs, thresh_range, window_sec)
    % scores_all: n_trials × n_freqs (best score per freq per trial)
    % true_class: 1 × n_trials, 0-indexed class labels
    n_trials  = size(scores_all, 1);
    n_classes = numel(freqs);
    acc_curve      = zeros(1, numel(thresh_range));
    itr_curve      = zeros(1, numel(thresh_range));
    dec_rate_curve = zeros(1, numel(thresh_range));

    for ti = 1:numel(thresh_range)
        thr = thresh_range(ti);
        [max_score, pred_idx] = max(scores_all, [], 2);
        decided = max_score >= thr;   % logical n_trials × 1

        n_dec = sum(decided);
        dec_rate_curve(ti) = n_dec / n_trials;

        if n_dec == 0
            acc_curve(ti) = 0;
            itr_curve(ti) = 0;
            continue
        end

        correct = (pred_idx(decided)' - 1) == true_class(decided);
        p_correct = mean(correct);
        acc_curve(ti) = 100 * p_correct;

        % ITR accounts for non-decisions: effective accuracy over ALL trials
        % (undecided trials count as wrong for throughput).
        eff_acc = sum(correct) / n_trials;
        itr_curve(ti) = calc_itr(eff_acc, n_classes, window_sec);
    end
end

function thr = pick_threshold_above_chance(scores_all, true_class, ...
        freqs, thresh_range, min_decided_frac)
    % Return the LOWEST threshold in thresh_range whose accuracy over the
    % DECIDED trials clears chance (1/n_classes), while still deciding at
    % least min_decided_frac of trials. Falls back to the lowest threshold
    % if none qualifies.
    n_classes = numel(freqs);
    chance    = 1 / n_classes;
    n_trials  = size(scores_all, 1);

    thr = thresh_range(1);   % fallback
    for ti = 1:numel(thresh_range)
        t = thresh_range(ti);
        [max_score, pred_idx] = max(scores_all, [], 2);
        decided = max_score >= t;
        if sum(decided) < max(1, round(min_decided_frac * n_trials))
            continue
        end
        correct = (pred_idx(decided)' - 1) == true_class(decided);
        if mean(correct) > chance
            thr = t;
            return
        end
    end
end

function C5 = thresholded_confusion(scores_all, true_class, n_classes, thr)
    % Build an n_classes × (n_classes+1) confusion matrix where the final
    % column counts trials that fell BELOW thr ("No decision").
    % Rows = true class (1..n_classes), cols = predicted (1..n_classes) + abstain.
    C5 = zeros(n_classes, n_classes + 1);
    [max_score, pred_idx] = max(scores_all, [], 2);
    for k = 1:numel(true_class)
        tr = true_class(k) + 1;        % 1-indexed true class
        if max_score(k) >= thr
            pc = pred_idx(k);          % predicted class
            C5(tr, pc) = C5(tr, pc) + 1;
        else
            C5(tr, end) = C5(tr, end) + 1;   % abstain column
        end
    end
end

function plot_thresholded_cm(C5, freqs, thr, cmap_name, ttl)
    % Render a 4×5 thresholded confusion matrix with a 'No dec.' column.
    n_classes = numel(freqs);
    imagesc(C5);
    if strcmpi(cmap_name, 'parula'); colormap(gca, parula); else; colormap(gca, hot); end
    colorbar;
    xlabel('Predicted'); ylabel('True');
    title(sprintf('%s (thr=%.2f)', ttl, thr));
    xt_labels = [arrayfun(@(f) sprintf('%dHz',f), freqs, 'UniformOutput', false), {'No dec.'}];
    yt_labels = arrayfun(@(f) sprintf('%dHz',f), freqs, 'UniformOutput', false);
    set(gca, 'XTick', 1:(n_classes+1), 'XTickLabel', xt_labels, ...
             'YTick', 1:n_classes,     'YTickLabel', yt_labels);
    for r = 1:n_classes
        for c = 1:(n_classes+1)
            text(c, r, num2str(C5(r,c)), 'HorizontalAlignment','center', ...
                'FontWeight','bold', 'Color','k');
        end
    end
    % Separate the abstain column visually.
    hold on;
    xline(n_classes + 0.5, 'w-', 'LineWidth', 2);
    hold off;
end

% ════════════════════════════════════════════════════════════════════════════
%% BRAINVISION READER (no toolbox needed)
% ════════════════════════════════════════════════════════════════════════════

function [data, srate, ch_names, markers] = bv_load(vhdr_path)
    % Minimal BrainVision reader: reads .vhdr, then binary .eeg.
    % Returns:
    %   data     : n_channels × n_samples (double, µV)
    %   srate    : sampling rate (Hz)
    %   ch_names : cell array of channel name strings
    %   markers  : struct array with fields .type .desc .sample

    hdr = bv_parse_vhdr(vhdr_path);
    srate     = hdr.srate;
    ch_names  = hdr.ch_names;
    n_ch      = hdr.n_ch;
    resolutions = hdr.resolutions;

    % Binary file lives next to the .vhdr
    [folder, stem, ~] = fileparts(vhdr_path);
    eeg_file = fullfile(folder, hdr.data_file);
    if ~isfile(eeg_file)
        % Try same stem with .eeg extension (some recorders lowercase it)
        eeg_file = fullfile(folder, [stem '.eeg']);
    end

    fid = fopen(eeg_file, 'rb', 'l');   % little-endian
    if fid < 0
        error('Cannot open EEG file: %s', eeg_file);
    end

    % Pick precision from the .vhdr [Binary Infos] BinaryFormat field.
    switch upper(hdr.bin_format)
        case {'IEEE_FLOAT_32', 'FLOAT_32', 'FLOAT'}
            precision = 'float32';
        case {'INT_16', 'INT16'}
            precision = 'int16';
        case {'INT_32', 'INT32'}
            precision = 'int32';
        otherwise
            precision = 'int16';   % BrainVision default
            warning('Unknown BinaryFormat "%s" — assuming int16.', hdr.bin_format);
    end
    fprintf('  [data diag] binary format: %s (%s), orientation: %s\n', ...
        hdr.bin_format, precision, hdr.orientation);

    if strcmpi(hdr.orientation, 'VECTORIZED')
        % All samples of ch1, then all of ch2, ... → read as [T × n_ch] then transpose
        raw = fread(fid, [Inf, n_ch], precision);  % wrong shape on purpose
        % Actually vectorized = channel-contiguous: read flat then reshape.
        % (Re-read properly below.)
        frewind(fid);
        flat = fread(fid, Inf, precision);
        nsamp = floor(numel(flat) / n_ch);
        raw = reshape(flat(1:nsamp*n_ch), nsamp, n_ch)';  % n_ch × T
    else
        % MULTIPLEXED (default): ch1s1, ch2s1, ..., chNs1, ch1s2, ...
        raw = fread(fid, [n_ch, Inf], precision);
    end
    fclose(fid);

    % Float formats are already in µV — do NOT apply int resolution scaling.
    if strcmpi(precision, 'float32')
        data = raw;
    else
        data = raw .* resolutions(:);   % int → µV via per-channel resolution
    end

    % Read markers
    vmrk_file = fullfile(folder, hdr.marker_file);
    if isfile(vmrk_file)
        markers = bv_parse_vmrk(vmrk_file);
    else
        markers = struct('type','','desc','','sample',{});
        warning('Marker file not found: %s', vmrk_file);
    end
end

function hdr = bv_parse_vhdr(vhdr_path)
    fid = fopen(vhdr_path, 'r', 'n', 'UTF-8');
    lines = {};
    while ~feof(fid)
        lines{end+1} = strtrim(fgetl(fid)); %#ok<AGROW>
    end
    fclose(fid);

    hdr.data_file   = '';
    hdr.marker_file = '';
    hdr.n_ch        = 0;
    hdr.srate       = 0;
    hdr.ch_names    = {};
    hdr.resolutions = [];
    hdr.bin_format  = 'INT_16';        % default per BrainVision spec
    hdr.orientation = 'MULTIPLEXED';   % default

    section = '';
    ch_idx  = 0;

    for li = 1:numel(lines)
        ln = lines{li};
        if isempty(ln) || ln(1) == ';', continue; end

        % Section header
        if ln(1) == '[' && ln(end) == ']'
            section = ln(2:end-1);
            continue
        end

        if ~contains(ln, '='), continue; end
        eq_pos = strfind(ln, '=');
        key    = strtrim(ln(1:eq_pos(1)-1));
        val    = strtrim(ln(eq_pos(1)+1:end));

        switch section
            case 'Common Infos'
                if strcmpi(key, 'DataFile'),          hdr.data_file   = val; end
                if strcmpi(key, 'MarkerFile'),        hdr.marker_file = val; end
                if strcmpi(key, 'NumberOfChannels'),  hdr.n_ch        = str2double(val); end
                if strcmpi(key, 'DataOrientation'),   hdr.orientation = val; end
                if strcmpi(key, 'SamplingInterval')
                    hdr.srate = round(1e6 / str2double(val));  % µs → Hz
                end

            case 'Binary Infos'
                if strcmpi(key, 'BinaryFormat'),      hdr.bin_format  = val; end

            case 'Channel Infos'
                % Format: Ch1=Fp1,,0.1,µV
                parts2 = strsplit(val, ',');
                ch_idx = ch_idx + 1;
                if numel(parts2) >= 1
                    hdr.ch_names{ch_idx} = parts2{1};
                else
                    hdr.ch_names{ch_idx} = sprintf('Ch%d', ch_idx);
                end
                if numel(parts2) >= 3 && ~isempty(parts2{3})
                    hdr.resolutions(ch_idx) = str2double(parts2{3});
                else
                    hdr.resolutions(ch_idx) = 0.1;   % default 0.1 µV/bit
                end
        end
    end

    if hdr.n_ch == 0 || hdr.srate == 0
        error('Could not parse channel count or sampling rate from %s', vhdr_path);
    end
    hdr.resolutions = hdr.resolutions(:);
end

function markers = bv_parse_vmrk(vmrk_path)
    fid = fopen(vmrk_path, 'r', 'n', 'UTF-8');
    lines = {};
    while ~feof(fid)
        lines{end+1} = strtrim(fgetl(fid)); %#ok<AGROW>
    end
    fclose(fid);

    markers = struct('type', {}, 'desc', {}, 'sample', {});
    in_markers = false;

    for li = 1:numel(lines)
        ln = lines{li};
        if isempty(ln) || ln(1) == ';', continue; end
        if strcmpi(ln, '[Marker Infos]'), in_markers = true; continue; end
        if ln(1) == '[', in_markers = false; continue; end
        if ~in_markers, continue; end
        if ~contains(ln, '='), continue; end

        % Mk<n>=<type>,<desc>,<sample>,<length>,<offset>
        eq = strfind(ln, '=');
        val = ln(eq(1)+1:end);
        parts = strsplit(val, ',');
        if numel(parts) < 3, continue; end

        mk.type   = strtrim(parts{1});
        mk.desc   = strtrim(parts{2});
        mk.sample = str2double(strtrim(parts{3}));
        if ~isnan(mk.sample)
            markers(end+1) = mk; %#ok<AGROW>
        end
    end
end

function trig_samples = bv_find_triggers(markers, srate) %#ok<INUSD>
    % Return sample indices for stimulus/trigger markers.
    % BrainVision marker types vary by recorder config. The TTL pulse may be
    % logged as 'Stimulus', 'Trigger', 'Response', etc. We accept any marker
    % EXCEPT the boilerplate ones ('New Segment', 'DC Correction') and print a
    % breakdown so mismatches are obvious.

    % First, print a summary of what marker types exist.
    if isempty(markers)
        fprintf('  [marker diag] .vmrk parsed but contains NO markers.\n');
        trig_samples = [];
        return
    end

    all_types = {markers.type};
    all_descs = {markers.desc};
    uniq_types = unique(all_types);
    fprintf('  [marker diag] %d markers, types present: %s\n', ...
        numel(markers), strjoin(uniq_types, ', '));
    % Show the unique descriptions too (helps spot S 1 / S  1 / etc.)
    uniq_descs = unique(all_descs);
    fprintf('  [marker diag] unique descriptions: %s\n', ...
        strjoin(uniq_descs, ' | '));

    % Boilerplate marker types/descriptions that are NOT trial triggers.
    skip_types = {'New Segment', 'DC Correction', ''};

    trig_samples = [];
    for mi = 1:numel(markers)
        ty = markers(mi).type;
        if any(strcmpi(ty, skip_types))
            continue
        end
        % Skip the very first "New Segment"-style marker even if mislabelled
        % (sample 1 boilerplate).
        if markers(mi).sample <= 1
            continue
        end
        trig_samples(end+1) = markers(mi).sample; %#ok<AGROW>
    end
    trig_samples = sort(trig_samples);
    fprintf('  [marker diag] %d trigger candidates after filtering.\n', ...
        numel(trig_samples));
end
