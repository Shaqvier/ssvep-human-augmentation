%% SSVEP Combined Analysis — BAND references (±0.5 Hz), pooled, single figure
% Variant of ssvep_combined_measured.m that replaces each single-frequency
% reference with a BAND of candidate references spanning nominal ±BAND_HALF Hz.
% For each class, CCA/FBCCA is computed against every candidate in the band and
% the MAX score is kept. This absorbs the per-LED-group timing offset (the
% non-uniform ~0.25 Hz shifts seen in the spectra) WITHOUT hard-coding a single
% correction and WITHOUT per-subject calibration — the decoder simply locks
% onto whichever sub-frequency the EEG actually contains.
%
% SNR uses a peak-search over the same band (reports the true peak, and the
% measured peak frequency per class).
%
% NOMINAL frequencies: 16 / 19 / 23 / 29 Hz, band ±0.5 Hz.
%   Tightest class spacing is 3 Hz (16->19), so ±0.5 Hz bands cannot overlap.
%
% Pooled across ALL recordings in the folder. Single combined figure only.
%
%   ssvep_combined_band(FOLDER)
%
% FOLDER may be one participant's offline folder (holding .vhdr recordings
% directly) or a master folder containing many at any depth; in the second case
% every offline folder underneath is analysed in turn. Each folder produces one
% saved PNG, SSVEP_combined_<ID>.png, and nothing is displayed on screen.

function ssvep_combined_band(DATA_DIR)

if nargin < 1 || isempty(DATA_DIR), DATA_DIR = '.'; end

% ── master folder? analyse each offline folder underneath, then return ─────
if isempty(dir(fullfile(DATA_DIR, '*.vhdr')))
    vh = dir(fullfile(DATA_DIR, '**', '*.vhdr'));
    if isempty(vh)
        error('No .vhdr files found in %s or any folder beneath it.', DATA_DIR);
    end
    subs = sort(unique(string({vh.folder})));
    fprintf('Master folder: %d offline folder(s) found.\n', numel(subs));
    for ii = 1:numel(subs)
        fprintf('\n[%d/%d] %s\n%s\n', ii, numel(subs), subs(ii), repmat('=',1,64));
        try
            ssvep_combined_band(char(subs(ii)));
        catch ME
            warning('%s failed: %s', subs(ii), ME.message);
        end
    end
    fprintf('\nDone: %d folder(s).\n', numel(subs));
    return;
end

% participant label from the folder name, or its parent when named "Offline"
pth = DATA_DIR; if endsWith(pth, filesep), pth(end) = []; end
[upDir, PID] = fileparts(pth);
if any(strcmpi(PID, {'offline','data','recordings','eeg'}))
    [~, PID] = fileparts(upDir);
end
PID = regexprep(PID, '[^\w\- ]', '_');
if isempty(PID), PID = 'participant'; end

% ════════════════════════════════════════════════════════════════════════════
%% CONFIG
% ════════════════════════════════════════════════════════════════════════════

% NOMINAL frequencies (Hz), class 0..3.
FREQS     = [16 19 23 29];

% Band half-width (Hz) and step. References are built at
% FREQS(fi) + (-BAND_HALF : BAND_STEP : +BAND_HALF).
BAND_HALF = 0.5;
BAND_STEP = 0.25;     % 0.25 Hz steps -> {-0.5,-0.25,0,+0.25,+0.5} = 5 refs/class

% Paradigm
TRIAL_SEQ = [0,2,1,3,2,0,3,1,2,3,1,0,3,2,0,3,1,2,0,1];
N_TRIALS  = 20;
CUE_MS    = 1500;   % TTL fires at cue onset; flicker starts CUE_MS later
                    % (matches CUE_MS in SSVEP_Offline_Experiment.ino)
STIM_MS   = 4000;

ANALYSIS_CH_NAMES = {'Pz','P3','O1','Oz','O2','P4'};

WINDOW_SEC  = 2.0;
STEP_SEC    = 0.25;
N_HARMONICS = 3;

NB = 5; FB_A = 1.25; FB_B = 0.25; FB_HIGH = 88; BUTTER_ORDER = 4;

SNR_FLANK_BINS = 2;
SNR_PEAK_HALF  = 0.5;   % Hz; peak-search half-window for SNR target

% ════════════════════════════════════════════════════════════════════════════
%% FIND FILES
% ════════════════════════════════════════════════════════════════════════════

vhdr_files = dir(fullfile(DATA_DIR, '*.vhdr'));
if isempty(vhdr_files)
    error('No .vhdr files found in: %s', DATA_DIR);
end
fprintf('Found %d recording(s).\n', numel(vhdr_files));

% Per-class band offsets (shared across files)
band_offsets = -BAND_HALF : BAND_STEP : BAND_HALF;
fprintf('Band: nominal ± %.2f Hz in %.2f Hz steps -> %d refs/class: [%s]\n', ...
    BAND_HALF, BAND_STEP, numel(band_offsets), ...
    strjoin(arrayfun(@(o) sprintf('%+.2f',o), band_offsets, 'UniformOutput', false), ' '));

% Pooled accumulators
P_true    = [];   % 1 × N
P_cca     = [];   % N × nFreq  (max rho over band)
P_fbcca   = [];   % N × nFreq  (max score over band)
P_snr     = [];   % N × nFreq  (peak-search SNR, linear)
P_peakhz  = [];   % N × nFreq  (measured peak frequency)
P_spec    = [];   % N × specLen
P_fr      = [];   % 1 × specLen

% ════════════════════════════════════════════════════════════════════════════
%% MAIN LOOP
% ════════════════════════════════════════════════════════════════════════════

for fk = 1:numel(vhdr_files)
    fname = vhdr_files(fk).name;
    fpath = fullfile(DATA_DIR, fname);
    fprintf('Loading %s ... ', fname);

    [eeg_full, srate, ch_names, markers] = bv_load(fpath);

    ch_idx = zeros(1, numel(ANALYSIS_CH_NAMES));
    ok = true;
    for ci = 1:numel(ANALYSIS_CH_NAMES)
        hit = find(strcmpi(ch_names, ANALYSIS_CH_NAMES{ci}), 1);
        if isempty(hit), ok = false; break; end
        ch_idx(ci) = hit;
    end
    if ~ok, fprintf('channel mismatch — skipped.\n'); continue; end
    data_occ = eeg_full(ch_idx, :);

    for ch = 1:size(data_occ,1)
        x = data_occ(ch,:); bad = ~isfinite(x);
        if any(bad) && ~all(bad)
            x(bad) = interp1(find(~bad), x(~bad), find(bad), 'linear', 'extrap');
            data_occ(ch,:) = x;
        elseif all(bad)
            data_occ(ch,:) = 0;
        end
    end

    trig = bv_find_triggers(markers, srate);
    if isempty(trig), fprintf('no triggers — skipped.\n'); continue; end
    nT = min(numel(trig), N_TRIALS);
    onset = trig(1:nT) + round(CUE_MS/1000 * srate);
    true_class = TRIAL_SEQ(1:nT);

    win_samp  = round(WINDOW_SEC * srate);
    step_samp = round(STEP_SEC   * srate);
    stim_samp = round(STIM_MS/1000 * srate);
    nyq = srate/2;
    t_ref = (0:win_samp-1)/srate;

    % Build a band of references per class: refs{fi}{bi}
    n_band = numel(band_offsets);
    refs = cell(1, numel(FREQS));
    for fi = 1:numel(FREQS)
        refs{fi} = cell(1, n_band);
        for bi = 1:n_band
            f = FREQS(fi) + band_offsets(bi);
            R = [];
            for h = 1:N_HARMONICS
                R = [R; sin(2*pi*h*f*t_ref); cos(2*pi*h*f*t_ref)];
            end
            refs{fi}{bi} = R';
        end
    end

    fb_b = cell(1,NB); fb_a = cell(1,NB);
    fb_w = (1:NB).^(-FB_A) + FB_B;
    for m = 1:NB
        [fb_b{m}, fb_a{m}] = butter(BUTTER_ORDER, [m*8 min(FB_HIGH,nyq-1)]/nyq, 'bandpass');
    end
    [b_bp, a_bp] = butter(4, [5 45]/nyq, 'bandpass');

    spec_len = floor(stim_samp/2) + 1;
    fr = (0:spec_len-1) * (srate/stim_samp);
    offsets = 0 : step_samp : (stim_samp - win_samp);
    freq_res = srate/stim_samp;

    for tr = 1:nT
        cls = true_class(tr);
        s0 = onset(tr); s1 = s0 + stim_samp - 1;
        if s1 > size(data_occ,2), continue; end
        epoch = data_occ(:, s0:s1);

        % ---- SNR with peak search over ±SNR_PEAK_HALF ----
        snr_ch   = zeros(numel(FREQS), size(epoch,1));
        peak_ch  = zeros(numel(FREQS), size(epoch,1));
        ch_spec  = zeros(size(epoch,1), spec_len);
        for ch = 1:size(epoch,1)
            full_fft = abs(fft(epoch(ch,:))).^2;
            ch_spec(ch,:) = full_fft(1:spec_len);
            for fi = 1:numel(FREQS)
                % search window around nominal target
                lo = round((FREQS(fi)-SNR_PEAK_HALF)/freq_res) + 1;
                hi = round((FREQS(fi)+SNR_PEAK_HALF)/freq_res) + 1;
                lo = max(lo,1); hi = min(hi, numel(full_fft));
                [~, rel] = max(full_fft(lo:hi));
                tb = lo + rel - 1;                 % measured peak bin
                peak_ch(fi,ch) = (tb-1)*freq_res;  % measured peak freq (Hz)
                fl = [tb-SNR_FLANK_BINS:tb-1, tb+1:tb+SNR_FLANK_BINS];
                fl = fl(fl>=1 & fl<=numel(full_fft));
                snr_ch(fi,ch) = full_fft(tb)/mean(full_fft(fl));
            end
        end

        % ---- Sliding-window CCA + FBCCA, max over band ----
        best_cca = zeros(1,numel(FREQS));
        best_fb  = zeros(1,numel(FREQS));
        for oi = 1:numel(offsets)
            ws = offsets(oi)+1; we = ws+win_samp-1;
            w = epoch(:, ws:we);

            df = filtfilt(b_bp, a_bp, w')';
            for fi = 1:numel(FREQS)
                for bi = 1:n_band
                    [~,~,r] = canoncorr(df', refs{fi}{bi});
                    best_cca(fi) = max(best_cca(fi), r(1));
                end
            end

            % FBCCA: for each sub-band, take max rho over the freq-band, then
            % weight-sum the squared maxima across sub-bands.
            fb = zeros(1,numel(FREQS));
            for m = 1:NB
                sub = filtfilt(fb_b{m}, fb_a{m}, w')';
                for fi = 1:numel(FREQS)
                    rmax = 0;
                    for bi = 1:n_band
                        [~,~,r] = canoncorr(sub', refs{fi}{bi});
                        rmax = max(rmax, r(1));
                    end
                    fb(fi) = fb(fi) + fb_w(m)*rmax^2;
                end
            end
            best_fb = max(best_fb, fb);
        end

        P_true(end+1)    = cls;
        P_cca(end+1,:)   = best_cca;
        P_fbcca(end+1,:) = best_fb;
        P_snr(end+1,:)   = mean(snr_ch,2)';
        P_peakhz(end+1,:)= mean(peak_ch,2)';
        if isempty(P_fr)
            P_fr = fr; P_spec(end+1,:) = mean(ch_spec,1);
        elseif numel(fr)==numel(P_fr)
            P_spec(end+1,:) = mean(ch_spec,1);
        end
    end
    fprintf('%d trials pooled.\n', nT);
end

nP = numel(P_true);
fprintf('\nTOTAL pooled trials: %d\n', nP);

% ════════════════════════════════════════════════════════════════════════════
%% METRICS (pooled)
% ════════════════════════════════════════════════════════════════════════════

[~, cca_pred]   = max(P_cca,   [], 2);
[~, fbcca_pred] = max(P_fbcca, [], 2);
cca_corr   = (cca_pred'   - 1) == P_true;
fbcca_corr = (fbcca_pred' - 1) == P_true;
cca_acc    = 100 * mean(cca_corr);
fbcca_acc  = 100 * mean(fbcca_corr);

snr_db        = 10*log10(mean(P_snr, 1, 'omitnan'));
peak_measured = mean(P_peakhz, 1, 'omitnan');

fprintf('\n=== COMBINED RESULTS (band ±%.2f Hz) ===\n', BAND_HALF);
fprintf('Nominal freqs: %s Hz\n', strjoin(arrayfun(@(f) num2str(f), FREQS, 'UniformOutput', false), ' '));
fprintf('CCA   accuracy: %.1f%%  (%d/%d)\n', cca_acc,   sum(cca_corr),   nP);
fprintf('FBCCA accuracy: %.1f%%  (%d/%d)\n', fbcca_acc, sum(fbcca_corr), nP);
fprintf('\nPer-class:\n');
fprintf('  Nominal  MeasPeak   SNR     CCA    FBCCA   n\n');
for fi = 1:numel(FREQS)
    m = P_true == (fi-1);
    fprintf('  %5.1f Hz  %6.2f Hz  %5.1fdB  %4.0f%%  %4.0f%%   %d\n', ...
        FREQS(fi), peak_measured(fi), snr_db(fi), ...
        100*mean(cca_corr(m)), 100*mean(fbcca_corr(m)), sum(m));
end

% ════════════════════════════════════════════════════════════════════════════
%% SINGLE COMBINED FIGURE
% ════════════════════════════════════════════════════════════════════════════

figS = figure('Visible','off','Name','SSVEP Combined (band ±0.5 Hz)', ...
    'NumberTitle','off','Position',[120 80 1500 850]);

flabels = arrayfun(@(f) sprintf('%dHz',f), FREQS, 'UniformOutput', false);

% -- SNR bar --
subplot(2,3,1);
bar(1:numel(FREQS), snr_db, 'FaceColor',[0.3 0.6 0.9]);
set(gca,'XTick',1:numel(FREQS),'XTickLabel',flabels);
ylabel('SNR (dB)'); title(sprintf('SNR (peak-search, n=%d)',nP)); grid on;

% -- Per-class accuracy --
subplot(2,3,2);
pc_cca = zeros(1,numel(FREQS)); pc_fb = zeros(1,numel(FREQS));
for fi = 1:numel(FREQS)
    m = P_true == (fi-1);
    pc_cca(fi) = 100*mean(cca_corr(m));
    pc_fb(fi)  = 100*mean(fbcca_corr(m));
end
bar(1:numel(FREQS), [pc_cca; pc_fb]');
set(gca,'XTick',1:numel(FREQS),'XTickLabel',flabels);
ylabel('Accuracy (%)'); ylim([0 105]); legend('CCA','FBCCA','Location','best');
title('Per-class accuracy'); grid on; yline(25,'k--','chance');

% -- Overall accuracy --
subplot(2,3,3);
bar([cca_acc fbcca_acc], 'FaceColor',[0.5 0.5 0.8]);
set(gca,'XTick',1:2,'XTickLabel',{'CCA','FBCCA'});
ylabel('Accuracy (%)'); ylim([0 105]); title('Overall accuracy'); grid on;
text(1, cca_acc+3, sprintf('%.1f%%',cca_acc), 'HorizontalAlignment','center','FontWeight','bold');
text(2, fbcca_acc+3, sprintf('%.1f%%',fbcca_acc), 'HorizontalAlignment','center','FontWeight','bold');

% -- CCA confusion --
subplot(2,3,4);
Cc = confusionmat(P_true, cca_pred'-1);
imagesc(Cc); colormap(gca, parula); colorbar;
xlabel('Predicted'); ylabel('True'); title(sprintf('CCA (%.0f%%)',cca_acc));
set(gca,'XTick',1:numel(FREQS),'YTick',1:numel(FREQS),'XTickLabel',flabels,'YTickLabel',flabels);
for r=1:numel(FREQS); for c=1:numel(FREQS)
    text(c,r,num2str(Cc(r,c)),'HorizontalAlignment','center','FontWeight','bold','Color','k');
end; end

% -- FBCCA confusion --
subplot(2,3,5);
Cf = confusionmat(P_true, fbcca_pred'-1);
imagesc(Cf); colormap(gca, hot); colorbar;
xlabel('Predicted'); ylabel('True'); title(sprintf('FBCCA (%.0f%%)',fbcca_acc));
set(gca,'XTick',1:numel(FREQS),'YTick',1:numel(FREQS),'XTickLabel',flabels,'YTickLabel',flabels);
for r=1:numel(FREQS); for c=1:numel(FREQS)
    text(c,r,num2str(Cf(r,c)),'HorizontalAlignment','center','FontWeight','bold','Color','k');
end; end

% -- Pooled spectra per class, with band shading --
subplot(2,3,6);
if ~isempty(P_spec)
    % Stimulus colours, so each trace matches the LED group it came from:
    % 16 Hz red, 19 Hz blue, 23 Hz green, 29 Hz yellow. Line, shading and
    % label all take the same colour.
    colors = [0.85 0.15 0.15;   % 16 Hz — red
              0.15 0.35 0.80;   % 19 Hz — blue
              0.15 0.60 0.25;   % 23 Hz — green
              0.90 0.70 0.10];  % 29 Hz — yellow
    if numel(FREQS) > size(colors,1)
        colors = [colors; lines(numel(FREQS)-size(colors,1))];
    end
    colors = colors(1:numel(FREQS), :);
    hold on;
    yl = [Inf -Inf];
    hLines = gobjects(1, numel(FREQS));
    for fi = 1:numel(FREQS)
        m = P_true == (fi-1);
        spec_avg = mean(P_spec(m,:), 1, 'omitnan');
        spec_db  = 10*log10(spec_avg / median(spec_avg));
        sel = P_fr >= 14 & P_fr <= 31;
        hLines(fi) = plot(P_fr(sel), spec_db(sel), 'Color', colors(fi,:), ...
            'LineWidth', 1.1);
        yl = [min(yl(1),min(spec_db(sel))), max(yl(2),max(spec_db(sel)))];
    end
    % Shade each ±BAND_HALF band. Patches and xlines are excluded from the
    % legend: with AutoUpdate on they would otherwise be picked up as the
    % first entries, so the swatches would show patch colours in patch order
    % rather than the four line colours.
    for fi = 1:numel(FREQS)
        xb = [FREQS(fi)-BAND_HALF, FREQS(fi)+BAND_HALF];
        patch([xb(1) xb(2) xb(2) xb(1)], [yl(1) yl(1) yl(2) yl(2)], ...
            colors(fi,:), 'FaceAlpha',0.10, 'EdgeColor','none', ...
            'HandleVisibility','off');
        xline(FREQS(fi), ':', flabels{fi}, 'Color', colors(fi,:)*0.85, ...
            'LabelOrientation','horizontal','FontSize',7, ...
            'HandleVisibility','off');
    end
    uistack(hLines, 'top');   % keep the traces above the shading
    xlabel('Frequency (Hz)'); ylabel('Power (dB re median)');
    title('Pooled spectra (shaded = ±0.5 Hz band)'); grid on; xlim([14 31]);
    legend(hLines, flabels, 'Location','northeast','FontSize',7, ...
           'AutoUpdate','off');
end

% ITR (Wolpaw): bits per selection x selections per minute, where a selection
% takes WINDOW_SEC. This is the classifier's rate; add CUE_MS and the
% inter-trial blank for the rate a user of the whole system would experience.
cca_itr   = wolpaw_itr(cca_acc/100,   numel(FREQS), WINDOW_SEC);
fbcca_itr = wolpaw_itr(fbcca_acc/100, numel(FREQS), WINDOW_SEC);

sgtitle(sprintf(['SSVEP Combined — band ±%.2f Hz — %d trials  |  ' ...
    'CCA %.0f%%, ITR %.1f bits/min  |  FBCCA %.0f%%, ITR %.1f bits/min'], ...
    BAND_HALF, nP, cca_acc, cca_itr, fbcca_acc, fbcca_itr), ...
    'FontSize',13,'FontWeight','bold');

outPng = fullfile(DATA_DIR, sprintf('SSVEP_combined_%s.png', PID));
exportgraphics(figS, outPng, 'Resolution', 200);
fprintf('\nFigure saved: %s\n', outPng);
close(figS);

fprintf('  CCA   %.1f%%  ITR %.1f bits/min\n', cca_acc,   cca_itr);
fprintf('  FBCCA %.1f%%  ITR %.1f bits/min\n', fbcca_acc, fbcca_itr);
fprintf('\nDone.\n');
end

% ════════════════════════════════════════════════════════════════════════════
%% BRAINVISION HELPERS (verbatim from ssvep_offline_analysis.m)
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


function itr = wolpaw_itr(P, N, T)
%WOLPAW_ITR  Bits per minute (Wolpaw et al. 2000).
%   B   = log2(N) + P*log2(P) + (1-P)*log2((1-P)/(N-1))
%   ITR = B * 60 / T
if N < 2 || T <= 0, itr = 0; return; end
if P >= 1
    B = log2(N);
elseif P <= 1/N
    B = 0;
else
    B = log2(N) + P*log2(P) + (1-P)*log2((1-P)/(N-1));
end
itr = max(B,0) * 60 / T;
end
