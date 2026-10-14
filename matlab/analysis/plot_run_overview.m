%% ========================================================================
%  plot_run_overview.m  --  per-run overview timeline (the fig1 layout)
%
%  Three stacked panels on a shared time axis:
%     1. occipital EEG, band-passed and offset-stacked
%     2. classifier raster: winning class per window, below/above threshold
%     3. max FBCCA score with the decision threshold
%  plus shaded selection windows, published-command stars and pin-9 TTL marks.
%
%  Run it inside a data folder: it makes one PNG per .mat file, or set
%  CFG.only_file to do a single run.
%  ========================================================================

clear; close all; clc;

%% ---------------------------- CONFIG -----------------------------------
CFG.data_dir  = pwd;
CFG.file_glob = '*.mat';
CFG.only_file = '';                       % e.g. 'exp1_20260724T112301.mat'
CFG.exclude   = {'analysis_summary.mat'};

CFG.occ_idx   = [2 14 13 19 16 17 18];    % occipital channels -- HARDCODED
CFG.trig_col  = 33;                       % pin-9 TTL column
CFG.freqs     = [16 19 23 29];
CFG.fs        = 500;

CFG.bp        = [5 45];                   % display band-pass (Hz)
CFG.car       = true;                     % common average ref (after filtering)
CFG.eeg_scale = [];                       % uV between channel rows; [] = auto
CFG.decimate  = 1;                        % plot every Nth EEG sample

CFG.corr_thresh = 0.30;
CFG.band_mode   = 'selection';            % 'selection' = cue->command
                                          % 'trial'     = cue->trial_end
CFG.out_suffix  = '_overview.png';
CFG.resolution  = 200;

% Okabe-Ito, colourblind safe, one per stimulus frequency
CFG.cmap = [0.00 0.45 0.70;               % 16 Hz
            0.90 0.62 0.00;               % 19 Hz
            0.00 0.62 0.45;               % 23 Hz
            0.80 0.40 0.70];              % 29 Hz
CFG.grey = [0.62 0.62 0.62];

%% ---------------------------- DRIVER -----------------------------------
if ~isempty(CFG.only_file)
    files = dir(fullfile(CFG.data_dir, CFG.only_file));
else
    files = dir(fullfile(CFG.data_dir, CFG.file_glob));
    files = files(~ismember({files.name}, CFG.exclude));
end
files = files(~[files.isdir]);
assert(~isempty(files), 'No .mat files found in %s', CFG.data_dir);

for k = 1:numel(files)
    fpath = fullfile(files(k).folder, files(k).name);
    fprintf('[%d/%d] %s\n', k, numel(files), files(k).name);
    try
        make_overview(fpath, CFG);
    catch ME
        warning('%s failed: %s', files(k).name, ME.message);
    end
end
fprintf('Done.\n');

%% ========================= MAIN PLOTTER ================================
function make_overview(fpath, CFG)
S = load(fpath);
[~, base] = fileparts(fpath);
fs = CFG.fs; if isfield(S,'fs'), fs = double(S.fs); end

% ---------------- markers ----------------------------------------------
M     = S.markers;
mtype = local_cellstr(M.type);
minfo = local_cellstr(M.info);
msamp = double(M.sample(:));

decIdx = find(strcmp(mtype,'decision'));
nD  = numel(decIdx);
dT  = msamp(decIdx)/fs;
dSc = nan(nD, numel(CFG.freqs));
dPass = zeros(nD,1); dPub = zeros(nD,1);
for i = 1:nD
    d = local_parse_kv(minfo{decIdx(i)});
    dPass(i) = local_fget(d,'pass',0);
    dPub(i)  = local_fget(d,'pub',0);
    if isfield(d,'s')
        v = sscanf(strrep(d.s,',',' '),'%f')';
        dSc(i,1:min(numel(v),numel(CFG.freqs))) = v(1:min(numel(v),numel(CFG.freqs)));
    end
end
[dMax, dArg] = max(dSc,[],2);             % argmax defines the raster row,
                                          % so sub-threshold windows show too

cueT = msamp(strcmp(mtype,'cue_onset') | strcmp(mtype,'home_reached'))/fs;
endT = msamp(strcmp(mtype,'trial_end')  | strcmp(mtype,'experiment_end'))/fs;
pubT = dT(dPub==1);

% ---------------- EEG ---------------------------------------------------
hasEEG = isfield(S,'eeg') && ~isempty(S.eeg);
if hasEEG
    X   = double(S.eeg);
    nS  = size(X,1);
    tE  = (0:nS-1)'/fs;
    idx = CFG.occ_idx(CFG.occ_idx <= size(X,2));
    [b,a] = butter(4, CFG.bp/(fs/2), 'bandpass');
    Xf = filtfilt(b, a, detrend(X(:,1:min(32,size(X,2))),'constant'));
    if CFG.car, Xf = Xf - mean(Xf,2); end   % CAR AFTER band-pass
    E = Xf(:, idx);
    % TTL rises
    ttlT = [];
    if size(X,2) >= CFG.trig_col
        ttlT = (find(diff(X(:,CFG.trig_col)) > 64) + 1)/fs;
    end
    dur = nS/fs;
else
    E = []; idx = []; ttlT = []; tE = [];
    dur = max([dT; endT; 0]);
end

% ---------------- selection bands ---------------------------------------
nSel = numel(cueT);
band = nan(nSel,2);
for i = 1:nSel
    e = endT(endT > cueT(i));
    if isempty(e), e = dur; else, e = e(1); end
    if strcmp(CFG.band_mode,'selection')
        p = pubT(pubT > cueT(i) & pubT <= e);
        if isempty(p), band(i,:) = [cueT(i) e]; else, band(i,:) = [cueT(i) p(1)]; end
    else
        band(i,:) = [cueT(i) e];
    end
end

%% ---------------- FIGURE ----------------------------------------------
F = figure('Color','w','Position',[80 80 1320 930]);
T = tiledlayout(F, 9, 1, 'TileSpacing','none', 'Padding','compact');
xl = [0 dur];

% ============ panel 1: EEG ============
axE = nexttile([5 1]); hold(axE,'on'); box(axE,'off');
if hasEEG
    if isempty(CFG.eeg_scale)
        step = 6 * median(std(E,0,1));    % auto row spacing
    else
        step = CFG.eeg_scale;
    end
    dec = max(1, round(CFG.decimate));
    for c = 1:numel(idx)
        off = (numel(idx) - c) * step;
        plot(axE, tE(1:dec:end), E(1:dec:end,c) + off, 'Color',[0.15 0.15 0.15], 'LineWidth',0.35);
        text(axE, -0.012*dur, off, sprintf('ch %d', idx(c)), ...
             'HorizontalAlignment','right','FontSize',9,'Clipping','off');
    end
    ylim(axE, [-step, numel(idx)*step]);
    % scale bar
    sb = round(step/3);
    plot(axE, [dur*0.995 dur*0.995], [-step*0.8 -step*0.8+sb], 'k-','LineWidth',2,'Clipping','off');
    text(axE, dur*0.988, -step*0.8+sb/2, sprintf('%g \\muV', sb), ...
         'HorizontalAlignment','right','FontSize',8,'Clipping','off');
else
    text(axE, 0.5, 0.5, 'no EEG in file', 'Units','normalized','HorizontalAlignment','center');
end
ylabel(axE, 'EEG (\muV)'); set(axE,'YTick',[],'XTickLabel',[]); xlim(axE, xl);
local_bands(axE, band, CFG, true);
for i = 1:numel(ttlT)
    plot(axE, ttlT(i), min(ylim(axE)), '^', 'MarkerSize',6, ...
         'MarkerFaceColor',[0.85 0.2 0.2], 'MarkerEdgeColor','none','Clipping','off');
end

% ============ panel 2: classifier raster ============
axR = nexttile([2 1]); hold(axR,'on'); box(axR,'off');
lo = dPass == 0; hi = dPass == 1;
plot(axR, dT(lo), dArg(lo), '.', 'Color', CFG.grey, 'MarkerSize',4);
for c = 1:numel(CFG.freqs)
    m = hi & dArg == c;
    plot(axR, dT(m), dArg(m), 'o', 'MarkerSize',4.5, ...
         'MarkerFaceColor', CFG.cmap(c,:), 'MarkerEdgeColor','none');
end
for i = 1:numel(pubT)
    j = find(dT == pubT(i), 1);
    plot(axR, pubT(i), dArg(j), 'p', 'MarkerSize',15, ...
         'MarkerFaceColor',[1 0.85 0.1], 'MarkerEdgeColor','k','LineWidth',0.6);
end
set(axR,'YTick',1:numel(CFG.freqs),'YTickLabel',compose('%d Hz',CFG.freqs), ...
        'XTickLabel',[],'YDir','reverse');
ylim(axR,[0.4 numel(CFG.freqs)+0.6]); xlim(axR, xl);
ylabel(axR,'class (Hz)'); local_bands(axR, band, CFG, false);

% ============ panel 3: score ============
axS = nexttile([2 1]); hold(axS,'on'); box(axS,'off');
plot(axS, dT, dMax, '-', 'Color',[0.35 0.35 0.35], 'LineWidth',0.7);
plot(axS, dT(lo), dMax(lo), '.', 'Color', CFG.grey, 'MarkerSize',4);
for c = 1:numel(CFG.freqs)
    m = hi & dArg == c;
    plot(axS, dT(m), dMax(m), 'o', 'MarkerSize',4, ...
         'MarkerFaceColor', CFG.cmap(c,:), 'MarkerEdgeColor','none');
end
yline(axS, CFG.corr_thresh, '--', sprintf('thresh %.2f', CFG.corr_thresh), ...
      'Color',[0.75 0.1 0.1], 'FontSize',8, 'LabelHorizontalAlignment','left');
xlim(axS, xl); ylim(axS, [0 max(0.7, max(dMax)*1.1)]);
ylabel(axS,'FBCCA score'); xlabel(axS,'time (s)');
local_bands(axS, band, CFG, false);

% ============ title + legend ============
title(T, sprintf('%s  —  %.0f s, %g Hz, %d selections, %d classifier windows', ...
      base, dur, fs, numel(pubT), nD), 'Interpreter','none','FontWeight','bold');

axL = axes(F,'Position',[0.08 0.955 0.86 0.03],'Visible','off'); hold(axL,'on');
h(1) = plot(axL, nan, nan, 'p','MarkerSize',12,'MarkerFaceColor',[1 0.85 0.1],'MarkerEdgeColor','k');
h(2) = plot(axL, nan, nan, '.','Color',CFG.grey,'MarkerSize',14);
h(3) = plot(axL, nan, nan, 'o','MarkerSize',5,'MarkerFaceColor',CFG.cmap(1,:),'MarkerEdgeColor','none');
h(4) = plot(axL, nan, nan, '^','MarkerSize',6,'MarkerFaceColor',[0.85 0.2 0.2],'MarkerEdgeColor','none');
h(5) = patch(axL, nan, nan, [0.90 0.90 0.94],'EdgeColor','none');
legend(axL, h, {'command published','window below threshold','window passed threshold', ...
       'pin-9 TTL (robot home)','selection window'}, ...
       'Orientation','horizontal','Location','north','Box','off','FontSize',9);

exportgraphics(F, fullfile(fileparts(fpath), [base CFG.out_suffix]), ...
               'Resolution', CFG.resolution);
close(F);
end

%% ========================= HELPERS =====================================
function local_bands(ax, band, CFG, withLabels)
% Shade the selection windows and push them behind the data.
    yl = ylim(ax);
    for i = 1:size(band,1)
        if any(isnan(band(i,:))), continue; end
        p = patch(ax, band(i,[1 2 2 1]), yl([1 1 2 2]), [0.90 0.90 0.94], ...
                  'EdgeColor','none','FaceAlpha',0.75);
        uistack(p,'bottom');
        if withLabels
            text(ax, mean(band(i,:)), yl(2), sprintf('selection %d', i), ...
                 'HorizontalAlignment','center','VerticalAlignment','bottom', ...
                 'FontSize',8,'Color',[0.35 0.35 0.4],'Clipping','off');
        end
    end
    ylim(ax, yl);
end

function c = local_cellstr(x)
    if iscell(x), c = x; elseif isstring(x), c = cellstr(x); else, c = cellstr(string(x)); end
end

function d = local_parse_kv(str)
    d = struct();
    parts = strsplit(strtrim(str), ';');
    for k = 1:numel(parts)
        if isempty(parts{k}), continue; end
        p = strsplit(parts{k}, '=');
        if numel(p)==2 && ~isempty(p{1})
            d.(matlab.lang.makeValidName(strtrim(p{1}))) = strtrim(p{2});
        end
    end
end

function v = local_fget(d,f,dflt)
    if isfield(d,f), v = str2double(d.(f)); if isnan(v), v = dflt; end
    else, v = dflt; end
end
