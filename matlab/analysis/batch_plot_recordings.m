function batch_plot_recordings(folder, varargin)
%BATCH_PLOT_RECORDINGS  One overview figure per recording in a folder.
%
% Generates the four-panel overview for every record_ssvep_continuous .mat in
% FOLDER and saves each as a PNG. Prints a summary table across all files.
%
%   panel 1  phase lane   trial start -> first command (BCI) | command -> home (robot)
%   panel 2  raw EEG      connected channels, bandpassed, trials shaded
%   panel 3  raster       winning class per window, passed vs failed
%   panel 4  scores       one trace per stimulus class, in the class colours
%
% USAGE
%   batch_plot_recordings('~/ssvep-workspace-augmentation/matlab')
%   batch_plot_recordings(folder, 'OutDir', '~/figs')
%   batch_plot_recordings(folder, 'Thresholds', [0.30 0.05])
%   batch_plot_recordings(folder, 'Freqs', [19 23])     % screwdriver blocks
%   batch_plot_recordings(folder, 'ShowMargin', false)  % drop the margin trace
%   batch_plot_recordings(folder, 'Close', false)       % keep figures open
%
% Handles both marker vocabularies seen across node builds: blocks that emit
% cue_onset, and blocks that emit home_reached with no cue_onset at all. In the
% latter case each trial is taken to start at the previous trial_end.

p = inputParser;
addParameter(p, 'OutDir',     '');                       % default: <folder>/figures
addParameter(p, 'Channels',   [2 14 13 19 16 17 18]);    % connected electrodes
addParameter(p, 'Freqs',      [16 19 23 29]);
addParameter(p, 'Band',       [1 45]);
addParameter(p, 'Thresholds', [0.30 0.05]);              % [corr margin]
addParameter(p, 'TrigChannel', 33);                      % pin-9 TTL, 0 = none
addParameter(p, 'TimeZone',   'Europe/London');          % fallback only
addParameter(p, 'ShowMargin', true);
addParameter(p, 'Close',      true);
addParameter(p, 'DPI',        150);
parse(p, varargin{:}); o = p.Results;

% Class colours, indexed to o.Freqs in order: 16 red, 19 blue, 23 green, 29 yellow.
o.Colors = [0.84 0.19 0.15;    % 16 Hz  red
            0.16 0.35 0.68;    % 19 Hz  blue
            0.18 0.60 0.31;    % 23 Hz  green
            0.88 0.68 0.05];   % 29 Hz  yellow/gold

folder = char(folder);
if isempty(o.OutDir), o.OutDir = fullfile(folder, 'figures'); end
if ~exist(o.OutDir, 'dir'), mkdir(o.OutDir); end

files = dir(fullfile(folder, '*.mat'));
files = files(~[files.isdir]);
if isempty(files)
    error('No .mat files found in %s', folder);
end
[~, ord] = sort([files.datenum]); files = files(ord);

fprintf('\n%d recording(s) in %s\n\n', numel(files), folder);
summary = table();

for k = 1:numel(files)
    fn = fullfile(files(k).folder, files(k).name);
    try
        row = plot_one(fn, o);
        summary = [summary; row]; %#ok<AGROW>
    catch ME
        fprintf('  SKIP %-34s  %s\n', files(k).name, ME.message);
    end
end

if ~isempty(summary)
    fprintf('\n===== summary =====\n');
    disp(summary);
    csv = fullfile(o.OutDir, 'summary.csv');
    writetable(summary, csv);
    fprintf('summary -> %s\nfigures -> %s\n\n', csv, o.OutDir);
end
end


% =============================================================================
function row = plot_one(matfile, o)

[~, base] = fileparts(matfile);
S = load(matfile);

% ── EEG ──────────────────────────────────────────────────────────────────────
eeg = double(S.eeg);
if size(eeg,1) > size(eeg,2), eeg = eeg.'; end     % force [channels x samples]
fs  = double(S.fs);
N   = size(eeg,2);
t   = (0:N-1)/fs;
dur = N/fs;

% ── wall-clock, taken from the FILENAME ──────────────────────────────────────
% The recorder names the file at flush time in local time, so the embedded
% yyyymmddTHHMMSS is the moment the block ENDED. Start is that minus the
% recording duration. t0u is unix and would need a timezone conversion to be
% readable, so the filename is both simpler and the thing you actually see in
% the folder — it is used here in preference.
tok = regexp(base, '(\d{8})T(\d{6})', 'tokens', 'once');
if numel(tok) == 2
    d1 = datetime([tok{1} tok{2}], 'InputFormat','yyyyMMddHHmmss');
    d0 = d1 - seconds(dur);
    tsrc = 'filename';
elseif isfield(S,'t0u') && ~isempty(S.t0u)
    d0 = datetime(double(S.t0u), 'ConvertFrom','posixtime', 'TimeZone','UTC');
    d0.TimeZone = o.TimeZone; d0.TimeZone = '';
    d1 = d0 + seconds(dur);
    tsrc = 't0u';
else
    d0 = NaT; d1 = NaT; tsrc = 'unknown';
end
when = sprintf('%s   %s - %s', char(d0,'eee dd MMM yyyy'), ...
               char(d0,'HH:mm:ss'), char(d1,'HH:mm:ss'));

% ── markers ──────────────────────────────────────────────────────────────────
mk    = S.markers;
mtype = string(mk.type);
minfo = string(mk.info);
msamp = double(mk.sample);

% ── decision stream ──────────────────────────────────────────────────────────
% Parsed from char, not string, so regexp returns an unambiguous cell array.
FRQ = o.Freqs;
nF  = numel(FRQ);
di  = find(mtype == "decision");
nd  = numel(di);
dt  = msamp(di)/fs;
dmx = nan(nd,1); dmr = nan(nd,1);
dps = false(nd,1); dpb = false(nd,1);
dsc = nan(nd, nF);
for i = 1:nd
    tk = regexp(char(minfo(di(i))), ...
        'win=(-?[\d.]+);max=([\d.]+);mar=([\d.]+);pass=(\d);pub=(\d);s=([\d.,]+)', ...
        'tokens', 'once');
    if numel(tk) < 6, continue; end
    dmx(i) = str2double(tk{2});
    dmr(i) = str2double(tk{3});
    dps(i) = str2double(tk{4}) == 1;
    dpb(i) = str2double(tk{5}) == 1;
    v = str2double(strsplit(tk{6}, ','));
    n = min(numel(v), nF);
    dsc(i, 1:n) = v(1:n);
end
if nd > 0 && all(isnan(dmx))
    warning('%s: no decision rows parsed - check the info field format.', base);
end
% win= is the winning FREQUENCY in Hz and is 0 on any failed window, so the
% class of a failed window has to come from the argmax of the s= vector.
[~, dam] = max(dsc, [], 2);
dwin = FRQ(dam).';

% ── trial structure ──────────────────────────────────────────────────────────
fin      = msamp(mtype == "trial_end")/fs;
info_end = minfo(mtype == "trial_end");
cue      = msamp(mtype == "cue_onset")/fs;
if isempty(cue)
    % Node build that emits home_reached instead. Trial n starts where trial
    % n-1 ended; trial 1 starts at the beginning of the recording.
    cue = [0; fin(1:end-1)];
    cue_src = 'inferred from trial_end';
else
    cue_src = 'cue_onset';
end
nT = min(numel(cue), numel(fin));

pub_t = dt(dpb); pub_f = dwin(dpb); pub_c = dam(dpb);

% ── TTL ──────────────────────────────────────────────────────────────────────
trig = [];
if o.TrigChannel > 0 && size(eeg,1) >= o.TrigChannel
    tr = eeg(o.TrigChannel,:);
    if max(tr) > min(tr)
        thr  = (max(tr)+min(tr))/2;
        trig = (find(diff(tr > thr) == 1) + 1)/fs;
    end
end

% ── filter ───────────────────────────────────────────────────────────────────
ch = o.Channels(o.Channels <= size(eeg,1));
[bb, aa] = butter(4, o.Band/(fs/2), 'bandpass');
E = filtfilt(bb, aa, eeg(ch,:).').';

% ── console line ─────────────────────────────────────────────────────────────
failFrac = zeros(1, nF);
if any(~dps)
    for c = 1:nF, failFrac(c) = mean(dam(~dps) == c); end
end
fprintf('%-34s %s | %6.1f s | %4d win | %3d pass | %2d pub | %d trials (%s)\n', ...
        [base '.mat'], char(d0,'dd MMM HH:mm'), dur, nd, sum(dps), ...
        sum(dpb), nT, cue_src);
fprintf('%38s noise floor by class: ', '');
fprintf('%dHz %2.0f%%  ', [FRQ(:).'; 100*failFrac]);
fprintf('\n');

% =============================================================================
% FIGURE
% =============================================================================
COL = o.Colors;
if nF > size(COL,1), COL = repmat(COL, ceil(nF/size(COL,1)), 1); end
GRN = [0.18 0.55 0.34]; ORG = [0.85 0.55 0.17]; PUR = [0.49 0.24 0.60];
GRY = [0.73 0.76 0.80];

fig = figure('Color','w','Position',[40 40 1500 980],'Visible','off');
tl  = tiledlayout(fig, 4, 1, 'TileSpacing','tight', 'Padding','compact');
tl.Title.String = sprintf(['%s.mat        %s        %.0f s, %d trials, ' ...
                           '%d windows, %d passed, %d published'], ...
                          base, when, dur, nT, nd, sum(dps), sum(dpb));
tl.Title.FontSize = 12; tl.Title.FontWeight = 'normal';
tl.Title.Interpreter = 'none';

% ─────────────────────────────────────────────────────────────────────────────
% Every panel sets its y-limits EXPLICITLY and shades before plotting. The
% shading patches span the full limit range, so a panel left on autoscale would
% stretch to the patch extent and collapse the real data to a flat line.
% ─────────────────────────────────────────────────────────────────────────────

% ── panel 1: phase lane ──────────────────────────────────────────────────────
yl0 = [0 1.95];
ax0 = nexttile(tl); hold(ax0,'on');
ylim(ax0, yl0);
for k = 1:nT
    c0 = cue(k); c1 = fin(k);
    if ~(c1 > c0), continue; end
    ts = pub_t(pub_t >= c0 & pub_t < c1);
    if isempty(ts)
        rectangle(ax0,'Position',[c0 0.75 c1-c0 0.5],'FaceColor',GRY,'EdgeColor','none');
        text(ax0,(c0+c1)/2, 1.0, 'no command', 'HorizontalAlignment','center', ...
             'FontSize',8,'Color','w','FontWeight','bold');
    else
        p0 = ts(1);
        wD = max(p0-c0, 1e-6);          % rectangle() rejects zero/negative width
        wR = max(c1-p0, 1e-6);
        rectangle(ax0,'Position',[c0 0.75 wD 0.5],'FaceColor',GRN,'EdgeColor','none');
        rectangle(ax0,'Position',[p0 0.75 wR 0.5],'FaceColor',ORG,'EdgeColor','none');
        if wD > 0.02*(c1-c0)
            text(ax0,(c0+p0)/2, 1.0, sprintf('%.1f',p0-c0), 'HorizontalAlignment','center', ...
                 'FontSize',7.5,'Color','w','FontWeight','bold');
        end
        text(ax0,(p0+c1)/2, 1.0, sprintf('robot  %.1f s',c1-p0), ...
             'HorizontalAlignment','center','FontSize',8.5,'Color','w','FontWeight','bold');
        if numel(ts) > 1     % commands that arrived while the arm was committed
            plot(ax0, ts(2:end), ones(1,numel(ts)-1), 'x', 'Color',[0.29 0.14 0.35], ...
                 'MarkerSize',8,'LineWidth',2);
        end
    end
    text(ax0, c0, 1.62, sprintf('trial %d',k), 'FontSize',8, 'Color',[.4 .4 .4], ...
         'FontAngle','italic');
    if k <= numel(info_end)
        text(ax0, c1, 0.30, char(info_end(k)), 'FontSize',7.5, ...
             'Color',[.12 .37 .66], 'HorizontalAlignment','center');
    end
end
plot(ax0, cue(1:nT), 0.45*ones(1,nT), 'v', 'MarkerFaceColor',GRN, ...
     'MarkerEdgeColor','none','MarkerSize',6);
plot(ax0, fin(1:nT), 0.45*ones(1,nT), '^', 'MarkerFaceColor',[.12 .37 .66], ...
     'MarkerEdgeColor','none','MarkerSize',6);
for x = trig(:).'
    plot(ax0, [x x], [0.20 0.62], 'Color',[0.75 0.22 0.17], 'LineWidth',2);
end
ylim(ax0, yl0);
set(ax0, 'YTick', [], 'XTick', [], 'Box','off', 'YColor','k', 'XColor','none');
ylabel(ax0, 'phases', 'FontSize',9);

% ── panel 2: EEG ─────────────────────────────────────────────────────────────
ax1 = nexttile(tl); hold(ax1,'on');
sc  = median(abs(E - median(E,2)),2)*1.4826; sc(sc==0) = 1;
off = 6*median(sc)*((numel(ch)-1):-1:0).';
Y   = E./sc*median(sc) + off;
yl1 = [min(Y(:))-1.5*median(sc), max(off)+5.5*median(sc)];
ylim(ax1, yl1);
shadeTrials(ax1, cue, fin, nT, yl1);
plot(ax1, t, Y.', 'k', 'LineWidth', 0.3);
for i = 1:numel(pub_t)
    plot(ax1, [pub_t(i) pub_t(i)], yl1, '--', 'Color', PUR, 'LineWidth', 1.0);
    text(ax1, pub_t(i), max(off)+1.4*median(sc), sprintf('%d Hz',pub_f(i)), ...
         'Rotation',90, 'FontSize',7.5, 'Color',COL(pub_c(i),:), ...
         'FontWeight','bold', 'HorizontalAlignment','right');
end
ylim(ax1, yl1);
set(ax1,'YTick',flipud(off),'YTickLabel',compose('ch %d',fliplr(ch)), ...
    'Box','off','XColor','none');
ylabel(ax1, sprintf('EEG %g-%g Hz', o.Band(1), o.Band(2)), 'FontSize',10);

% ── panel 3: raster ──────────────────────────────────────────────────────────
yl2 = [min(FRQ)-3.5, max(FRQ)+3.5];
ax2 = nexttile(tl); hold(ax2,'on');
ylim(ax2, yl2);
shadeTrials(ax2, cue, fin, nT, yl2);
scatter(ax2, dt(~dps), dwin(~dps), 7, GRY, 'filled');
for c = 1:nF
    m = dps & dam == c;
    scatter(ax2, dt(m), dwin(m), 30, COL(c,:), 'filled', ...
            'MarkerEdgeColor','w','LineWidth',0.3);
end
for i = 1:numel(pub_t)
    plot(ax2, [pub_t(i) pub_t(i)], yl2, '--', 'Color', PUR, 'LineWidth', 1.0);
end
scatter(ax2, pub_t, pub_f, 170, PUR, 'p', 'filled', 'MarkerEdgeColor','w','LineWidth',0.4);
ylim(ax2, yl2);
set(ax2,'YTick',FRQ,'YTickLabel',compose('%d Hz',FRQ),'Box','off','XColor','none');
ylabel(ax2,'winning class','FontSize',10);
grid(ax2,'on'); ax2.GridColor = [0.85 0.87 0.90]; ax2.GridAlpha = 1;

% ── panel 4: per-class scores ────────────────────────────────────────────────
% One trace per stimulus frequency, in the class colours, rather than just the
% max/margin pair. The max is simply the upper envelope of these traces, so
% nothing is lost and the competition between classes becomes visible.
top = max([dsc(:); dmr; o.Thresholds(:)], [], 'omitnan');
if isempty(top) || ~isfinite(top), top = 1; end
yl3 = [0, top*1.22];
ax3 = nexttile(tl); hold(ax3,'on');
ylim(ax3, yl3);
shadeTrials(ax3, cue, fin, nT, yl3);
for i = 1:numel(pub_t)
    plot(ax3, [pub_t(i) pub_t(i)], yl3, '--', 'Color', PUR, 'LineWidth', 1.0);
end
hS = gobjects(1,nF);
for c = 1:nF
    hS(c) = plot(ax3, dt, dsc(:,c), 'LineWidth',0.85, 'Color',COL(c,:));
end
lh = hS; ll = compose('%d Hz', FRQ(:).');
if o.ShowMargin
    hM = plot(ax3, dt, dmr, 'LineWidth',0.7, 'Color',[0.45 0.45 0.45]);
    lh = [hS hM]; ll = [ll, {'margin'}];
end
plot(ax3, [0 t(end)], o.Thresholds(1)*[1 1], ':', 'Color',[0.2 0.2 0.2],'LineWidth',1.3);
plot(ax3, [0 t(end)], o.Thresholds(2)*[1 1], ':', 'Color',[0.55 0.55 0.55],'LineWidth',1.1);
text(ax3, t(end), o.Thresholds(1), sprintf('corr %.2f ',o.Thresholds(1)), ...
     'FontSize',7.5,'Color',[0.2 0.2 0.2],'HorizontalAlignment','right','VerticalAlignment','bottom');
ylim(ax3, yl3);
legend(ax3, lh, ll, 'Location','northwest','Box','off','FontSize',8, ...
       'Orientation','horizontal','NumColumns',nF+1);
ylabel(ax3,'FBCCA score','FontSize',10); xlabel(ax3,'time (s)','FontSize',10);
box(ax3,'off');

linkaxes([ax0 ax1 ax2 ax3],'x');
xlim(ax3, [0 t(end)]);

% ── shared legend ────────────────────────────────────────────────────────────
lg = gobjects(1,6);
lg(1) = patch(ax0,'XData',NaN,'YData',NaN,'FaceColor',GRN,'EdgeColor','none', ...
              'DisplayName','trial start to command (BCI)');
lg(2) = patch(ax0,'XData',NaN,'YData',NaN,'FaceColor',ORG,'EdgeColor','none', ...
              'DisplayName','command to home (robot)');
lg(3) = plot(ax0,NaN,NaN,'x','Color',[0.29 0.14 0.35],'MarkerSize',8, ...
             'LineWidth',2,'DisplayName','command while arm busy');
lg(4) = plot(ax0,NaN,NaN,'-','Color',[0.75 0.22 0.17],'LineWidth',2, ...
             'DisplayName','pin-9 TTL');
lg(5) = plot(ax0,NaN,NaN,'p','Color',PUR,'MarkerFaceColor',PUR,'MarkerSize',10, ...
             'LineStyle','none','DisplayName','command published');
lg(6) = plot(ax0,NaN,NaN,'o','Color',GRY,'MarkerFaceColor',GRY,'MarkerSize',5, ...
             'LineStyle','none','DisplayName','window below threshold');
ylim(ax0, yl0);
legend(ax0, lg, 'Orientation','horizontal', 'Box','off', 'FontSize',8.5, ...
       'Location','northoutside', 'NumColumns',6);

% ── save ─────────────────────────────────────────────────────────────────────
png = fullfile(o.OutDir, [base '.png']);
exportgraphics(fig, png, 'Resolution', o.DPI);
if o.Close, close(fig); else, set(fig,'Visible','on'); end

% ── summary row ──────────────────────────────────────────────────────────────
decide = nan(nT,1); robot = nan(nT,1);
for k = 1:nT
    ts = pub_t(pub_t >= cue(k) & pub_t < fin(k));
    if ~isempty(ts), decide(k) = ts(1)-cue(k); robot(k) = fin(k)-ts(1); end
end
row = table( string([base '.mat']), d0, d1, dur, nT, nd, sum(dps), sum(dpb), ...
             mean(decide,'omitnan'), mean(robot,'omitnan'), ...
             string(cue_src), string(tsrc), ...
    'VariableNames', {'file','start','stop','dur_s','trials','windows','passed', ...
                      'published','mean_decide_s','mean_robot_s', ...
                      'trial_marker','time_source'});
end


% =============================================================================
function shadeTrials(ax, cue, fin, nT, yl)
% Shade each trial rather than drawing lines at its edges - with ~1000 markers
% per block, lines produce an unreadable picket fence. Patches are drawn to the
% panel's own explicit y-limits, never to an arbitrary large value, so they
% cannot drag an autoscaled axis out of range.
for k = 1:nT
    if ~(fin(k) > cue(k)), continue; end
    if mod(k,2), c = [0.965 0.975 0.985]; else, c = [0.925 0.945 0.965]; end
    patch(ax, [cue(k) fin(k) fin(k) cue(k)], [yl(1) yl(1) yl(2) yl(2)], c, ...
          'EdgeColor','none', 'HandleVisibility','off');
end
end
