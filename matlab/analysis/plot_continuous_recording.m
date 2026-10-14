function plot_continuous_recording(target, varargin)
%PLOT_CONTINUOUS_RECORDING  Review figure for the continuous (screwdriver) runs.
%
%   PLOT_CONTINUOUS_RECORDING(FILE)      one recording
%   PLOT_CONTINUOUS_RECORDING(FOLDER)    every *.mat in the folder, oldest first
%   PLOT_CONTINUOUS_RECORDING(..., 'Name', Value, ...)
%
%   Three stacked panels sharing one time axis:
%     1  EEG, the seven occipital channels, band-passed and per-channel
%        normalised
%     2  FBCCA max score and margin per window, with the online thresholds
%     3  wrist rotation within each trial, unwrapped, with the final net
%        rotation labelled at the end of each trial
%
%   Trials span cue_onset -> trial_end and are shaded by the direction the
%   robot actually turned, taken from the jog_start marker:
%     GREEN  +1  screw   / right  (class 1, Blue 19 Hz)
%     RED    -1  unscrew / left   (class 0, Red  16 Hz)
%     GREY       no jog_start — the classifier never moved the wrist
%   Red shading therefore lines up with the Red 16 Hz stimulus that drives it.
%
%   Only trial boundaries are marked — individual classifier decisions are not
%   drawn. Panel 2 carries the score traces if you need to see window detail.
%
%   WRIST ANGLE: joint 7 wraps at +/-180 deg, and the logged wrist_net marker is
%   a raw end-minus-start subtraction that does not account for this — its
%   magnitude and even its sign are wrong on any trial that crosses the wrap.
%   Panel 3 unwraps the wrist_jog series and reports that instead. Both values
%   are printed to the console so the discrepancy is visible.
%
%   Name-value options
%     'Channels'    vector   EEG rows to draw (default [2 14 13 19 16 17 18])
%     'Band'        [lo hi]  band-pass for display (default [1 45])
%     'CorrThresh'  scalar   drawn on panel 2 (default 0.30)
%     'MarginThresh' scalar  drawn on panel 2 (default 0.05)
%     'StepDeg'     scalar   achieved rotation per step once moving, in
%                            degrees (default 11.55)
%     'FirstStepDeg' scalar  rotation of the first step from rest
%                            (default 14.32 = JOG_STEP_RAD)
%     'HomeJ7Deg'   scalar   joint-7 angle at home, the reference the wrist
%                            rotates away from (default 89.95 = 1.57 rad)
%     'NWindows'    scalar   classifier windows in a trial (default 20)
%     'VotesStart'  scalar   VOTES_START from the node (default 4)
%     'Save'        logical  write a PNG (default true). Figures are always
%                            rendered off-screen and closed, never displayed.
%
%   MAXIMUM ROTATION. Panel 3 is scaled to the most the wrist could physically
%   have turned in one trial, the same value in both directions:
%
%       steps = NWindows - (VotesStart - 1)          = 20 - 3 = 17
%       max   = FirstStepDeg + (steps - 1) * StepDeg
%             = 14.32 + 16 * 11.55 deg                = 199.1 deg
%
%   VotesStart-1 windows are subtracted because the run has to build before the
%   first step: the 4th consecutive same-class window is the one that both
%   starts the jog and takes the first step. That first step is worth the FULL
%   commanded 14.32 deg because the arm is stationary when it is issued; every
%   later step loses part of itself to trajectory overlap, hence StepDeg.
%
%   StepDeg is the ACHIEVED step, not the commanded one. The node adds
%   JOG_STEP_RAD (0.25 rad = 14.32 deg) to the MEASURED joint angle, which still
%   lags the previous 0.3 s trajectory when the next window arrives 0.259 s
%   later, so roughly 19%% of each commanded step is lost to that overlap. The
%   mean step actually logged is reported per file and a warning is raised if it
%   drifts from StepDeg, since that would change the scale.
%     'OutDir'      char     where PNGs go
%                            (default: <folder of the recordings>/figures)

p = inputParser;
p.addParameter('Channels',     [2 14 13 19 16 17 18], @isnumeric);
p.addParameter('Band',         [1 45], @(x)isnumeric(x)&&numel(x)==2);
p.addParameter('CorrThresh',   0.30,  @isnumeric);
p.addParameter('MarginThresh', 0.05,  @isnumeric);
p.addParameter('StepDeg',      11.55, @isnumeric);   % achieved deg per step
p.addParameter('FirstStepDeg', 14.32, @isnumeric);   % JOG_STEP_RAD, no lag
p.addParameter('HomeJ7Deg',    rad2deg(1.57), @isnumeric);  % HOME_WAYPOINT(7)
p.addParameter('NWindows',     20,    @isnumeric);   % 5.0 s / 0.25 s
p.addParameter('VotesStart',   4,     @isnumeric);   % windows to start jogging
p.addParameter('Save',         true,  @islogical);
p.addParameter('OutDir',       '',    @(x)ischar(x)||isstring(x));
p.parse(varargin{:});
opt = p.Results;

% ── gather files ────────────────────────────────────────────────────────────
if isfolder(target)
    d = dir(fullfile(target, '*.mat'));
    d = d(~[d.isdir]);
    stamp = NaT(numel(d),1); keep = false(numel(d),1);
    for i = 1:numel(d)
        tok = regexp(d(i).name, '(\d{8})T(\d{6})', 'tokens', 'once');
        if ~isempty(tok)
            keep(i) = true;
            stamp(i) = datetime([tok{1} tok{2}], 'InputFormat', 'yyyyMMddHHmmss');
        end
    end
    d = d(keep); [~, ord] = sort(stamp(keep)); d = d(ord);
    assert(~isempty(d), 'No timestamped .mat files in %s', target);
    files = fullfile({d.folder}, {d.name});
    baseDir = target;
else
    assert(isfile(target), 'Not found: %s', target);
    files = {target};
    baseDir = fileparts(target);
end
if isempty(opt.OutDir), opt.OutDir = fullfile(baseDir, 'figures'); end
if opt.Save && ~isfolder(opt.OutDir), mkdir(opt.OutDir); end
fprintf('Plotting %d recording(s)\n', numel(files));

for i = 1:numel(files)
    local_one(files{i}, opt);
end
end

% ═════════════════════════════════════════════════════════════════════════════
function local_one(fpath, opt)

RIGHT_COL = [0.15 0.60 0.25];      % +1 screw / right  -> green
LEFT_COL  = [0.80 0.15 0.15];      % -1 unscrew / left -> red
NONE_COL  = [0.55 0.55 0.55];
ALPHA     = 0.13;

S = load(fpath);
assert(isfield(S,'markers'), '%s has no markers table.', fpath);
fs = 500; if isfield(S,'fs'), fs = double(S.fs); end

[mtype, minfo, mt] = local_markers(S.markers, fs);

% ── trials: cue_onset -> trial_end, direction from jog_start ────────────────
iCue = find(ismember(mtype, {'cue_onset','home_reached','trial_start'}));
iEnd = find(strcmp(mtype, 'trial_end'));
iX   = find(strcmp(mtype, 'experiment_start'), 1, 'first');
if isempty(iX), iX = 1; end

Tr = struct([]);
for k = 1:numel(iEnd)
    if k == 1, lo = iX; else, lo = iEnd(k-1); end
    cand = iCue(iCue > lo & iCue < iEnd(k));
    if isempty(cand), st = lo; else, st = cand(end); end
    rows = st:iEnd(k);
    Tr(k).t0 = mt(st);
    Tr(k).t1 = mt(iEnd(k));

    % A trial is NOT one direction. _start_jog re-publishes jog_start on every
    % restart and on every direction flip, so pair each jog_start with the
    % jog_stop that follows it and treat each pair as its own segment.
    segs = struct('t0',{},'t1',{},'dir',{},'cls',{});
    cur  = [];
    for q = rows
        switch mtype{q}
            case 'jog_start'
                v = sscanf(strrep(minfo{q}, ',', ' '), '%f');
                if numel(v) < 2, continue; end
                if ~isempty(cur)        % flip with no intervening stop
                    segs(end+1) = struct('t0',cur.t0,'t1',mt(q), ...
                                         'dir',cur.dir,'cls',cur.cls); %#ok<AGROW>
                end
                cur = struct('t0',mt(q),'t1',NaN,'dir',sign(v(2)),'cls',v(1));
            case 'jog_stop'
                if ~isempty(cur)
                    segs(end+1) = struct('t0',cur.t0,'t1',mt(q), ...
                                         'dir',cur.dir,'cls',cur.cls); %#ok<AGROW>
                    cur = [];
                end
        end
    end
    if ~isempty(cur)
        segs(end+1) = struct('t0',cur.t0,'t1',mt(iEnd(k)), ...
                             'dir',cur.dir,'cls',cur.cls); %#ok<AGROW>
    end
    Tr(k).segs = segs;
    dirs = [segs.dir];
    Tr(k).nflip = sum(diff(dirs) ~= 0);
    Tr(k).dir   = 0;
    if ~isempty(dirs), Tr(k).dir = sign(sum(dirs)); end   % dominant, for tallies

    % wrist trace, unwrapped
    iw = rows(strcmp(mtype(rows), 'wrist_jog'));
    ang = []; tw = [];
    for q = iw
        v = sscanf(strrep(minfo{q}, ',', ' '), '%f');
        if numel(v) >= 3, ang(end+1) = v(3); tw(end+1) = mt(q); end %#ok<AGROW>
    end
    if numel(ang) >= 1
        u = unwrap(ang * pi/180) * 180/pi;      % undo the +/-180 joint wrap
        % The logged angle is the TARGET after each step, so the first step's
        % rotation is not spanned by (last - first): the pre-jog angle is never
        % logged. Joint 7 always starts a trial at the home angle, so measure
        % from there and the first step is included.
        ref = opt.HomeJ7Deg + round((u(1) - opt.HomeJ7Deg)/360)*360;
        Tr(k).tw    = tw;
        Tr(k).rel   = u - ref;
        Tr(k).net   = u(end) - ref;
        Tr(k).first_offset = u(1) - ref;        % should be +/- FirstStepDeg
    else
        Tr(k).tw = []; Tr(k).rel = []; Tr(k).net = NaN; Tr(k).first_offset = NaN;
    end
    iN = rows(strcmp(mtype(rows), 'wrist_net'));
    Tr(k).net_logged = NaN;
    if ~isempty(iN), Tr(k).net_logged = str2double(minfo{iN(1)}); end
end
allSeg = [Tr.segs];
nR = sum([allSeg.dir] > 0); nL = sum([allSeg.dir] < 0);
nJ = numel(allSeg); nFlip = sum([Tr.nflip] > 0); nNo = sum(arrayfun(@(x)isempty(x.segs), Tr));

% ── maximum rotation physically reachable in one trial ──────────────────────
nStepMax = opt.NWindows - (opt.VotesStart - 1);
maxRot   = opt.FirstStepDeg + (nStepMax - 1) * opt.StepDeg;
allStep = [];
for k = 1:numel(Tr)
    if numel(Tr(k).rel) > 1, allStep = [allStep, abs(diff(Tr(k).rel))]; end %#ok<AGROW>
end
stepSeen = mean(allStep, 'omitnan');
fo = abs([Tr.first_offset]);
fo = fo(~isnan(fo));
if ~isempty(fo) && any(abs(fo - opt.FirstStepDeg) > 2)
    warning(['First logged target sits %.2f deg from home on some trials, ' ...
             'not the expected %.2f. HomeJ7Deg may be wrong for this session ' ...
             '— every net rotation would then be offset by the difference.'], ...
             max(abs(fo - opt.FirstStepDeg)) + opt.FirstStepDeg, opt.FirstStepDeg);
end
if ~isnan(stepSeen) && abs(stepSeen - opt.StepDeg) > 0.1 * opt.StepDeg
    warning(['Mean logged step here is %.2f deg, but the scale assumes ' ...
             '%.2f deg. The %%-of-maximum figures will be off — pass ' ...
             '''StepDeg'',%.2f if this run really did step differently.'], ...
             stepSeen, opt.StepDeg, stepSeen);
end

% ── decisions ───────────────────────────────────────────────────────────────
iD = find(strcmp(mtype, 'decision'));
tD = mt(iD); mxD = nan(size(iD)); mrD = nan(size(iD));
for q = 1:numel(iD)
    v = regexp(minfo{iD(q)}, 'max=([-\d\.]+)', 'tokens', 'once');
    if ~isempty(v), mxD(q) = str2double(v{1}); end
    v = regexp(minfo{iD(q)}, 'mar=([-\d\.]+)', 'tokens', 'once');
    if ~isempty(v), mrD(q) = str2double(v{1}); end
end

% ── EEG ─────────────────────────────────────────────────────────────────────
X = []; tE = [];
if isfield(S,'eeg') && ~isempty(S.eeg)
    E = S.eeg;
    if size(E,1) > size(E,2), E = E.'; end       % force channels x samples
    ch = opt.Channels(opt.Channels <= size(E,1));
    X  = double(E(ch, :));
    tE = (0:size(X,2)-1) / fs;
    try
        [b, a] = butter(4, opt.Band / (fs/2), 'bandpass');
        X = filtfilt(b, a, X.').';
    catch
        warning('Band-pass failed — drawing the raw signal.');
    end
    X = (X - mean(X, 2, 'omitnan')) ./ max(std(X, 0, 2, 'omitnan'), eps);
end

% ── figure ──────────────────────────────────────────────────────────────────
[~, nm] = fileparts(fpath);
fig = figure('Color','w', 'Position', [40 40 1500 900], 'Name', nm, ...
             'Visible', 'off');   % rendered off-screen; saved, never shown
tl  = tiledlayout(fig, 3, 1, 'TileSpacing','compact', 'Padding','compact');
tEndAll = max([mt(end), 1]);

ax1 = nexttile(tl); hold(ax1,'on');
if ~isempty(X)
    off = 4 * (size(X,1):-1:1);
    for c = 1:size(X,1)
        plot(ax1, tE, X(c,:) + off(c), 'k-', 'LineWidth', 0.25);
    end
    set(ax1, 'YTick', sort(off), 'YTickLabel', ...
             compose('ch %d', opt.Channels(end:-1:1)));
    ylim(ax1, [min(off)-4, max(off)+4]);
else
    text(ax1, 0.5, 0.5, 'no EEG in file', 'Units','normalized', ...
         'HorizontalAlignment','center');
end
ylabel(ax1, sprintf('EEG (%g-%g Hz, normalised)', opt.Band(1), opt.Band(2)));

ax2 = nexttile(tl); hold(ax2,'on');
plot(ax2, tD, mxD, '-', 'Color', [0.15 0.15 0.30], 'LineWidth', 0.9);
plot(ax2, tD, mrD, '-', 'Color', [0.10 0.65 0.55], 'LineWidth', 0.9);
yline(ax2, opt.CorrThresh,   ':', sprintf('corr %.2f', opt.CorrThresh), ...
      'Color', [0.15 0.15 0.30], 'LabelHorizontalAlignment','left');
yline(ax2, opt.MarginThresh, ':', sprintf('margin %.2f', opt.MarginThresh), ...
      'Color', [0.10 0.65 0.55], 'LabelHorizontalAlignment','left');
ylabel(ax2, 'FBCCA score');
legend(ax2, {'max score','margin'}, 'Location','northeast', 'FontSize', 8, ...
       'AutoUpdate','off');

ax3 = nexttile(tl); hold(ax3,'on');
yline(ax3, 0, '-', 'Color', [0.7 0.7 0.7]);
for k = 1:numel(Tr)
    if isempty(Tr(k).tw), continue; end
    for q = 1:numel(Tr(k).segs)
        sg = Tr(k).segs(q);
        m  = Tr(k).tw >= sg.t0 - 1e-6 & Tr(k).tw <= sg.t1 + 1e-6;
        if nnz(m) < 2, m = m | circshift(m, -1); end   % keep single-step jogs visible
        if ~any(m), continue; end
        plot(ax3, Tr(k).tw(m), Tr(k).rel(m), '-', 'LineWidth', 1.6, ...
             'Color', local_dircol(sg.dir, RIGHT_COL, LEFT_COL, NONE_COL));
    end
    col = local_dircol(sign(Tr(k).net), RIGHT_COL, LEFT_COL, NONE_COL);
    plot(ax3, Tr(k).tw(end), Tr(k).net, 'o', 'MarkerSize', 5, ...
         'MarkerFaceColor', col, 'MarkerEdgeColor', 'k');
    text(ax3, Tr(k).tw(end), Tr(k).net, ...
         sprintf('  %+.0f\\circ (%.0f%%)%s', Tr(k).net, 100*abs(Tr(k).net)/maxRot, ...
                 local_iif(Tr(k).nflip > 0, ' flip', '')), ...
         'Color', col, 'FontSize', 8, 'FontWeight','bold', ...
         'VerticalAlignment', local_iif(Tr(k).net >= 0, 'bottom', 'top'));
end
ylim(ax3, [-maxRot maxRot]);
yline(ax3, maxRot, '--', 'Color', [0.4 0.4 0.4]);
yline(ax3, -maxRot, '--', 'Color', [0.4 0.4 0.4]);
text(ax3, 0, maxRot, sprintf(' 100%% = %+.0f\\circ ', maxRot), 'FontSize', 8, ...
     'Color', [0.4 0.4 0.4], 'VerticalAlignment','top');
text(ax3, 0, -maxRot, sprintf(' 100%% = %+.0f\\circ ', -maxRot), 'FontSize', 8, ...
     'Color', [0.4 0.4 0.4], 'VerticalAlignment','bottom');
ylabel(ax3, 'wrist rotation (\circ, unwrapped)');
xlabel(ax3, 'time (s)');

% shading + trial boundaries on every panel
for ax = [ax1 ax2 ax3]
    if ax == ax3, ylim(ax3, [-maxRot maxRot]); end
    yl = ylim(ax);
    for k = 1:numel(Tr)
        patch(ax, [Tr(k).t0 Tr(k).t1 Tr(k).t1 Tr(k).t0], [yl(1) yl(1) yl(2) yl(2)], ...
              NONE_COL, 'FaceAlpha', 0.06, 'EdgeColor','none', ...
              'HandleVisibility','off');
        for q = 1:numel(Tr(k).segs)
            sg  = Tr(k).segs(q);
            col = local_dircol(sg.dir, RIGHT_COL, LEFT_COL, NONE_COL);
            patch(ax, [sg.t0 sg.t1 sg.t1 sg.t0], [yl(1) yl(1) yl(2) yl(2)], ...
                  col, 'FaceAlpha', ALPHA, 'EdgeColor','none', ...
                  'HandleVisibility','off');
        end
        xline(ax, Tr(k).t0, '-',  'Color', [0.25 0.25 0.25], 'LineWidth', 1.1);
        xline(ax, Tr(k).t1, '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.1);
    end
    ylim(ax, yl); xlim(ax, [0 tEndAll]);
    grid(ax, 'on');
    set(ax, 'Layer', 'top');
end
for k = 1:numel(Tr)
    text(ax1, Tr(k).t0, max(ylim(ax1)), sprintf(' %d', k), 'FontSize', 8, ...
         'VerticalAlignment','top', 'Color', [0.35 0.35 0.35]);
end
linkaxes([ax1 ax2 ax3], 'x');

hR = patch(ax1, NaN, NaN, RIGHT_COL, 'FaceAlpha', 0.35, 'EdgeColor','none');
hL = patch(ax1, NaN, NaN, LEFT_COL,  'FaceAlpha', 0.35, 'EdgeColor','none');
hN = patch(ax1, NaN, NaN, NONE_COL,  'FaceAlpha', 0.35, 'EdgeColor','none');
legend(ax1, [hR hL hN], ...
       {'jogging right (+1, class 1, Blue 19 Hz)', ...
        'jogging left (-1, class 0, Red 16 Hz)', 'trial, wrist idle'}, ...
       'Location','northoutside', 'Orientation','horizontal', 'FontSize', 8);

title(tl, sprintf(['%s  —  %.0f s, %g Hz, %d trials, %d jog segments ' ...
      '(%d right, %d left) | %d trial(s) flipped direction, %d with no jog'], ...
      nm, tEndAll, fs, numel(Tr), nJ, nR, nL, nFlip, nNo), ...
      'FontWeight','bold', 'Interpreter','none');

% ── console table for the cleaning pass ─────────────────────────────────────
fprintf('\n  %s\n', nm);
fprintf(['  max reachable rotation = %.1f deg  (%d steps: %.2f + %d x %.2f ' ...
         'deg, measured from home %.2f deg)\n'], maxRot, nStepMax, ...
        opt.FirstStepDeg, nStepMax-1, opt.StepDeg, opt.HomeJ7Deg);
if ~isnan(stepSeen)
    fprintf('  mean logged step here  = %.2f deg\n', stepSeen);
end
fprintf('  trial |  start |    end | jogs | flips | segments (dir @ start-stop s) | net rot | %% of max\n');
for k = 1:numel(Tr)
    segTxt = strings(1, numel(Tr(k).segs));
    for q = 1:numel(Tr(k).segs)
        sg = Tr(k).segs(q);
        segTxt(q) = sprintf('%s@%.2f-%.2f', local_iif(sg.dir > 0, '+1', '-1'), ...
                            sg.t0 - Tr(k).t0, sg.t1 - Tr(k).t0);
    end
    fprintf('   %3d  | %6.1f | %6.1f | %4d | %5d | %-29s | %+7.1f | %7.0f%%\n', ...
            k, Tr(k).t0, Tr(k).t1, numel(Tr(k).segs), Tr(k).nflip, ...
            strjoin(segTxt, ' '), Tr(k).net, 100*abs(Tr(k).net)/maxRot);
end

if opt.Save
    outDir = opt.OutDir;
    if ~isfolder(outDir), mkdir(outDir); end
    png = fullfile(outDir, sprintf('overview_%s.png', nm));
    exportgraphics(fig, png, 'Resolution', 180);
    fprintf('  saved -> %s\n', png);
end
close(fig);
end

% ═════════════════════════════════════════════════════════════════════════════
function c = local_dircol(dir, R, L, N)
if dir > 0, c = R; elseif dir < 0, c = L; else, c = N; end
end

function v = local_iif(c, a, b)
if c, v = a; else, v = b; end
end

function [mtype, minfo, mt] = local_markers(M, fs)
mtype = local_cellstr(M.type);
if any(strcmp(M.Properties.VariableNames, 'info'))
    minfo = local_cellstr(M.info);
else
    minfo = repmat({''}, numel(mtype), 1);
end
if any(strcmp(M.Properties.VariableNames, 'sample'))
    mt = double(M.sample) / fs;
else
    mt = double(M.t_unix); mt = mt - mt(1);
end
mt = mt(:);
end

function c = local_cellstr(x)
if isstring(x),   c = cellstr(x);
elseif iscell(x), c = cellfun(@char, x, 'UniformOutput', false);
elseif ischar(x), c = cellstr(x);
else,             c = arrayfun(@(v) char(string(v)), x, 'UniformOutput', false);
end
c = c(:);
end
