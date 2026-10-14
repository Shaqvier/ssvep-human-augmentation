function R = plot_wrist_rotation_groups(folder, varargin)
%PLOT_WRIST_ROTATION_GROUPS  Wrist rotation per cued trial, early vs late runs.
%
%   R = PLOT_WRIST_ROTATION_GROUPS(FOLDER)
%   R = PLOT_WRIST_ROTATION_GROUPS(FOLDER, 'Name', Value, ...)
%
%   Sorts the recordings in FOLDER chronologically, splits them into the first
%   N and the last N (default 4 each), and draws one figure with two panels.
%   Each panel holds two boxes:
%
%       CCW (cued class 0, Red  16 Hz, unscrew)   red
%       CW  (cued class 1, Blue 19 Hz, screw)     green
%
%   ONE DATA POINT = ONE CUED TRIAL, and trials are grouped by what the
%   SEQUENCE asked for, never by what the robot did. The Arduino sketch runs a
%   fixed order, SEQUENCE = {0,1,0,1,0,1,1,0,1,0}, five of each per run, so four
%   runs give exactly 20 CCW and 20 CW. Grouping by the observed direction
%   instead would quietly move a failed trial into the other box and make both
%   boxes look better than the system performed.
%
%   SIGN CONVENTION. The plotted value is the trial's net rotation measured
%   TOWARDS THE CUED DIRECTION:
%
%       value = net_rotation * cue_direction        (CCW = -1, CW = +1)
%
%   so it is the magnitude of the rotation, POSITIVE when the wrist turned the
%   way the cue asked and NEGATIVE when it turned the opposite way. A cued-right
%   trial cued CW in which the robot unscrewed 50 deg plots at -50, in the CW box.
%   Zero means the wrist never moved.
%
%   Net rotation is measured from the home angle, so the first step of the trial
%   is included: the logged wrist_jog value is the TARGET after each step, and
%   the pre-jog angle is never logged, so (last - first) would miss one step.
%
%   Name-value options
%     'NGroup'      scalar  runs per group (default 4)
%     'Labels'      cell    two panel titles
%                           (default {'BCI task','BCI and bimanual task'})
%     'Sequence'    vector  cued class per trial, 0 = CCW, 1 = CW
%                           (default [0 1 0 1 0 1 1 0 1 0], from the sketch)
%     'MaxRot'      scalar  reference maximum in degrees, used for the per-cent
%                           figures and the dashed lines (default 200)
%     'HomeJ7Deg'   scalar  joint-7 angle at home (default 89.95 = 1.57 rad)
%     'Participant' char    ID for the filenames (default: the folder name)
%     'OutDir'      char    output folder (default <FOLDER>/figures)
%     'Save'        logical write the PNG and summary .mat (default true)
%     'WriteCSV'    logical also write the trial table as CSV (default false)
%     'Visible'     logical show the figure (default false)
%
%   Nothing is ever dropped. If a recording's classifier cadence differs from
%   the rest it is reported and kept.

p = inputParser;
p.addParameter('NGroup',      4,     @isnumeric);
p.addParameter('Labels',      {'BCI task','BCI and bimanual task'}, @iscell);
p.addParameter('Sequence',    [0 1 0 1 0 1 1 0 1 0], @isnumeric);
p.addParameter('MaxRot',      200,   @isnumeric);
p.addParameter('HomeJ7Deg',   rad2deg(1.57), @isnumeric);
p.addParameter('Participant', '',    @(x)ischar(x)||isstring(x));
p.addParameter('OutDir',      '',    @(x)ischar(x)||isstring(x));
p.addParameter('Save',        true,  @islogical);
p.addParameter('WriteCSV',    false, @islogical);
p.addParameter('Visible',     false, @islogical);
p.parse(varargin{:});
opt = p.Results;

assert(isfolder(folder), 'Folder not found: %s', folder);
if isempty(opt.OutDir), opt.OutDir = fullfile(folder, 'figures'); end
cueDir = 2*opt.Sequence(:)' - 1;          % class 0 -> -1 (CCW), 1 -> +1 (CW)

% ── chronological file list ─────────────────────────────────────────────────
d = dir(fullfile(folder, '*.mat')); d = d(~[d.isdir]);
stamp = NaT(numel(d),1); keep = false(numel(d),1);
for i = 1:numel(d)
    tok = regexp(d(i).name, '(\d{8})T(\d{6})', 'tokens', 'once');
    if ~isempty(tok)
        keep(i) = true;
        stamp(i) = datetime([tok{1} tok{2}], 'InputFormat', 'yyyyMMddHHmmss');
    end
end
d = d(keep); [stamp, ord] = sort(stamp(keep)); d = d(ord);
nF = numel(d);
assert(nF >= 2, 'Need at least 2 timestamped recordings in %s', folder);

n = min(opt.NGroup, floor(nF/2));
if n < opt.NGroup
    warning('Only %d recordings — using %d per group to keep them disjoint.', nF, n);
end
grpIdx = {1:n, nF-n+1:nF};
fprintf('%d recordings; group 1 = runs 1-%d, group 2 = runs %d-%d\n', ...
        nF, n, nF-n+1, nF);

% ── per-trial rotations ─────────────────────────────────────────────────────
T = table();
for g = 1:2
    for i = grpIdx{g}
        fpath = fullfile(d(i).folder, d(i).name);
        S  = load(fpath, 'markers', 'fs');
        fs = 500; if isfield(S,'fs'), fs = double(S.fs); end
        [tr, per] = local_trials(S.markers, fs, opt);

        if ~isnan(per) && abs(per - 0.256) > 0.35*0.256
            fprintf(['  note: %s ran the classifier at %.1f Hz, not the usual ' ...
                     '~3.9 Hz (kept)\n'], d(i).name, 1/per);
        end
        if numel(tr) ~= numel(cueDir)
            warning(['%s has %d trials but the sequence has %d — pairing the ' ...
                     'first %d. Check this run.'], d(i).name, numel(tr), ...
                     numel(cueDir), min(numel(tr), numel(cueDir)));
        end

        for k = 1:min(numel(tr), numel(cueDir))
            cd_  = cueDir(k);
            val  = tr(k).net * cd_;              % + towards cue, - away from it
            T = [T; table(g, i, string(d(i).name), stamp(i), k, cd_, ...
                    tr(k).net, val, 100*val/opt.MaxRot, ...
                    sign(tr(k).net), sign(tr(k).net) == cd_, ...
                    tr(k).nseg, tr(k).nflip, ...
                'VariableNames', {'group','run','file','when','trial', ...
                    'cue_dir','net_deg','rot_deg','pct_of_max','moved_dir', ...
                    'agrees','n_segments','n_flips'})]; %#ok<AGROW>
        end
    end
end
assert(~isempty(T), 'No trials with wrist data found.');

% ── figure ──────────────────────────────────────────────────────────────────
CCW_COL = [0.80 0.15 0.15];
CW_COL  = [0.15 0.60 0.25];
COLS    = [CCW_COL; CW_COL];
NAMES   = {'CCW cued','CW cued'};

vis = 'off'; if opt.Visible, vis = 'on'; end
fig = figure('Color','w', 'Position', [60 60 1200 600], 'Visible', vis);
tl  = tiledlayout(fig, 1, 2, 'TileSpacing','compact', 'Padding','compact');

lim = max([max(abs(T.rot_deg))*1.30, opt.MaxRot*1.15]);
ax  = gobjects(1,2);

for g = 1:2
    ax(g) = nexttile(tl); hold(ax(g),'on'); grid(ax(g),'on');
    m    = T.group == g;
    vals = {T.rot_deg(m & T.cue_dir < 0), T.rot_deg(m & T.cue_dir > 0)};

    for b = 1:2
        y = vals{b};
        if isempty(y), continue; end
        boxchart(ax(g), repmat(b, numel(y), 1), y, 'BoxFaceColor', COLS(b,:), ...
                 'MarkerStyle','none', 'BoxWidth', 0.55);
        scatter(ax(g), b + 0.17*(rand(numel(y),1)-0.5), y, 22, 'filled', ...
                'MarkerFaceColor', [0.15 0.15 0.15], 'MarkerFaceAlpha', 0.40);
        mu = mean(y, 'omitnan');
        plot(ax(g), b, mu, 'd', 'MarkerSize', 9, 'MarkerFaceColor', COLS(b,:), ...
             'MarkerEdgeColor','k', 'LineWidth', 1);
        text(ax(g), b, lim*0.90, sprintf('%.0f\\circ\n(%.0f%%)', mu, ...
             100*mu/opt.MaxRot), 'HorizontalAlignment','center', ...
             'FontSize', 9, 'FontWeight','bold', 'Color', COLS(b,:));
    end

    yline(ax(g), 0, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1);
    if opt.MaxRot > 0
        yline(ax(g), opt.MaxRot, '--', 'Color', [0.45 0.45 0.45]);
        text(ax(g), 0.45, opt.MaxRot, sprintf(' max %.0f\\circ (100%%)', opt.MaxRot), ...
             'FontSize', 8, 'Color', [0.45 0.45 0.45], 'VerticalAlignment','bottom');
    end
    set(ax(g), 'XTick', 1:2, 'XTickLabel', NAMES);
    xlim(ax(g), [0.4 2.6]); ylim(ax(g), [-lim lim]);
    ylabel(ax(g), 'rotation towards the cued direction (\circ)');

    for b = 1:2
        text(ax(g), b, -lim + 0.045*2*lim, sprintf('n=%d', numel(vals{b})), ...
             'HorizontalAlignment','center', 'FontSize', 8, 'Color', [0.35 0.35 0.35]);
    end

    lbl = sprintf('group %d', g);
    if numel(opt.Labels) >= g, lbl = opt.Labels{g}; end
    nWrong = sum(m & ~T.agrees & T.moved_dir ~= 0);
    nStill = sum(m & T.moved_dir == 0);
    title(ax(g), sprintf(['%s  (runs %d-%d, %d trials)\n' ...
          '%d turned the wrong way, %d never moved'], lbl, ...
          grpIdx{g}(1), grpIdx{g}(end), sum(m), nWrong, nStill));
end
linkaxes(ax, 'y');

[~, fname] = fileparts(folder);
pid = opt.Participant; if isempty(pid), pid = fname; end
pid = char(string(pid));
title(tl, sprintf(['%s — wrist rotation per cued trial ' ...
      '(positive = turned as cued, negative = turned the other way)'], pid), ...
      'FontWeight','bold', 'Interpreter','none');

% ── per-group summary table ─────────────────────────────────────────────────
By = table();
for g = 1:2
    lbl = sprintf('group %d', g);
    if numel(opt.Labels) >= g, lbl = opt.Labels{g}; end
    for b = 1:2
        m = T.group == g;
        switch b
            case 1, m = m & T.cue_dir < 0; nm = "CCW";
            case 2, m = m & T.cue_dir > 0; nm = "CW";
        end
        y = T.rot_deg(m);
        if isempty(y), continue; end
        By = [By; table(string(pid), g, string(lbl), nm, numel(y), ...
                mean(y,'omitnan'), std(y,'omitnan'), median(y,'omitnan'), ...
                100*mean(y,'omitnan')/opt.MaxRot, ...
                sum(m & ~T.agrees & T.moved_dir ~= 0), sum(m & T.moved_dir == 0), ...
            'VariableNames', {'participant','group','group_label','cue', ...
                'n_trials','mean_deg','sd_deg','median_deg','mean_pct_of_max', ...
                'n_wrong_direction','n_no_movement'})]; %#ok<AGROW>
    end
end

fprintf('\n  group                | cue   |  n |     mean |    %% max |       SD | wrong | still\n');
for r = 1:height(By)
    fprintf('  %-20s | %-5s | %2d | %+8.1f | %7.1f%% | %8.1f | %5d | %5d\n', ...
            By.group_label(r), By.cue(r), By.n_trials(r), By.mean_deg(r), ...
            By.mean_pct_of_max(r), By.sd_deg(r), By.n_wrong_direction(r), ...
            By.n_no_movement(r));
end

% ── outputs ─────────────────────────────────────────────────────────────────
R = struct();
R.participant = string(pid);
R.generated   = datetime('now');
R.folder      = string(folder);
R.trials      = T;
R.by_group    = By;
R.files       = string({d.name})';
R.when        = stamp;
R.groups      = {grpIdx};
R.sequence    = opt.Sequence(:)';
R.max_rot_deg = opt.MaxRot;
R.opts        = opt;

if opt.Save
    if ~isfolder(opt.OutDir), mkdir(opt.OutDir); end
    stampStr = datestr(now, 'yyyymmddTHHMMSS');
    png = fullfile(opt.OutDir, sprintf('wrist_rotation_%s_%s.png', pid, stampStr));
    exportgraphics(fig, png, 'Resolution', 200);
    matOut = fullfile(opt.OutDir, sprintf('summary_wrist_%s_%s.mat', pid, stampStr));
    save(matOut, '-struct', 'R');
    fprintf('\n  figure  -> %s\n  summary -> %s\n', png, matOut);
    if opt.WriteCSV
        csvOut = fullfile(opt.OutDir, sprintf('trials_wrist_%s_%s.csv', pid, stampStr));
        writetable(T, csvOut);
        fprintf('  trials  -> %s\n', csvOut);
    end
end
if ~opt.Visible, close(fig); end
end

% ═════════════════════════════════════════════════════════════════════════════
function [tr, decPeriod] = local_trials(M, fs, opt)
%LOCAL_TRIALS  Net wrist rotation per trial, measured from the home angle.
mtype = local_cellstr(M.type);
if any(strcmp(M.Properties.VariableNames,'info'))
    minfo = local_cellstr(M.info);
else
    minfo = repmat({''}, numel(mtype), 1);
end
if any(strcmp(M.Properties.VariableNames,'sample'))
    mt = double(M.sample) / fs;
else
    mt = double(M.t_unix); mt = mt - mt(1);
end

td = mt(strcmp(mtype,'decision'));
decPeriod = NaN;
if numel(td) > 2, decPeriod = median(diff(td)); end

iEnd = find(strcmp(mtype,'trial_end'));
iCue = find(ismember(mtype, {'cue_onset','home_reached','trial_start'}));
iX   = find(strcmp(mtype,'experiment_start'), 1, 'first'); if isempty(iX), iX = 1; end

tr = struct('net',{},'nseg',{},'nflip',{});
for k = 1:numel(iEnd)
    if k == 1, lo = iX; else, lo = iEnd(k-1); end
    cand = iCue(iCue > lo & iCue < iEnd(k));
    if isempty(cand), st = lo; else, st = cand(end); end
    rows = st:iEnd(k);

    iw = rows(strcmp(mtype(rows),'wrist_jog'));
    if isempty(iw)
        tr(k) = struct('net', 0, 'nseg', 0, 'nflip', 0);   % wrist never moved
        continue;
    end
    ang = nan(1, numel(iw));
    for q = 1:numel(iw)
        v = sscanf(strrep(minfo{iw(q)}, ',', ' '), '%f');
        if numel(v) >= 3, ang(q) = v(3); end
    end
    u   = unwrap(ang * pi/180) * 180/pi;
    ref = opt.HomeJ7Deg + round((u(1) - opt.HomeJ7Deg)/360)*360;

    dirs = [];
    for q = rows(strcmp(mtype(rows),'jog_start'))
        v = sscanf(strrep(minfo{q}, ',', ' '), '%f');
        if numel(v) >= 2, dirs(end+1) = sign(v(2)); end %#ok<AGROW>
    end
    tr(k) = struct('net', u(end) - ref, 'nseg', numel(dirs), ...
                   'nflip', sum(diff(dirs) ~= 0));
end
end

function c = local_cellstr(x)
if isstring(x),   c = cellstr(x);
elseif iscell(x), c = cellfun(@char, x, 'UniformOutput', false);
elseif ischar(x), c = cellstr(x);
else,             c = arrayfun(@(v) char(string(v)), x, 'UniformOutput', false);
end
c = c(:);
end
