function R = analyze_discrete_online(folder, varargin)
%ANALYZE_DISCRETE_ONLINE  Batch analysis of discrete online SSVEP-BCI runs.
%
%   R = ANALYZE_DISCRETE_ONLINE(FOLDER)
%   R = ANALYZE_DISCRETE_ONLINE(FOLDER, 'Name', Value, ...)
%
%   Scans FOLDER for *.mat recordings produced by record_ssvep_continuous.m,
%   sorts them chronologically from the yyyymmddTHHMMSS stamp in the filename,
%   prompts once per run for the cue configuration, then produces:
%
%     1. Time from cue onset to command, minus the cue duration.
%        Negative results are excluded from the timing stats and counted as
%        false positives (delivered class == cued class) or false negatives
%        (delivered class ~= cued class).
%     2. Robot completion time per trial (trial_end - command, and
%        trial_end - cue). Trials with no command (classifier timeout) and
%        trials flagged as tag-not-found are stripped out and counted.
%     3. Confusion matrix of the online decisions (cued vs delivered class).
%     4. Per-run outcome tally.
%
%   Cued and uncued runs in the same folder are pooled; the condition is
%   tagged per trial in the output table so they can be split later.
%
%   Junk data (a run or individual trials where something went wrong) can be
%   scrapped from the per-run dialog. Scrapped trials are ARCHIVED, not
%   deleted: they stay in the output table with 'scrapped' = true and the
%   reason recorded, but are excluded from every statistic and from all four
%   panels except the outcome tally, where they show as a separate grey bar.
%
%   Outputs ONE figure and ONE summary .mat file.
%
%   Name-value options
%     'Participant'  char     participant ID (prompted if omitted)
%     'OutDir'       char     where to write outputs (default = FOLDER)
%     'WriteCSV'     logical  also dump the trial table as CSV (default false)
%     'MaxTrials'    scalar   trials past this index are auto-scrapped as
%                             recording-save padding (default 4)
%
%   Timing note: all times come from the marker 'sample' column divided by fs,
%   i.e. the EEG clock, which is monotone and free of the NaNs in t_lsl.
%
%   Trial segmentation is driven by trial_end, the only marker every trial is
%   guaranteed to have. Start markers are used where present; where a node
%   build publishes none, trial 1 starts at experiment_start and each later
%   trial starts at the previous trial_end (the robot reaching home, which is
%   when the Arduino resumes flickering). The console reports how many starts
%   were inferred in each run.
%
%   Known data limits (not fixable in analysis):
%     - The cue class is not written to the recording, so the cued frequency
%       per trial is entered by hand in the per-run dialog.
%     - trial_end:'r' is published both on success and on scan failure, so
%       trials where nothing was delivered must also be entered by hand.
%     - The uncued node advances on DELIVERY, not on homing, so a failed pick
%       or a lost tag produces an extra trial: an uncued run can contain 5 or
%       more trial_end markers for 4 delivered objects. The trial count is
%       therefore taken from the markers, never assumed to be 4, and the
%       dialog refuses to continue until the cue labels match that count.

% ─────────────────────────────────────────────────────────────────────────────
% 0. Options
% ─────────────────────────────────────────────────────────────────────────────
p = inputParser;
p.addParameter('Participant', '',    @(x)ischar(x)||isstring(x));
p.addParameter('OutDir',      '',    @(x)ischar(x)||isstring(x));
p.addParameter('WriteCSV',    false, @islogical);
p.addParameter('MaxTrials',   4,     @isnumeric);   % trials beyond this are padding
p.parse(varargin{:});
opt = p.Results;

assert(isfolder(folder), 'Folder not found: %s', folder);
if isempty(opt.OutDir), opt.OutDir = folder; end
if ~isfolder(opt.OutDir), mkdir(opt.OutDir); end

% ─────────────────────────────────────────────────────────────────────────────
% 1. Find and chronologically sort the recordings
% ─────────────────────────────────────────────────────────────────────────────
d = dir(fullfile(folder, '*.mat'));
d = d(~[d.isdir]);
keep = false(numel(d),1); stamp = NaT(numel(d),1);
for i = 1:numel(d)
    tok = regexp(d(i).name, '(\d{8})T(\d{6})', 'tokens', 'once');
    if ~isempty(tok)
        keep(i)  = true;
        stamp(i) = datetime([tok{1} tok{2}], 'InputFormat', 'yyyyMMddHHmmss');
    end
end
d = d(keep); stamp = stamp(keep);
assert(~isempty(d), 'No files matching *yyyymmddTHHMMSS*.mat in %s', folder);
[stamp, ord] = sort(stamp); d = d(ord);
nRun = numel(d);
fprintf('Found %d run(s) in %s\n', nRun, folder);

if isempty(opt.Participant)
    a = inputdlg({'Participant ID'}, 'Session', [1 40], {'P01'});
    if isempty(a), R = []; return; end
    opt.Participant = strtrim(a{1});
end

% ─────────────────────────────────────────────────────────────────────────────
% 2. Per-run: load markers, segment trials, ask the operator for the config
% ─────────────────────────────────────────────────────────────────────────────
runs = struct([]);
DEFAULT_SEQ = [16 23 19 29];   % SEQUENCE {0,2,1,3} in sequence_experiment_cue.ino

for k = 1:nRun
    fpath = fullfile(d(k).folder, d(k).name);
    S = load(fpath, 'markers', 'fs', 'exp_id');            % never touches eeg
    assert(isfield(S,'markers'), 'No markers table in %s', d(k).name);
    fs = 500; if isfield(S,'fs'), fs = double(S.fs); end

    [mtype, minfo, mt] = local_markers(S.markers, fs);
    [iStart, iEnd, nDerived] = local_segment(mtype);
    nTr                = numel(iStart);
    if nDerived > 0
        fprintf(['  run %d (%s): %d of %d trial start(s) inferred — no start ' ...
                 'marker in the recording.\n'], k, d(k).name, nDerived, nTr);
    end

    isUncued = contains(lower(d(k).name), 'uncued') || ...
               any(strcmp(mtype(iStart), 'home_reached')) || nDerived > 0;

    % Pre-scan: what did the robot actually deliver, and how long did each
    % trial take? Printed before the dialog so the trial count and order can
    % be reconciled against the run sheet BEFORE the cue labels are typed.
    preHz = nan(nTr,1); preDur = nan(nTr,1);
    for j = 1:nTr
        rr = iStart(j)+1 : iEnd(j)-1;
        rr = rr(strcmp(mtype(rr), 'decision'));
        [~, w_, p_, u_] = local_decisions(minfo(rr), mt(rr), mt(iStart(j)));
        ipre = find(u_ == 1, 1, 'first');
        if isempty(ipre), ipre = local_firstrun(w_, p_, 4); end
        if ~isempty(ipre), preHz(j) = w_(ipre); end
        preDur(j) = mt(iEnd(j)) - mt(iStart(j));
    end
    fprintf('\n  run %d/%d  %s  (%s)\n', k, nRun, d(k).name, ...
            ternary(isUncued, 'uncued', 'cued'));
    fprintf('    trial | start (s) | delivered (Hz) | trial length (s)\n');
    for j = 1:nTr
        fprintf('      %2d  |  %8.1f |  %12s  |  %10.1f  %s\n', ...
                j, mt(iStart(j)) - mt(1), ...
                ternary(isnan(preHz(j)), 'none', num2str(preHz(j))), preDur(j), ...
                ternary(j > opt.MaxTrials, '<- padding, auto-scrapped', ''));
    end
    nExpect = min(nTr, opt.MaxTrials);

    defSeq = DEFAULT_SEQ(mod(0:max(nTr,1)-1, 4) + 1);
    delivTxt = strjoin(arrayfun(@(v) ternary(isnan(v),'none',num2str(v)), ...
                                preHz(:)', 'UniformOutput', false), ',');
    prompt = { 'Stimulus frequencies present (Hz) — sets the matrix axes:', ...
               sprintf(['Cued Hz for trials 1-%d of %d; delivered %s  ' ...
                        '(0 = unknown):'], nExpect, nTr, delivTxt), ...
               'Condition (cued / uncued):', ...
               'Cue duration to subtract (s):', ...
               'Trials with NO delivery (tag lost / failed pick), blank = none:', ...
               'SCRAP (blank = keep all; ''all'' = scrap whole run; or trial indices):', ...
               'Scrap reason (free text, optional):' };
    defs   = { '16, 19, 23, 29', ...
               ternary(isUncued, '', strjoin(string(defSeq(1:nExpect)), ', ')), ...
               ternary(isUncued, 'uncued', 'cued'), ...
               num2str(1.5 * ~isUncued), ...
               '', '', '' };
    ttl = sprintf('%s  %s  |  run %d/%d  |  %s%s', opt.Participant, ...
                  datestr(stamp(k), 'dd mmm yyyy  HH:MM:SS'), k, nRun, d(k).name, ...
                  ternary(nDerived > 0, sprintf('  [%d start(s) inferred]', nDerived), ''));
    while true
        ans_ = inputdlg(prompt, ttl, [1 70], defs);
        if isempty(ans_), R = []; fprintf('Cancelled.\n'); return; end
        lblTry = local_nums(ans_{2});
        nLbl   = numel(lblTry);
        if nLbl == nExpect
            lblNZ = lblTry(lblTry ~= 0);
            if numel(unique(lblNZ)) == numel(lblNZ), break; end
            q = questdlg(sprintf(['You entered a repeated frequency: %s\n\n' ...
                'A cue sequence normally contains each frequency once per ' ...
                'run. A repeat usually means the DELIVERED sequence was ' ...
                'typed instead of the CUE — which scores the trial as ' ...
                'correct and hides a real error.\n\nKeep it anyway?'], ...
                strjoin(string(lblTry), ', ')), 'Repeated cue frequency', ...
                'Re-enter', 'Keep', 'Re-enter');
            if strcmp(q, 'Keep'), break; end
        else
            uiwait(warndlg(sprintf(['%d cue label(s) for %d scored trial(s).\n\n' ...
                'A mismatch silently shifts every label and makes the whole run ' ...
                'look wrong. Enter exactly %d values; use 0 for any trial you ' ...
                'cannot label (e.g. a repeat attempt after a failed delivery).'], ...
                nLbl, nExpect, nExpect), 'Cue label count', 'modal'));
        end
        defs = ans_;
        continue
    end

    fset    = local_nums(ans_{1});
    cueHz   = local_nums(ans_{2});
    cueHz(cueHz == 0) = NaN;                 % 0 = deliberately unlabelled
    cueHz(end+1:nTr)  = NaN;                 % padding trials carry no label
    cond    = lower(strtrim(ans_{3}));
    cueDur  = str2double(ans_{4}); if isnan(cueDur), cueDur = 0; end
    noDeliv = local_nums(ans_{5});
    scrapRaw   = strtrim(ans_{6});
    scrapWhole = strcmpi(scrapRaw, 'all') || strcmpi(scrapRaw, 'run');
    scrapTr    = local_nums(scrapRaw);
    scrapWhy   = strtrim(ans_{7});
    if (scrapWhole || ~isempty(scrapTr)) && isempty(scrapWhy)
        scrapWhy = 'unspecified';
    end
    if scrapWhole
        fprintf('  run %d (%s) scrapped: %s\n', k, d(k).name, scrapWhy);
    elseif ~isempty(scrapTr)
        fprintf('  run %d: trial(s) %s scrapped: %s\n', k, ...
                strjoin(string(scrapTr), ','), scrapWhy);
    end

    runs(k).file    = d(k).name;
    runs(k).path    = fpath;
    runs(k).when    = stamp(k);
    runs(k).fs      = fs;
    runs(k).fset    = fset(:)';
    runs(k).cueHz   = cueHz(1:nTr);
    runs(k).cond    = cond;
    runs(k).cueDur  = cueDur;
    runs(k).noDeliv    = noDeliv;
    runs(k).scrapWhole = scrapWhole;
    runs(k).scrapTr    = scrapTr;
    runs(k).scrapWhy   = scrapWhy;
    runs(k).mtype   = mtype;  runs(k).minfo = minfo;  runs(k).mt = mt;
    runs(k).iStart  = iStart; runs(k).iEnd = iEnd;    runs(k).nTr = nTr;
    runs(k).n_derived_starts = nDerived;
end

% ─────────────────────────────────────────────────────────────────────────────
% 3. Per-trial metrics
% ─────────────────────────────────────────────────────────────────────────────
T = table();

for k = 1:nRun
    r = runs(k);
    for j = 1:r.nTr
        i0 = r.iStart(j);
        i1 = r.iEnd(j);
        tCue = r.mt(i0);

        rows = i0+1 : i1-1;
        rows = rows(strcmp(r.mtype(rows), 'decision'));
        [dt, win, pass, pub] = local_decisions(r.minfo(rows), r.mt(rows), tCue);

        % ── online command: first pub=1, else re-derive the 4-in-a-row rule
        ip = find(pub == 1, 1, 'first');
        if isempty(ip), ip = local_firstrun(win, pass, 4); end
        if isempty(ip)
            tCmd = NaN; predHz = NaN;
        else
            tCmd = dt(ip); predHz = win(ip);
        end
        tCmdCorr = tCmd - r.cueDur;          % metric 1

        % ── trial end / completion
        endReason = 'r';
        if ~isempty(r.minfo{i1}), endReason = lower(r.minfo{i1}(1)); end
        tEnd    = r.mt(i1) - tCue;
        tExec   = tEnd - tCmd;               % robot execution time
        timeout = endReason == 't' || isnan(tCmd);
        noDel   = ismember(j, r.noDeliv);

        padding  = j > opt.MaxTrials;
        scrapped = r.scrapWhole || ismember(j, r.scrapTr) || padding;
        scrapWhy = "";
        if padding,       scrapWhy = "padding trial (recording-save check)";
        elseif scrapped,  scrapWhy = string(r.scrapWhy);
        end

        cueHz   = r.cueHz(j);
        correct = ~isnan(predHz) && ~isnan(cueHz) && predHz == cueHz;
        early   = ~isnan(tCmdCorr) && tCmdCorr < 0;

        fpFlag = early &&  correct;          % early but right  -> false positive
        fnFlag = early && ~correct;          % early and wrong  -> false negative

        T = [T; table( string(opt.Participant), k, j, string(r.cond), r.when, ...
                string(r.file), cueHz, predHz, correct, ...
                tCmd, tCmdCorr, early, fpFlag, fnFlag, ...
                tEnd, tExec, timeout, noDel, string(endReason), numel(dt), ...
                scrapped, scrapWhy, ...
            'VariableNames', {'participant','run','trial','condition','when', ...
                'file','cue_hz','pred_hz','correct','t_cmd_from_cue', ...
                't_cmd_corrected','early','false_pos','false_neg', ...
                't_trial_end','t_robot_exec','classifier_timeout','no_delivery', ...
                'end_reason','n_windows','scrapped','scrap_reason'})]; %#ok<AGROW>
    end
end

% ─────────────────────────────────────────────────────────────────────────────
% 4. ONE figure
% ─────────────────────────────────────────────────────────────────────────────
clean   = ~T.scrapped;                       % archived trials never count
nScrap  = sum(T.scrapped);
nClean  = sum(clean);
valid   = clean & ~T.early & ~isnan(T.t_cmd_corrected);
nFalse  = sum(clean & T.early);
nFP     = sum(clean & T.false_pos);
nFN     = sum(clean & T.false_neg);
nTO     = sum(clean & T.classifier_timeout);
nND     = sum(clean & T.no_delivery);
okExec  = clean & ~T.classifier_timeout & ~T.no_delivery & ~isnan(T.t_robot_exec);
scored  = clean & ~isnan(T.cue_hz);          % trials with a known cue label
nScored = sum(scored);
nUnlab  = sum(clean & isnan(T.cue_hz));
acc     = sum(scored & T.correct) / max(nScored,1);

lbl = unique([[runs.fset]'; T.cue_hz; T.pred_hz]);
lbl = sort(lbl(~isnan(lbl)));

fig = figure('Name', sprintf('Discrete online — %s', opt.Participant), ...
             'Color', 'w', 'Position', [60 60 1350 880]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['%s — discrete online SSVEP-BCI  ' ...
      '(%d runs, %d trials analysed, %d scrapped)'], ...
      opt.Participant, nRun, nClean, nScrap), 'FontWeight', 'bold');

% (a) time from cue to command, grouped by the frequency being attended
ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
mA = valid & ~isnan(T.cue_hz);
local_boxgroup(ax, T.cue_hz(mA), T.t_cmd_corrected(mA), lbl, [0.20 0.45 0.70]);
xlabel(ax,'cued frequency (Hz)'); ylabel(ax,'time to command (s)');
title(ax, sprintf(['Cue \\rightarrow command (cue duration removed)\n' ...
    'mean %.2f \\pm %.2f s (SD)   |   false results = %d  (FP %d, FN %d)'], ...
    mean(T.t_cmd_corrected(valid),'omitnan'), std(T.t_cmd_corrected(valid),'omitnan'), ...
    nFalse, nFP, nFN));

% (b) robot completion time, grouped by the object actually picked
ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
mB = okExec & ~isnan(T.pred_hz);
local_boxgroup(ax, T.pred_hz(mB), T.t_robot_exec(mB), lbl, [0.75 0.35 0.15]);
xlabel(ax,'delivered frequency (Hz)'); ylabel(ax,'robot execution time (s)');
title(ax, sprintf(['Command \\rightarrow trial end\n' ...
    'mean %.1f \\pm %.1f s (SD)   |   no delivery = %d   |   classifier timeouts = %d'], ...
    mean(T.t_robot_exec(okExec),'omitnan'), std(T.t_robot_exec(okExec),'omitnan'), ...
    nND, nTO));

% (c) confusion matrix, with early (pre-stimulus) detections marked
ax = nexttile(tl);
C  = zeros(numel(lbl), numel(lbl)+1);         % last column = no command issued
CE = zeros(size(C));                          % subset of C that fired early
for i = find(clean)'
    a = find(lbl == T.cue_hz(i), 1);
    if isempty(a), continue; end
    if isnan(T.pred_hz(i))
        b = size(C,2);
    else
        b = find(lbl == T.pred_hz(i), 1);
        if isempty(b), continue; end
    end
    C(a,b) = C(a,b) + 1;
    if T.early(i), CE(a,b) = CE(a,b) + 1; end
end
imagesc(ax, C); colormap(ax, flipud(gray)); axis(ax,'equal','tight'); hold(ax,'on');
set(ax, 'XTick', 1:numel(lbl)+1, ...
        'XTickLabel', [compose('%g', lbl(:)'), {'none'}], ...
        'YTick', 1:numel(lbl), 'YTickLabel', compose('%g', lbl(:)'));
EARLY_COL = [0.85 0.10 0.55];
for a = 1:size(C,1)
    for b = 1:size(C,2)
        dy = ternary(CE(a,b) > 0, -0.13, 0);
        text(ax, b, a + dy, num2str(C(a,b)), 'HorizontalAlignment','center', ...
             'Color', ternary(C(a,b) > max(C(:))/2, 'w', 'k'), 'FontWeight','bold');
        if CE(a,b) > 0
            rectangle(ax, 'Position', [b-0.5 a-0.5 1 1], 'EdgeColor', EARLY_COL, ...
                      'LineWidth', 2.5);
            text(ax, b, a + 0.20, sprintf('%d early', CE(a,b)), ...
                 'HorizontalAlignment','center', 'Color', EARLY_COL, ...
                 'FontSize', 8, 'FontWeight','bold');
        end
    end
end
xlabel(ax,'delivered (Hz)'); ylabel(ax,'cued (Hz)');
title(ax, sprintf(['Online confusion matrix  —  accuracy %.1f%% (%d/%d labelled)\n' ...
      '\\color[rgb]{0.85,0.10,0.55}%d early (on-diagonal = FP, off = FN)' ...
      '\\color{black}%s'], 100*acc, sum(scored & T.correct), nScored, ...
      sum(clean & T.early), ...
      ternary(nUnlab > 0, sprintf('   |   %d unlabelled, excluded', nUnlab), '')));

% (d) per-run outcome tally
ax = nexttile(tl);
cats = zeros(nRun, 5);
for k = 1:nRun
    m  = T.run == k & clean;
    kn = ~isnan(T.cue_hz);
    cats(k,1) = sum(m &  T.correct & ~T.early);
    cats(k,2) = sum(m & ~T.correct & ~T.early & ~T.classifier_timeout &  kn);
    cats(k,3) = sum(m &  T.classifier_timeout);
    cats(k,4) = sum(m &  T.early);
    cats(k,5) = sum(m & ~T.early & ~T.classifier_timeout & ~kn);
end
bh = bar(ax, 1:nRun, cats, 'stacked'); grid(ax,'on');
bh(5).FaceColor = [0.55 0.55 0.55];
legend(ax, bh, {'correct','wrong class','no command (timeout)', ...
                'early / invalid','unlabelled cue'}, ...
       'Location','best', 'FontSize', 8);
ylim(ax, [0 max(sum(cats,2)) + 0.5]);
xlabel(ax,'run'); ylabel(ax,'trials'); xlim(ax,[0.5 nRun+0.5]);
title(ax, sprintf('Outcome per run  (pooled: %s; %d scrapped trial(s) not shown)', ...
      strjoin(unique(cellstr(T.condition))', ' + '), nScrap));

% ─────────────────────────────────────────────────────────────────────────────
% 5. ONE summary file
% ─────────────────────────────────────────────────────────────────────────────
R = struct();
R.participant = string(opt.Participant);
R.generated   = datetime('now');
R.folder      = string(folder);
R.trials      = T;
R.runs        = rmfield(runs, {'mtype','minfo','mt','iStart','iEnd'});
R.confusion   = struct('labels_hz', lbl(:)', 'counts', C, ...
                       'note', "rows = cued, columns = delivered, last column = no command");
R.summary     = struct( ...
    'n_runs',               nRun, ...
    'n_trials_recorded',    height(T), ...
    'n_trials_analysed',    nClean, ...
    'n_trials_scored',      nScored, ...
    'n_unlabelled',         nUnlab, ...
    'n_scrapped',           nScrap, ...
    'accuracy',             acc, ...
    'n_false_results',      nFalse, ...
    'n_false_positive',     nFP, ...
    'n_false_negative',     nFN, ...
    'n_classifier_timeout', nTO, ...
    'n_no_delivery',        nND, ...
    'mean_t_cmd_s',         mean(T.t_cmd_corrected(valid),'omitnan'), ...
    'sd_t_cmd_s',           std(T.t_cmd_corrected(valid),'omitnan'), ...
    'mean_exec_s',          mean(T.t_robot_exec(okExec),'omitnan'), ...
    'sd_exec_s',            std(T.t_robot_exec(okExec),'omitnan'), ...
    'mean_total_trial_s',   mean(T.t_trial_end(okExec),'omitnan'), ...
    'sd_total_trial_s',     std(T.t_trial_end(okExec),'omitnan'), ...
    'median_t_cmd_s',       median(T.t_cmd_corrected(valid),'omitnan'), ...
    'iqr_t_cmd_s',          iqr(T.t_cmd_corrected(valid)), ...
    'median_exec_s',        median(T.t_robot_exec(okExec),'omitnan'), ...
    'iqr_exec_s',           iqr(T.t_robot_exec(okExec)));
R.opts        = opt;

stampStr = datestr(now, 'yyyymmddTHHMMSS');
matOut = fullfile(opt.OutDir, sprintf('summary_discrete_%s_%s.mat', opt.Participant, stampStr));
save(matOut, '-struct', 'R');
figOut = fullfile(opt.OutDir, sprintf('figure_discrete_%s_%s.png', opt.Participant, stampStr));
exportgraphics(fig, figOut, 'Resolution', 200);
if opt.WriteCSV
    writetable(T, fullfile(opt.OutDir, sprintf('trials_%s_%s.csv', opt.Participant, stampStr)));
end

fprintf('\n  cue balance across the session (each frequency should appear equally):\n');
for i = 1:numel(lbl)
    fprintf('    %5g Hz cued %d time(s), delivered %d time(s)\n', lbl(i), ...
            sum(clean & T.cue_hz == lbl(i)), sum(clean & T.pred_hz == lbl(i)));
end
if numel(unique(arrayfun(@(f) sum(clean & T.cue_hz == f), lbl))) > 1
    fprintf(['    ^ unequal — if the design cues each frequency once per run, ' ...
             'a cue label was mistyped.\n']);
end

fprintf('\n──────── %s ────────\n', opt.Participant);
fprintf([' trials %d analysed (%d scrapped) | accuracy %.1f%% of labelled | ' ...
         'mean t_cmd %.2f s (SD %.2f) | mean exec %.1f s (SD %.1f)\n'], ...
        nClean, nScrap, 100*acc, R.summary.mean_t_cmd_s, R.summary.sd_t_cmd_s, ...
        R.summary.mean_exec_s, R.summary.sd_exec_s);
fprintf(' false results %d (FP %d / FN %d) | timeouts %d | no delivery %d\n', ...
        nFalse, nFP, nFN, nTO, nND);
fprintf(' summary -> %s\n figure  -> %s\n', matOut, figOut);
end

% ═════════════════════════════════════════════════════════════════════════════
% helpers
% ═════════════════════════════════════════════════════════════════════════════
function v = local_nums(s)
v = str2double(strsplit(strtrim(s), {',',' ',';'}));
v = v(~isnan(v));
end

function [mtype, minfo, mt] = local_markers(M, fs)
%LOCAL_MARKERS  Normalise the markers table into cellstr type/info + time (s).
mtype = local_cellstr(M.type);
if any(strcmp(M.Properties.VariableNames, 'info'))
    minfo = local_cellstr(M.info);
else
    minfo = repmat({''}, numel(mtype), 1);
end
if any(strcmp(M.Properties.VariableNames, 'sample'))
    mt = double(M.sample) / fs;                 % EEG clock: monotone, no NaNs
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

function [iStart, iEnd, nDerived] = local_segment(mtype)
%LOCAL_SEGMENT  Pair every trial_end with the marker that opened its trial.
%
%   trial_end is the only event guaranteed to exist for every trial, so it
%   drives the segmentation. The opening marker for each trial is, in order of
%   preference:
%     1. the last cue_onset / home_reached / trial_start lying between the
%        previous trial_end and this one   (cued runs, and uncued runs from
%        node builds that publish home_reached)
%     2. the previous trial_end            (uncued runs with no start marker:
%        the Arduino resumes flickering the moment the robot reports home,
%        which is that same event)
%     3. experiment_start, for trial 1     (the moment the block was launched)
%
%   nDerived counts the trials that fell back to 2 or 3, i.e. whose start time
%   is inferred rather than marked.
startNames = {'cue_onset', 'home_reached', 'trial_start'};
iS = find(ismember(mtype, startNames));
iE = find(strcmp(mtype, 'trial_end'));
iX = find(strcmp(mtype, 'experiment_start'), 1, 'first');
if isempty(iX), iX = 1; end

iStart = zeros(numel(iE),1); iEnd = iE(:); nDerived = 0;
for k = 1:numel(iE)
    if k == 1, lo = iX; else, lo = iE(k-1); end
    cand = iS(iS > lo & iS < iE(k));
    if isempty(cand)
        iStart(k) = lo;                 % inferred: previous end, or block start
        nDerived  = nDerived + 1;
    else
        iStart(k) = cand(end);          % marked
    end
end

keep = iStart < iEnd;
iStart = iStart(keep); iEnd = iEnd(keep);
end

function [dt, win, pass, pub] = local_decisions(info, t, tCue)
%LOCAL_DECISIONS  Parse 'win=..;max=..;mar=..;pass=..;pub=..;s=a,b,c,d'.
%   win is the winning frequency in Hz (0 = failed window), not a class index.
n = numel(info);
dt = t(:) - tCue; win = nan(n,1); pass = zeros(n,1); pub = zeros(n,1);
for i = 1:n
    s = info{i};
    v = regexp(s, 'win=([-\d\.]+)', 'tokens', 'once'); if ~isempty(v), win(i)  = str2double(v{1}); end
    v = regexp(s, 'pass=(\d)',      'tokens', 'once'); if ~isempty(v), pass(i) = str2double(v{1}); end
    v = regexp(s, 'pub=(\d)',       'tokens', 'once'); if ~isempty(v), pub(i)  = str2double(v{1}); end
end
end

function idx = local_firstrun(win, pass, V)
%LOCAL_FIRSTRUN  First index at which V consecutive identical passing wins occur.
idx = []; run = 0; last = NaN;
for i = 1:numel(win)
    if pass(i) == 1 && win(i) > 0 && win(i) == last, run = run + 1;
    elseif pass(i) == 1 && win(i) > 0,               run = 1; last = win(i);
    else,                                            run = 0; last = NaN;
    end
    if run >= V, idx = i; return; end
end
end

function local_boxgroup(ax, g, y, lbl, col)
%LOCAL_BOXGROUP  Box per frequency with jittered raw points and per-group n.
%   boxchart is used on a numeric x so the raw points can be overlaid; the
%   axis is then relabelled with the frequencies.
x = nan(size(g));
for i = 1:numel(lbl), x(g == lbl(i)) = i; end
ok = ~isnan(x) & ~isnan(y);
x = x(ok); y = y(ok);
if isempty(x), return; end
boxchart(ax, x, y, 'BoxFaceColor', col, 'MarkerStyle', 'none', 'BoxWidth', 0.55);
for i = 1:numel(lbl)                       % group mean, since the box shows median
    m = mean(y(x == i), 'omitnan');
    if ~isnan(m)
        plot(ax, i, m, 'd', 'MarkerSize', 8, 'MarkerFaceColor', col, ...
             'MarkerEdgeColor', 'k', 'LineWidth', 1);
    end
end
scatter(ax, x + 0.16*(rand(numel(x),1)-0.5), y, 22, 'filled', ...
        'MarkerFaceColor', [0.15 0.15 0.15], 'MarkerFaceAlpha', 0.45);
set(ax, 'XTick', 1:numel(lbl), 'XTickLabel', compose('%g', lbl(:)'));
xlim(ax, [0.4 numel(lbl)+0.6]);
yl = ylim(ax); ylim(ax, [yl(1) - 0.12*diff(yl), yl(2)]);
for i = 1:numel(lbl)
    n = sum(x == i);
    text(ax, i, min(ylim(ax)) + 0.03*diff(ylim(ax)), sprintf('n=%d', n), ...
         'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0.35 0.35 0.35]);
end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
