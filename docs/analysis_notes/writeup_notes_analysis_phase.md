# Write-up notes — online analysis phase

Everything from this analysis phase that should surface somewhere in the dissertation,
grouped by where it most likely belongs. Items marked **⚠** are things an examiner could
reasonably challenge if they are *not* disclosed.

---

## 1. Methods — discrete online experiment

**Timing base.** All trial timing derives from `markers.sample / fs`, the EEG sample
clock, not wall-clock timestamps. The `t_lsl` column is `NaN` throughout the recordings
and `t_unix` is subject to host-clock drift; the sample index is monotone and shares a
clock with the EEG itself. This is why times can be quoted to millisecond precision.

**Command detection.** A command is issued when four consecutive classifier windows agree
on the same class and both gates pass (`corr_thresh = 0.30`, `margin_thresh = 0.05`).
Windows advance every ~0.256 s. The recorded `pub=1` flag marks the published command.

> Worth stating explicitly: an offline replay of the recorded per-window score vectors,
> re-applying the same gate, reproduced the online command times **exactly** on a
> verification file. This confirms the logged decision stream is a faithful record of what
> the online system did, which is what licenses any later re-analysis from these files.

**Time to command** = `cue_onset` → first `pub=1`, minus the 1.5 s cue duration, making
the metric stimulus-onset-to-command. For uncued runs the subtraction is zero — there was
no cue to remove — so both conditions start at stimulus onset, but only the cued figure
involves a subtraction.

**Robot execution time** = command → `trial_end`, reported separately from BCI latency.
Keeping these apart matters: they are properties of different subsystems.

**⚠ Negative corrected times.** A command arriving less than 1.5 s after cue onset cannot
have been driven by the flicker. These are excluded from timing statistics and reported
separately as false positives (delivered class matched the cue) or false negatives (it did
not). Give the count; do not silently drop them.

**Mean vs median.** Means ± SD are reported throughout, on supervisory direction.
**⚠ Note in every box-plot caption that the centre line is the median while the diamond is
the mean** — they diverge visibly here because detection latencies are right-skewed (hard
floor at the window length, long tail to the timeout). `median` and `iqr` are retained in
every summary file, so both can be quoted: *"mean 2.94 ± 1.65 s (median 2.6 s)"* costs one
clause and is more honest for skewed data.

**ITR is not reported here.** Removed deliberately; the offline experiment provides it. If
ever reinstated, note that the online paradigm truncates the stimulus the moment a command
is accepted (the Arduino freezes the LEDs on `'p'`), so any threshold sweep is valid only
for settings *more permissive* than the online ones — stricter settings have no stimulus
left to fire on and would be censored, not slow.

---

## 2. Methods — continuous (screwdriver) experiment

**Direction mapping.** Class 0 = Red LED = 16 Hz = −1 = unscrew/left. Class 1 = Blue LED =
19 Hz = +1 = screw/right. Figures use red for left and green for right, aligning the plot
colour with the stimulus colour that drives it.

**⚠ Rotation is measured from the home joint angle**, `HOME_WAYPOINT[6] = 1.57 rad =
89.95°`, not from the first logged wrist sample. The logged `wrist_jog` value is the
*target after each step*, and the pre-jog angle is never logged, so `last − first` misses
the first step entirely. The home reference was verified empirically: the first logged
target sits at exactly `89.95 ± 14.35°` on every trial checked, matching one commanded
`JOG_STEP_RAD` (0.25 rad = 14.32°). Without this correction every net rotation is
understated by one full step.

**Trials are not single-direction.** `_start_jog` republishes `jog_start` on every restart
and every direction reversal. In one representative run only 3 of 10 trials were a single
uninterrupted jog; two reversed direction mid-trial. A per-trial direction label taken
from the first `jog_start` is therefore wrong for those trials.

**Grouping is by the cued sequence, not by observed motion.** `SEQUENCE[10] =
{0,1,0,1,0,1,1,0,1,0}` is hardcoded in the Arduino sketch — five left and five right per
run, so four runs give exactly 20 of each. **⚠ Grouping by what the robot actually did
would move failed trials into the other condition and make both look better than the
system performed.** Say that this was avoided.

**Sign convention.** Plotted value = `net_rotation × cue_direction`: the magnitude of
rotation, positive when the wrist turned as cued and negative when it turned the opposite
way. Zero means the wrist never moved. State this in the caption — it is not the obvious
convention, and a reader will otherwise assume left-negative/right-positive.

**Maximum reachable rotation = 200°.** Derivation, all of which should appear in the text:

| quantity | value | source |
|---|---|---|
| stimulus window | 5.000 s | `STIM_MS = 5000` |
| classifier period | 0.256 s (measured) | median inter-decision interval |
| windows per trial | 19–20 | 20 in 3/10 trials, 19 in 7/10 |
| votes to start jogging | 4 | `VOTES_START`; windows 1–3 do not step |
| maximum steps | 17 | 20 − 3 |
| first step | 14.32° | full `JOG_STEP_RAD`, arm stationary |
| subsequent steps | 11.55° (measured) | trajectory-overlap lag |
| **ceiling** | **14.32 + 16 × 11.55 = 199.1 ≈ 200°** | |

**⚠ Declare the ±6% slack in this denominator.** The nominal 20-window count assumes a
0.250 s period; the measured period is 0.256 s, giving 19.5 windows. Nineteen windows
would put the ceiling at 187.6°. Quote percentages to the nearest 5% and state the
assumption rather than implying exactness.

---

## 3. System findings worth reporting as results, not just caveats

**Trajectory-overlap lag costs ~19% of every commanded jog step.** `_step` computes
`target = measured_joint_angle + JOG_STEP_RAD`, but the previous 0.3 s trajectory has not
finished when the next window arrives 0.256 s later. The achieved step is therefore
**11.55° against a commanded 14.32°**. This is a real, quantified control-loop finding — a
consequence of commanding position increments faster than the trajectory executes — and it
is more interesting than a footnote. It also explains why no trial approaches the
theoretical ceiling.

**The hysteresis, not the participant, sets the ceiling.** The best-performing participant
reached 164.9° of a possible ~200° (83%) with no early jog termination; the shortfall is
almost entirely the four-window start-up cost. For weaker participants the dominant loss
is repeated stop/restart cycles under `VOTES_STOP`, leaving the wrist idle for much of the
5 s window.

**Left/right asymmetry.** Left-cued trials averaged ≈65% of maximum against ≈53% for
right-cued in the no-block condition — roughly a 12-point gap, about three times the size
of the block effect being tested, within the same participants. Since 16 Hz drives left
and 19 Hz drives right, this speaks directly to the frequency-selection claim.
**This paired comparison has not yet been tested and probably should be.**

---

## 4. ⚠ Data and logging limitations that must be disclosed

Instrumentation gaps, not analysis choices. Each was worked around; each should be stated
plainly, and each belongs in future work as a fix.

| # | Limitation | Consequence | Workaround used |
|---|---|---|---|
| 1 | **Cue class is not written to the recording.** `cue_onset` carries the experiment ID, not the class. | Ground truth cannot be recovered from the data alone. | Labels entered by hand from the fixed Arduino `SEQUENCE`; a duplicate-frequency guard and a session-level cue-balance check were added. |
| 2 | **`trial_end:'r'` is published for both success and scan failure.** | Tag-not-found trials unrecoverable from the `.mat`; the `TrialLogger` `.jsonl` that separates them was not run. | Entered by hand from run sheets. `trial_end:'t'` (classifier timeout) *is* recoverable. |
| 3 | **Uncued node publishes no start marker for trial 1** (`home_reached` opens only trials 2…N); some builds publish none at all. | Trial 1 was originally dropped; one run yielded 1 trial instead of 4. | Segmentation driven by `trial_end`; missing starts inferred from `experiment_start` and the previous `trial_end`. |
| 4 | **Trial 1 of an uncued run starts at `experiment_start`** — when the block was launched, not when the participant engaged. | That trial's time-to-command includes operator setup dead time and is not a true latency. | Flagged; isolate with `trial == 1 & condition == "uncued"` if reporting latencies. |
| 5 | **The uncued node advances on delivery, not on homing.** | A failed pick produces an extra trial: 5 trials for 4 delivered objects. Trial counts vary. | Trial count read from markers, never assumed; trials past the 4th auto-archived as recording-save padding. |
| 6 | **`wrist_net` is computed without unwrapping.** | A raw end-minus-start subtraction. On one run **8 of 10 trials were wrong and every `+1` trial had its sign inverted** — logging −222.9° for a true +127.1° rotation. | Recomputed from the unwrapped `wrist_jog` series. **Do not quote `wrist_net` anywhere.** |
| 7 | **A command can be published more than once per trial**, and the robot ignores later ones while busy. | Ambiguity over which command counts. | The first `pub=1` is used — both the classifier's decision and the one the arm acted on. |
| 8 | **One development session ran the classifier at 83 Hz instead of ~3.9 Hz.** | Entirely different control regime: the arm cannot finish a jog trajectory between windows, so rotation-per-window collapses and percentages are meaningless. | Excluded. A cadence check now reports any recording whose decision period deviates from the norm. |
| 9 | **Split recordings.** Two halves separated by real dead time. | Naive concatenation would shift every post-seam trial earlier by the length of the gap. | Merged with the true gap NaN-filled, recovered from `t0u`. **⚠ The merged file takes the later half's name, so its filename timestamp is not the recording start** — the true start is in `t0u`. |
| 10 | **Stale comments in the node source.** `JOG_STEP_RAD` is commented "1° per accepted detection" while the code sets 0.25 rad (14.32°); `JOINT7_MIN/MAX_RAD` are dead constants (no clamp is applied). | Misleads anyone reading the source. | Corrected in the write-up; fix in the repo before it is public. |

### A labelling error that was caught, and how

Worth a sentence in the data-quality section, because it shows the audit worked. One run
was initially scored 4/4 correct when it should have been 3/4. The cued-frequency tallies
across the session came out **9 / 8 / 7 / 8** instead of 8 / 8 / 8 / 8 — with each
frequency cued once per run they must be equal. The imbalance located a single trial where
the *delivered* sequence had been transcribed into the cue field, converting a genuine
23→16 Hz error into a spurious correct. The cue-balance report now runs automatically for
every participant.

---

## 5. Statistics

**Design.** Within-subject, n = 10, every participant measured in both conditions.
Comparisons are paired; each participant is their own control. This matters because
between-participant spread is large (right-cued rotation ranged from ~1.5% to ~80% of
maximum) and would swamp the effect in an unpaired test.

**Two averaging levels, both computed; state the distinction:**

- *Trial-level* — every trial weighted equally; describes the pooled sample.
- *Subject-level* — each participant reduced to one value first, then averaged. Every
  participant weighted equally and **n = 10**, the n any inferential test uses.
  **Report subject-level as the headline.**

**Result — effect of holding the block (subject-level, paired):**

| comparison | difference | p | Cohen's *dz* |
|---|---|---|---|
| all (primary) | −3.58 pp | 0.212 | −0.43 |
| left cued | −5.39 pp | 0.124 | −0.54 |
| right cued | −1.77 pp | 0.618 | −0.16 |

**⚠ This is a null result from an underpowered test and must be phrased as such.** With
n = 10 a paired test needs *dz* ≈ 0.72 to reach p < 0.05 and *dz* ≈ 1.0 for 80% power. The
observed *dz* = 0.43 would need roughly **40–45 participants** to detect reliably. "Not
significant" here means "not measurable with ten people", not "no effect"
([Amrhein, Greenland & McShane, *Nature* 2019](https://www.nature.com/articles/d41586-019-00857-9)).

Suggested phrasing:

> Holding the block reduced rotation toward the cued direction by 3.6 percentage points
> (95% CI …), *dz* = −0.43. With ten participants this study was powered to detect only
> differences of roughly 10 percentage points or larger, so the result should be read as
> inconclusive rather than as evidence of no effect.

**Report confidence intervals, not just p-values** — a CI gives both the estimate and the
range of effects the data cannot exclude
([Cumming, *Psychological Science* 2014](https://journals.sagepub.com/doi/10.1177/0956797613504966)).
**Cohen's *dz* is the correct effect size for paired data**; the independent-samples *d* is
not interchangeable
([Lakens, *Frontiers in Psychology* 2013](https://www.frontiersin.org/articles/10.3389/fpsyg.2013.00863/full)).

**Multiple comparisons.** Left, right and all are three tests on overlapping data. Declare
one primary comparison (all) and report the others as descriptive; otherwise apply the
Bonferroni-adjusted α = 0.0167.

**⚠ Do not claim equivalence.** A non-significant t-test does not show the two conditions
are the same. If the argument is that holding the block does *not* impair control, that
needs equivalence testing (TOST) and a stated smallest meaningful difference
([Lakens 2017](https://journals.sagepub.com/doi/10.1177/1948550617697177)).

**Consistency worth mentioning:** all three differences are negative and 8 of 10
participants moved in the same direction (sign test p ≈ 0.11). Suggestive, not significant,
and the three measures are not independent — but it means the point estimate is not driven
by a single outlier.

---

## 6. Reproducibility — analysis pipeline

Two independent pipelines, no shared state.

**Discrete (reach-and-retrieve)**

1. `merge_split_recording` — join a run split across two files (only when needed)
2. `analyze_discrete_online` — per participant; one figure + one summary `.mat`
3. `combine_discrete_online` — pools the ten summaries into the group figure

**Continuous (screwdriver)**

4. `plot_continuous_recording` — per-recording review figure, used for data cleaning
5. `plot_wrist_rotation_groups` — per participant; two-condition box plots + summary
6. `combine_wrist_rotation` — pools the participant summaries
7. `compare_wrist_conditions` — paired tests, effect sizes, power

Group-level scripts read only the per-participant summary files, never the raw recordings,
so all manual cleaning decisions are preserved and auditable.

**Data handling to state in the methods:** excluded trials are *archived, not deleted* —
they remain in every output table with a `scrapped` flag and a free-text reason, and are
excluded from statistics only. Counts and reasons are recoverable with
`R.trials(R.trials.scrapped, {'run','trial','scrap_reason'})`.

**⚠ One methodological caution to hold to:** exclusion criteria must be decidable from the
event itself, not from the outcome. A trial binned because something went wrong during the
run is legitimate; a trial binned because the classifier got it wrong is the failure being
measured.

---

## 7. Recommended repository fixes before it is made public

Not analysis, but they belong in future work — each is a one-to-few-line change that would
have removed a manual step or a source of error:

1. `unwrap()` the joint angle before computing `wrist_net` at logging time
2. Write the cued class into the `cue_onset` marker payload
3. Distinguish `trial_end` success from scan failure with different reason bytes
4. Publish a start marker for every trial in the uncued node, including the first
5. Count homing rather than delivery when advancing the uncued trial counter
6. Correct the stale `JOG_STEP_RAD` comment and remove the unused joint-7 clamp constants
7. Log the classifier decision period at session start so cadence changes are self-evident

---

## 8. Discussion points to carry forward from Method write-up

Captured while drafting Chapter 3, for use in Chapter 5 (Discussion). Not analysis to be
re-derived — just flags for where the Method chapter simplified something that has a more
interesting story worth telling once the numbers are in front of the reader.

**Wrist-wrap bug (Section 3.8, continuous task).** The Method chapter states plainly that
joint 7 wraps at ±180° and the raw rotation signal is unwrapped before computing net
rotation. What Method deliberately leaves out (kept there for reproducibility, not for
narrative): the *reason* this was necessary — the originally logged `wrist_net` marker's
sign and magnitude are unreliable on any trial that crosses the wrap boundary, which is a
genuine data-quality bug caught during analysis, not a design choice made up front. Worth
one paragraph in Discussion: what the symptom looked like when first spotted (implausible
net-rotation values / sign flips), how it was traced to the wrap, and the fix now listed as
repository item 1 above (`unwrap()` at logging time) so it doesn't recur for future users of
the codebase.

**Trajectory-overlap lag (~19% per step).** Quantified system finding from the continuous
task analysis — worth framing as a genuine contribution (a characterised cost of the control
loop design) rather than a limitation to apologise for. Tie it back to the 200° ceiling
derivation in Section 3.8/3.3: the ceiling assumes back-to-back steps with no overlap loss,
so the *achieved* maximum should be reported alongside the *theoretical* ceiling with this
lag factored in, and the gap between the two numbers is itself worth a sentence.

**Underpowered ANOVA (2×2 repeated measures, condition × direction, p = 0.21).** Must be
framed as inconclusive, not as evidence of no effect — n=10 is underpowered for the observed
effect size. State the achieved power or required n for the effect size seen, if computed
(see `compare_wrist_conditions`/`wrist_anova.m`), so the inconclusiveness claim is backed by
a number rather than asserted.

Specific interpretive points to make here (kept out of Results, which should report only the
F/df/p/effect-size numbers themselves):
- The direction effect's SD (27.63 pp) is nearly three times its mean (10.34 pp) — meaning
  participants disagree about the *sign* of the effect (some rotate further CW than CCW,
  others the reverse), not merely its size. No test could call an effect like that reliable.
- The condition effect sits at dz ≈ 0.43. With n=10, a paired comparison needs roughly
  dz ≈ 0.72 for p < 0.05 and dz ≈ 1.0 for 80% power — so this effect would need roughly
  40–45 participants to detect reliably. "Not significant" here means "not measurable with
  ten people," not "no effect exists" (Amrhein, Greenland & McShane, *Nature* 2019).
- Worth contrasting explicitly with the offline CCA-vs-FBCCA comparison (Section 4.1), which
  found a real, well-powered effect (dz ≈ 0.82) — a useful rhetorical pairing showing the
  write-up distinguishes a genuine null-supporting result from an underpowered one, rather
  than treating all non-significant results the same way.

**Left/right vs. CCW/CW asymmetry (65% vs. 53%, if still current).** Confirm against final
combined-group figures before writing this into Discussion — flagged here only as a
candidate, not confirmed accurate as of this note.

**Banded vs. fixed-reference offline/online SNR mismatch.** Method (3.3) already states that
banded CCA references correct for FastLED drift and shouldn't be compared directly to live
classifier performance, which uses fixed nominal references. Discussion is the place to
actually make that comparison explicit — i.e., show the offline banded-reference numbers
next to the online fixed-reference numbers and account for the gap, rather than leaving the
reader to infer it from the caveat in Method alone.

---

## 9. Report backlog — analysis to run before final submission

Items identified as worth doing but deliberately deferred, so they don't block the current
writing pass. Come back to these once 1.1 (Objectives) is drafted and Chapter 5 structure is
otherwise stable.

**Subject-level one-sample t-test, discrete online accuracy vs. chance (25%).** Tests whether
per-participant accuracy (10 participants) is significantly above the 4-class chance level.
Given the observed effect size is large (90% vs. 25% is a very large gap), n=10 is expected
to be more than sufficient — unlike the underpowered wrist-rotation condition effect
(dz≈0.43), this is not a borderline-power situation. Report as: one-sample t-test (or
Wilcoxon signed-rank against 25% if a non-parametric version is preferred), t/W statistic,
df, p-value. Add to Section 4.2 (Results) as a factual statistic; do not interpret there —
save interpretation (if any is needed at all, given how visually obvious the gap already is)
for 5.1.

**Sequencing note:** do this after 1.1 is finished, alongside the return pass to 5.1 that
was already planned once Objectives exist to reference explicitly.

**Citation decision: Amrhein, Greenland & McShane (2019) in 5.1's ANOVA paragraph.**
Currently kept (`\cite{amrhein_scientists_2019}`) next to the "non-significant does not mean
no effect, it is inconclusive" sentence. Not strictly required — this is a well-established
methodological point, and the citation could be dropped without weakening the claim much.
Revisit during the final trim pass: keep if the bibliography/word count has room, cut if
trimming for space. Not worth further discussion beyond this note.

**Add figures to Chapter 3 (Method).** Currently text-only in several places that would
benefit from a photo/screenshot: the stimulus LED arrays (discrete 4-group array and
continuous row), an AprilTag detection example (tag36h11, as seen by the wrist camera),
the EEG cap/electrode setup (actiCAP + ActiChamp), and one representative frame each from
the discrete reach-and-retrieve and continuous wrist-rotation experiments in progress. Keep
in mind the 20 figure/table cap for the whole report when deciding how many of these to
include and where.

**Trim-pass idea: replace scattered experiment-difference prose with a comparison table.**
Sections 3.7 (Experimental Protocol) describe differences between discrete-cued,
discrete-uncued, continuous non-bimanual, continuous bimanual, and continuous displacement
conditions across several separate paragraphs (trial count, cue duration, stimulus window
length, presence/absence of cue, timeout behaviour, direction convention). Consider
collapsing this into a single comparison table (conditions as columns, these attributes as
rows) during the final trim pass — likely reduces word count while improving scannability.
Check remaining figure/table budget (20 total) before committing.

---

## 10. Planning report assessor feedback — items to address in final report

From the planning report grading, supervised by Etienne Burdet. Two points are directly
actionable for the final report.

**Concrete non-muscular SRL use-case example (Chapter 1/2).** Assessor noted: "Examples of
possible use-case for non-muscular control of SRL would have strengthened the intro." The
current Objectives/Background text (reused from the planning report) states the *problem*
with EMG/foot-pedal/motor-imagery control abstractly but never gives a concrete example of
what a non-muscular SRL is actually used for. Easy fix now available: the bimanual
screwdriver task (holding a tool via SRL while both hands do something else) is a real,
already-demonstrated example — use it as the concrete use-case in 1.1 or 2.1 rather than
leaving the motivation abstract.

**System-level augmentation success criteria (Chapter 5, Limitations).** Assessor noted the
planning report evaluated BCI and robotic components individually but never defined
system-level metrics showing the technology provides *meaningful human augmentation* —
i.e., tasks that would be difficult or impossible without the SRL. This critique still
applies to the final report: all reported metrics (SSVEP accuracy, rotation %, displacement
rate) are component-level, not augmentation-level (e.g., no "with vs. without SRL" task
comparison exists). Add one sentence to 5.3 acknowledging this gap directly — better to name
it as a known limitation than have an examiner independently notice the same gap the
planning-report assessor already flagged.

**Not directly actionable, noted for awareness only:** figure label/text sizing was flagged
as too small in the planning report's figures — worth a general reminder to check font size
on any final figures before submission, no specific section to update.

---

## 11. Other items to check before submission

**Make own figure for the BCI paradigm comparison chart (Chapter 2, Section 2.3).** The
planning report's Figure 1 (SCP/ERD-ERS/P300/SSVEP comparison on training time vs. ITR)
was reused from a cited source \cite{amiri_review_2013}. Check whether directly reproducing
this figure counts as image plagiarism/copyright issue even with citation — safer to redraw
it as an original figure using the same underlying data/comparison, rather than copying the
source image directly. Also double check whether the ITR comparison table (SSVEP vs P300,
10 subjects) has the same concern, since it's reproduced from \cite{katyal_novel_2020}.

**Add more citations to Section 2.4 (CCA).** The CCA mechanism/limitations paragraph
(generic sine/cosine references, no personalised templates, accuracy drop below 1s windows,
harmonic overlap) currently carries little to no direct citation support of its own —
several of these claims (CCA's window-length sensitivity, personalised-template limitations,
harmonic overlap between lower and higher target frequencies) are standard findings in the
SSVEP-CCA literature and should be individually supported rather than left as unsupported
assertions carried over from the planning report.

**Add a brief note to Section 3.1 (Stimulus Design) on the scrapped diffuser idea.** A
diffuser was initially considered for participant comfort/photosensitivity safety but was
not pursued, because designing a diffuser for a 3x3 LED matrix per stimulus group turned out
to be more complex than initially thought. Diffuser-related pilot findings (CCA accuracy
~0.1 lower with diffused light) were cut from Chapter 2's Background section since the
diffuser was never part of the final design — but a one-clause mention in Method explaining
why no diffuser is used would preempt a reader wondering about this, especially since
photoepilepsy safety is otherwise handled via the participant exclusion criterion.
