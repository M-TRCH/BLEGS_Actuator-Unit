# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The manuscript repository for a **TCI Tier 1 journal paper** (KMUTNB International Journal of Applied Science and Technology) about the BLEGS quadruped: a four-legged robot with 2-DOF separated five-bar legs, BLDC actuators, and a data-driven model that compensates the legs' non-linear mechanical error.

This repo holds **writing and analysis only**. The robot firmware and control code live in a separate repo, `../BLEGS_Actuator-Unit` (branch `migrate-python-files`), which has its own CLAUDE.md — read it for anything about the robot itself.

## Layout

- `01_Manuscript/drafts/` — the manuscript, one Markdown file per chapter. **Only `02_methods.md` has content**; introduction, results and discussion are empty files.
- `01_Manuscript/FORMAT_GUIDE.md` — the journal's formatting rules (TH Sarabun New, 2 columns, 8–10 A4 pages, IEEE references). Consult before making layout decisions; it is a summary of the journal's official template.
- `04_Code/Test_2-3/` — **copies** of two scripts from the code repo. They have already diverged from their sources; treat the code repo as canonical and re-copy rather than editing here.
- `02_Figures/`, `03_Data/`, `05_References/` — empty placeholders.

Experiment media (videos, ~500 MB each) live outside the repo on `D:\THESIS\...` and are not tracked.

## Writing conventions (follow the existing text)

The manuscript is **Thai academic prose**; English appears in parentheses on first use of a technical term. Match these patterns exactly — they are consistent throughout `02_methods.md`:

- Paragraphs open with `&emsp;` for the indent.
- Headings use decimal numbering, max 3 levels per the format guide (`### 2.3.1`, then `#### 2.3.3.1` where a fourth level was unavoidable).
- Display equations are `$$...$$` on their own line, introduced by `ตามสมการที่ (N)`.
- **Equation numbers are sequential and maintained by hand.** Inserting an equation renumbers every later one plus its cross-references — grep `สมการที่ (` and fix all of them. There are currently 26.
- Variables are defined after the equation as a `โดยที่:` bullet list.
- Figures are placeholders: `<mark>[แทรก รูปที่ N: caption]</mark>`. Figure and table captions must not use the word "แสดง" (journal rule).
- Values still to be decided are marked with `<mark>[รอกำหนด...]</mark>`.

**Chapter 2 is methods.** It states what was done and how quantities are defined. It must not report measured values or draw conclusions — where a number comes out of the work, defer it with "รายงานในบทผลการทดลอง" or "นำเสนอในบทถัดไป", as §2.2.1 and §2.3.2 already do. Equipment specifications (marker sizes, camera resolution) are not results and belong in methods.

## The three experiments

- **§2.1 Torque-to-weight** — force gauge on the leg tip at stall, 5 repeats.
- **§2.2 Foot-position accuracy and ML compensation** — ArUco ground truth over a workspace grid at four payloads, five models (MLP/RF/SVR/POLY3/POLY4) predicting the position error from (θ_A, θ_B, I_A, I_B), then a circle-trajectory replay with compensation on/off. Corresponds to `python/paper_testing/test_2/` in the code repo.
- **§2.3 Whole-robot walking** — §2.3.1 sets up the vision measurement, §2.3.2 compares compensation on/off × payload 0/3 kg on a straight open-loop walk, §2.3.3 (designed, **not yet run**) characterises terrain capability under teleoperation with speed and cost of transport. Corresponds to `python/paper_testing/test_3/`.

## Measurement facts for §2.3 (verified from the videos)

- Runway world frame: **X = 0–80 cm across, Y = 0–120 cm along**, origin at marker ID 4; centreline is X = 40. Heading 90° means the body points along +Y.
- Robot back tags measure **19.9 cm across × 46.6 cm along** (tape measured). The tracking code assumed 20 × 43 until this was corrected — check `ROBOT_TAG_LOCAL_POINTS_CM` if numbers look off.
- **The runway markers sit on the floor but the robot tags ride ~31 cm above it**, so the ground-plane homography magnifies every measured distance by m ≈ 1.21–1.22 (consistent across all four videos to within 0.006; implies a camera height near 1.8 m). Distances must be divided by m; angles need no correction. §2.3.1 equations (16)–(17) document this. Raising the runway markers to tag height would remove the effect for future recordings.
- The robot walks about 2 m but the marker grid covers only 120 cm, so **roughly half of each track is extrapolated** beyond the calibrated area. The tag-rectangle self-check shows the mapping stays consistent there, but the proportion should be reported.
- §2.3.2 has **one recorded run per condition**, not the 5 repeats used in §2.1 and §2.2.5.

## Running the analysis

The vision scripts need OpenCV, which the Python on PATH does not have. Use the conda interpreter:

```bash
"$USERPROFILE/miniconda3/python.exe" apriltag_runway_tilt_check.py --video "D:/THESIS/walk_test/walk.MOV" --frame-step 3 --no-display --no-video
```

Run it from the **code repo** (`../BLEGS_Actuator-Unit/python/paper_testing/test_3/`), not from the stale copy in `04_Code/`. It writes `<video>_pose.csv` next to the video and prints the deviation summary plus the parallax scale check. Full-resolution 4K detection takes a couple of minutes per clip; `--no-video` skips re-encoding the annotated output.

## Known gaps between the manuscript and the code that produced the data

Each of these was verified against the actual files and still needs a decision. They matter because they change numbers that would be published.

- **§2.2.5 circle radius**: the text specifies r = 35 mm, but `vision_based_trajectory_eval.py` and `calibration_coverage_check.py` both use 30 mm, and they disagree with each other on the centre (−170 vs −175 mm). Radial error is computed against these constants, so a 5 mm error dominates the ~1 mm effect being reported.
- **§2.2.3 sample count**: the text says a 180-point grid and 720 samples; `workspace_grid.csv` holds 152 points and `grid_log.csv` 608 rows.
- **§2.2.1 lens distortion**: `single_leg_xy_control.py` loads `calibration_olympus25mm.npz`, which does not exist — the file `camera_calibration.py` produces is `output/params/calibration_olympus25f1.2.npz`. The loader fails silently, so the capture ran without undistortion, contrary to the text.
- **§2.2.4 validation**: the text describes training on static data and evaluating on dynamic motion, but `train_compensation_model.py` fits and predicts on the same set, so `training_summary.txt` reports in-sample error. Those figures must not be presented as model accuracy, and picking a model from them favours the most overfit one.
- **§2.1 moment arm**: r = 0.105 m is described as the distance from the joint axis to the leg tip, but 105 mm is the motor crank length (L_AC); the tip is further out (L_CE = 145 mm beyond the elbow). This feeds the headline torque-to-weight figure.
- The physical runway has a **misprinted duplicate ID-6 tag** standing in for ID 7; the tracker works around it with `--duplicate-id6-mode`. Either reprint before collecting final data or disclose it.
