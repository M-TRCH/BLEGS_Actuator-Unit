# CLAUDE.md — paper/

Paper-layer guidance. Repo-wide architecture, build commands, and robot facts live in the **root [`CLAUDE.md`](../CLAUDE.md)** — read that first. This file only covers what is specific to the manuscript.

## What this is

The manuscript for a TCI Tier 1 journal paper (The Journal of KMUTNB) about the BLEGS quadruped and its data-driven leg-error compensation. Everything here is writing and submission material; the code that produced the data is `../python/paper_testing/` (test_2 = single-leg + ML models, test_3 = whole-robot walking) — **there is no code copy under paper/ by design**, so never "fix" analysis scripts here.

## Writing conventions (match the existing text exactly)

The manuscript is Thai academic prose; English in parentheses on first use of a technical term. Patterns used consistently in `manuscript/drafts/02_methods.md`:

- Paragraphs open with `&emsp;`.
- Headings use decimal numbering (`### 2.3.1`, `#### 2.3.3.1` where a 4th level was unavoidable).
- Display equations are `$$...$$`, introduced by `ตามสมการที่ (N)`; variables defined after in a `โดยที่:` bullet list.
- **Equation numbers are sequential and maintained by hand** — inserting one renumbers every later one plus its cross-references. Grep `สมการที่ (` and fix all. Currently 26 equations, all in 02_methods.md.
- Figure/table placeholders: `<mark>[แทรก รูปที่ N: caption]</mark>`; undecided values: `<mark>[รอกำหนด...]</mark>`. **pandoc silently drops `<mark>` (raw HTML) in docx output** — `build.ps1` warns about remaining ones.
- Captions must not use the word "แสดง" (journal rule).

**Chapter 2 is methods**: it defines procedures and metrics but must not report measured values or conclusions. Defer numbers with "รายงานในบทผลการทดลอง" / "นำเสนอในบทถัดไป" as §2.2.1 and §2.3.2 already do. Equipment specs (marker sizes, camera resolution) are fine in methods; results are not.

## Journal rules that shape the work

Full set: `journal/FORMAT_GUIDE.md` and the official docs in `journal/instructions/`. The ones that bite:

- **≤ 9 A4 pages total**, two columns, TH Sarabun New. Chapter 2 alone is long — check page count before writing chapters 1/3/4 at full length.
- **Every figure AND table must also be submitted as a separate .jpg** → `figures/export/`, `tables/export/`.
- Abstract ≤ 250 words, stand-alone, no citations; keywords must not repeat title words; conclusion 1–2 paragraphs, must not restate the abstract.
- References: IEEE, English only, Thai sources get `(in Thai)`; every entry cited in text.
- Double-blind review by 3 external reviewers — an anonymized copy may be needed.

## Build pipeline

`build.ps1` = pandoc over `manuscript/drafts/00_*.md … 06_*.md` with `journal/templates/reference.docx`. What pandoc cannot do (finish in Word): equation numbers flush right, per-section column layout, caption placement. reference.docx rules — incl. the Thai-font trap (`w:cs`/`w:szCs`) — are in `journal/README.md`.

## Measurement facts for §2.3 (verified from the four walk videos)

- Runway frame: X = 0–80 cm across (centreline 40), Y = 0–120 cm along; heading 90° = along +Y. Analysis: `../python/paper_testing/test_3/apriltag_runway_tilt_check.py` (writes `<video>_pose.csv`; videos on `D:\THESIS\walk_test\`).
- Robot back tags: **19.9 × 46.6 cm** tape-measured (`ROBOT_TAG_LOCAL_POINTS_CM`).
- **Parallax**: runway markers lie on the floor, robot tags ride ~31 cm above it → homography magnifies distances by m ≈ 1.21–1.22 (consistent across all four videos; camera ≈ 1.8 m). Divide distances by m; angles unaffected. §2.3.1 eq. (16)–(17). Raising runway markers to tag height removes the effect for future recordings.
- Robot walks ~2 m but the grid covers 120 cm → about half of each track is extrapolated (tag-rectangle self-check shows the mapping holds; report the proportion).
- §2.3.2 has **one recorded run per condition**; §2.1/§2.2.5 used 5 repeats.

## Known text-vs-code gaps still to resolve (details + file:line in `action-plan.md`)

1. §2.2.5 circle: text r=35 mm, analysis scripts use 30 mm and disagree on centre (−170 vs −175).
2. §2.2.3 counts: text 180 points/720 samples; files hold 152/608.
3. §2.2.1 undistortion: capture ran without it (stale calibration filename), contrary to the text.
4. §2.2.4 validation: `training_summary.txt` is in-sample; do NOT present it as model accuracy.
5. §2.1 moment arm: r=0.105 m described as axis→foot, but 105 mm is the crank length.
6. Physical runway has a duplicate ID-6 tag standing in for ID 7 (`--duplicate-id6-mode`).
