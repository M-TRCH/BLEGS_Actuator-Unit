# Build the submission .docx from the Markdown drafts with pandoc.
# Run from anywhere:  powershell -File paper/build.ps1
#
# What pandoc does NOT handle (finish in Word afterwards — see
# manuscript/submission/SUBMISSION_CHECKLIST.md):
#   - equation numbers flushed to the column's right edge
#   - the two-column body layout (front matter stays single-column,
#     so columns are applied per-section in Word, not in reference.docx)
#   - table captions above tables / figure captions below figures
$ErrorActionPreference = "Stop"
$paperDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Resolve pandoc: PATH first, then common install locations
$pandoc = $null
$cmd = Get-Command pandoc -ErrorAction SilentlyContinue
if ($cmd) { $pandoc = $cmd.Source }
if (-not $pandoc) {
    $candidates = @(
        "$env:LOCALAPPDATA\Pandoc\pandoc.exe",
        "$env:ProgramFiles\Pandoc\pandoc.exe",
        "$env:USERPROFILE\miniconda3\Library\bin\pandoc.exe",
        "$env:USERPROFILE\miniconda3\Scripts\pandoc.exe"
    )
    foreach ($c in $candidates) { if (Test-Path $c) { $pandoc = $c; break } }
}
if (-not $pandoc) {
    Write-Error "pandoc not found. Install with: winget install --id JohnMacFarlane.Pandoc"
}

$reference = Join-Path $paperDir "journal\templates\reference.docx"
if (-not (Test-Path $reference)) {
    Write-Error "reference.docx not found at $reference (see paper/journal/README.md)"
}

$drafts = @(
    "00_frontmatter.md", "01_introduction.md", "02_methods.md",
    "03_results.md", "04_discussion.md", "05_acknowledgement.md",
    "06_references.md"
) | ForEach-Object { Join-Path $paperDir "manuscript\drafts\$_" }

$out = Join-Path $paperDir "manuscript\submission\manuscript.docx"

# Warn about <mark> placeholders: pandoc drops raw HTML silently in docx
# output, so unresolved figure/value placeholders would vanish without trace.
$marks = Select-String -Path $drafts -Pattern "<mark>" -SimpleMatch
if ($marks) {
    Write-Warning ("{0} <mark> placeholders still present - they will NOT appear in the .docx:" -f $marks.Count)
    $marks | Group-Object Path | ForEach-Object { Write-Warning ("  {0}: {1}" -f (Split-Path -Leaf $_.Name), $_.Count) }
}

& $pandoc @drafts `
    --reference-doc=$reference `
    --resource-path=(Join-Path $paperDir "figures\export") `
    -o $out
if ($LASTEXITCODE -ne 0) { Write-Error "pandoc failed ($LASTEXITCODE)" }
Write-Host "Wrote $out"
