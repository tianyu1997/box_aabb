# Build English T-RO paper (pdflatex)
param([switch]$Clean)

$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot

if ($Clean) {
    Remove-Item -Force *.aux, *.bbl, *.blg, *.log, *.out, *.pdf, *.toc, *.lof, *.lot -ErrorAction SilentlyContinue
    Write-Host "Cleaned."
    Pop-Location; return
}

pdflatex -interaction=nonstopmode root.tex
bibtex root
pdflatex -interaction=nonstopmode root.tex
pdflatex -interaction=nonstopmode root.tex

if (Test-Path root.pdf) {
    Write-Host "Build OK -> root.pdf"
} else {
    Write-Error "Build failed — see root.log"
}
Pop-Location
