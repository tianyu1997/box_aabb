# Build Chinese T-RO paper (xelatex + xeCJK)
param([switch]$Clean)

$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot

if ($Clean) {
    Remove-Item -Force *.aux, *.bbl, *.blg, *.log, *.out, *.pdf, *.toc, *.lof, *.lot -ErrorAction SilentlyContinue
    Write-Host "Cleaned."
    Pop-Location; return
}

xelatex -interaction=nonstopmode root_cn.tex
bibtex root_cn
xelatex -interaction=nonstopmode root_cn.tex
xelatex -interaction=nonstopmode root_cn.tex

if (Test-Path root_cn.pdf) {
    Write-Host "Build OK -> root_cn.pdf"
} else {
    Write-Error "Build failed — see root_cn.log"
}
Pop-Location
