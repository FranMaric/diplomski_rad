# Build the Docker image (run once; re-run after editing resample_30hz.py)
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
docker build -t ros2-resample "$scriptDir"
if (-not $?) { Write-Error "Docker build failed"; exit 1 }

# Process every episode bag directory
$bagsDir = Join-Path $scriptDir "bags"
$episodes = Get-ChildItem $bagsDir -Directory | Sort-Object Name

foreach ($ep in $episodes) {
    $outputDir = Join-Path $bagsDir "$($ep.Name)_30hz"
    if (Test-Path $outputDir) {
        Write-Host "Skipping $($ep.Name) — output already exists"
        continue
    }

    Write-Host "`nProcessing $($ep.Name) ..."
    docker run --rm `
        -v "${bagsDir}:/bags" `
        ros2-resample `
        python3 /scripts/resample_30hz.py "/bags/$($ep.Name)" -o "/bags/$($ep.Name)_30hz"

    if (-not $?) { Write-Warning "Failed on $($ep.Name)" }
}

Write-Host "`nAll done."
