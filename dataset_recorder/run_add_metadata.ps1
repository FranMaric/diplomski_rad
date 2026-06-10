# Add a latched /metadata topic to every episode bag.
#
# Phase 1 – generate states files (run first, no flags):
#   .\run_add_metadata.ps1
#   .\run_add_metadata.ps1 -ThirtyHz
#
# Phase 2 – write bags after editing the states files:
#   .\run_add_metadata.ps1 -Write
#   .\run_add_metadata.ps1 -ThirtyHz -Write
#
# Output bags land in bags/<episode>_metadata/ next to the originals.
# A states_report.txt is written to this dataset_recorder folder after -Write.

param(
    [switch]$Write,
    [switch]$ThirtyHz   # target episode_N_30hz bags instead of episode_N bags
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$bagsDir   = Join-Path $scriptDir "bags"

# Rebuild image (picks up any script changes)
docker build -t ros2-metadata "$scriptDir"
if (-not $?) { Write-Error "Docker build failed"; exit 1 }

# Match episode directories based on source flag
$namePattern = if ($ThirtyHz) { '^episode_\d+_30hz$' } else { '^episode_\d+$' }
$episodes = Get-ChildItem $bagsDir -Directory |
    Where-Object { $_.Name -match $namePattern } |
    Sort-Object Name

if ($episodes.Count -eq 0) {
    Write-Warning "No matching episode directories found in $bagsDir"
    exit 0
}

# Clear previous report and stats JSON before the write loop so they accumulate fresh
if ($Write) {
    $reportHost = Join-Path $scriptDir "states_report.txt"
    $statsHost  = Join-Path $scriptDir "states_stats.json"
    if (Test-Path $reportHost) { Remove-Item $reportHost }
    if (Test-Path $statsHost)  { Remove-Item $statsHost  }
}

foreach ($ep in $episodes) {
    $epName = $ep.Name

    if ($Write) {
        Write-Host "`nWriting metadata for $epName ..."
        docker run --rm `
            -v "${bagsDir}:/bags" `
            -v "${scriptDir}:/dataset_recorder" `
            ros2-metadata `
            python3 /scripts/add_metadata_topic.py "/bags/$epName" --write `
                -o "/bags/${epName}_metadata" `
                --report /dataset_recorder/states_report.txt
        if (-not $?) { Write-Warning "Failed on $epName" }
    } else {
        Write-Host "`nInit states file for $epName ..."
        docker run --rm `
            -v "${bagsDir}:/bags" `
            ros2-metadata `
            python3 /scripts/add_metadata_topic.py "/bags/$epName"
        if (-not $?) { Write-Warning "Failed on $epName" }
    }
}

if ($Write) {
    Write-Host "`nGenerating summary ..."
    docker run --rm `
        -v "${scriptDir}:/dataset_recorder" `
        ros2-metadata `
        python3 /scripts/add_metadata_topic.py --summarize --report /dataset_recorder/states_report.txt

    Write-Host "`nAll done. Bags written to bags/<episode>_metadata/"
    Write-Host "State duration report: $reportHost"
} else {
    Write-Host "`nAll states files created. Edit them, then re-run with -Write."
}
