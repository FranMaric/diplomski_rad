# Extract Z-force contact readings from all episode_N_metadata bags.
#
# Reads /optoforce_0 WrenchStamped and /metadata, writes one CSV per episode
# (contact frames only) into dataset_recorder/contact_forces/.
#
# Usage:
#   .\run_extract_contact_forces.ps1              # episode_N_metadata bags
#   .\run_extract_contact_forces.ps1 -ThirtyHz    # episode_N_30hz_metadata bags

param(
    [switch]$ThirtyHz   # target 30 Hz metadata bags instead of full-rate ones
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$bagsDir   = Join-Path $scriptDir "bags"

docker build -t ros2-metadata "$scriptDir"
if (-not $?) { Write-Error "Docker build failed"; exit 1 }

$namePattern = if ($ThirtyHz) { '^episode_\d+_30hz_metadata$' } else { '^episode_\d+_metadata$' }
$episodes = Get-ChildItem $bagsDir -Directory |
    Where-Object { $_.Name -match $namePattern } |
    Sort-Object Name

if ($episodes.Count -eq 0) {
    Write-Warning "No matching episode directories found in $bagsDir"
    exit 0
}

foreach ($ep in $episodes) {
    $epName = $ep.Name
    Write-Host "`nExtracting contact forces for $epName ..."
    docker run --rm `
        -v "${bagsDir}:/bags" `
        -v "${scriptDir}:/dataset_recorder" `
        ros2-metadata `
        python3 /scripts/extract_contact_forces.py "/bags/$epName" `
            -o /dataset_recorder/contact_forces
    if (-not $?) { Write-Warning "Failed on $epName" }
}

Write-Host "`nDone. CSVs written to $scriptDir\contact_forces\"
