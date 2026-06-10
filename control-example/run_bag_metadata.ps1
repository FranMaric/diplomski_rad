$ErrorActionPreference = "Stop"

$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$arg1 = if ($args.Count -gt 0) { $args[0] } else { "" }

docker run --rm `
  -v "E:\diplomski_rad\control-example\bags:/bags" `
  -v "${SCRIPT_DIR}\bag_metadata.py:/bag_metadata.py" `
  control-example:latest `
  /bin/bash -c "source /opt/ros/noetic/setup.bash && python3 /bag_metadata.py $arg1"
