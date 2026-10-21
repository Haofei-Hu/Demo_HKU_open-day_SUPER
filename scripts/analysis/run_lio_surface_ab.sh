#!/usr/bin/env bash
# Ground-only experiment; run with bash after downloading the two Python tools
# beside this file. Existing flight workflow helpers remain in ~/Downloads.
set -e
tool_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/fastlio_ws/install/setup.bash"
source "$HOME/super_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

manager="$tool_dir/super_session.py"
observer="$tool_dir/hover_drift_check.py"
test -f "$manager"
test -f "$observer"
/usr/bin/python3 -c 'import yaml'
mkdir -p "$HOME/Downloads/lio_surface_ab"
run_dir=$(mktemp -d "$HOME/Downloads/lio_surface_ab/run_XXXXXXXX")
echo "REPORT DIRECTORY: $run_dir"
echo 'Keep aircraft physically stationary, disarmed, motors stopped throughout.'
echo 'This stops the managed vision/control stack, then runs two core-only trials.'
trap 'echo "Experiment interrupted/failed. Reports: $run_dir. Do not fly or resume vision from this incomplete experiment." >&2' ERR

# The existing manager requires fresh connected/disarmed/on-ground FC state
# before shutdown. It stops control/vision before LIO and MAVROS.
/usr/bin/python3 "$manager" stop
for size in 0.5 0.2; do
  if [ "$size" = 0.5 ]; then label=A_0p5; else label=B_0p2; fi
  echo "TRIAL: $label; restart with fixed extrinsics; warm up 20 seconds."
  /usr/bin/python3 "$manager" start --fixed-lio-extrinsics --lio-surf-size "$size"
  sleep 20
  timeout 30s ros2 param dump /laser_mapping > "$run_dir/${label}_params.yaml"
  /usr/bin/python3 - "$run_dir/${label}_params.yaml" "$size" <<'PY'
import sys, yaml
p=yaml.safe_load(open(sys.argv[1]))['/laser_mapping']['ros__parameters']
checks={
    'filter_size_surf': p['filter_size_surf']==float(sys.argv[2]),
    'filter_size_map': p['filter_size_map']==0.5,
    'point_filter_num': p['point_filter_num']==3,
    'fixed_extrinsics': p['mapping']['extrinsic_est_en'] is False,
    'live_clock': p['use_sim_time'] is False,
}
if not all(checks.values()):
    raise SystemExit('Unexpected live settings: '+str(checks))
print('Verified live surface size:', p['filter_size_surf'])
PY
  /usr/bin/python3 "$observer" --duration 60 --wait 30 --output "$run_dir/$label"
  /usr/bin/python3 "$manager" stop
done

# Restore the installed baseline with the previously used fixed-extrinsics
# override. Never restore the old alignment or restart vision/control.
/usr/bin/python3 "$manager" start --fixed-lio-extrinsics
sleep 10
timeout 30s ros2 param dump /laser_mapping > "$run_dir/restored_params.yaml"
/usr/bin/python3 - "$run_dir" <<'PY'
import json, sys, zipfile
from pathlib import Path
import yaml
root=Path(sys.argv[1]); params=[]
for label, expected in [('A_0p5',0.5),('B_0p2',0.2)]:
    p=yaml.safe_load((root/(label+'_params.yaml')).read_text())['/laser_mapping']['ros__parameters']
    if p.pop('filter_size_surf') != expected: raise SystemExit('Wrong trial setting')
    params.append(p)
    report=json.loads((root/label/'report.json').read_text())
    if report['result']!='CAPTURE_COMPLETE': raise SystemExit('Incomplete '+label)
if params[0]!=params[1]: raise SystemExit('Other live LIO parameters changed between A and B')
restored=yaml.safe_load((root/'restored_params.yaml').read_text())['/laser_mapping']['ros__parameters']
if restored.pop('filter_size_surf')!=0.5 or restored!=params[0]:
    raise SystemExit('Restored LIO parameters differ from baseline')
with zipfile.ZipFile(str(root)+'.zip','w',zipfile.ZIP_DEFLATED) as z:
    for label in ('A_0p5','B_0p2'):
        z.write(root/label/'diagnostic.zip',label+'/diagnostic.zip')
    for file in root.glob('*_params.yaml'):z.write(file,file.name)
print('BUNDLE:',str(root)+'.zip')
print('Baseline LIO settings verified. Only core processes are running.')
print('Vision/control remain stopped. Old alignment must not be reused after LIO restarts.')
PY
