"""Run with the reviewed original IMU header as the single argument."""
import sys,tempfile,pathlib
from unittest.mock import patch
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts/maintenance'))
import apply_fastlio_init_guard as m
original=pathlib.Path(sys.argv[1]).read_bytes()
assert m.sha(m.render(original))==m.AFTER
assert m.render(m.render(original))==m.render(original)
try:m.render(original+b'\n')
except ValueError:pass
else:raise AssertionError('unreviewed accepted')
with tempfile.TemporaryDirectory() as td:
 p=pathlib.Path(td)/'IMU_Processing.hpp';p.write_bytes(original);back=pathlib.Path(td)/'backups'
 with patch.object(m,'mapping_running',return_value=True):
  try:m.apply(p,back)
  except RuntimeError:pass
  else:raise AssertionError('running accepted')
 assert p.read_bytes()==original and not back.exists()
 with patch.object(m,'mapping_running',return_value=False):b=m.apply(p,back)
 assert (b/p.name).read_bytes()==original and m.sha(p.read_bytes())==m.AFTER
 assert (b/'manifest.json').exists() and (b/'change.patch').exists()
 with patch.object(m,'mapping_running',return_value=False):assert m.apply(p,back) is None
 assert len(list(back.iterdir()))==1
 p.write_bytes(original)
 with patch.object(m,'mapping_running',side_effect=[False,True]):
  try:m.apply(p,back)
  except RuntimeError:pass
  else:raise AssertionError('race accepted')
 assert p.read_bytes()==original
 assert not list(p.parent.glob('.imu_guard_*'))
print('Patch checksum/refusal/running-process/backup/idempotency/race/cleanup checks: PASS')
