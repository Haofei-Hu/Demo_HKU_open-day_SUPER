#!/usr/bin/env python3
"""Copy the reported ground config, changing only convergence/logging settings.

No flight limits or optimizer-success claims are implied. Keep the aircraft
disarmed. Use the original config_name to restore the original configuration.
"""
from copy import deepcopy
from pathlib import Path
import shutil
import sys
import tempfile
import yaml


def create(source):
    source = Path(source)
    text = source.read_text()
    data = yaml.safe_load(text)
    if data['traj_opt']['exp_traj']['opt_accuracy'] != 5e-6:
        raise ValueError('Expected original exp_traj.opt_accuracy=5e-6; no changes made.')
    tree = yaml.compose(text)

    def scalar(keys):
        node = tree
        for key in keys:
            if not isinstance(node, yaml.MappingNode):
                raise ValueError('Unexpected YAML structure; no changes made.')
            matches = [value for name, value in node.value if name.value == key]
            if len(matches) != 1:
                raise ValueError('Missing/duplicate YAML key; no changes made.')
            node = matches[0]
        if not isinstance(node, yaml.ScalarNode):
            raise ValueError('Expected scalar YAML value; no changes made.')
        return node

    changes = [(('traj_opt', 'exp_traj', 'opt_accuracy'), '1.0e-4'),
               (('traj_opt', 'switch', 'save_log_en'), 'true')]
    edits = []
    for keys, value in changes:
        node = scalar(keys)
        edits.append((node.start_mark.index, node.end_mark.index, value))
    updated = text
    for start, end, value in sorted(edits, reverse=True):
        updated = updated[:start] + value + updated[end:]
    expected = deepcopy(data)
    expected['traj_opt']['exp_traj']['opt_accuracy'] = 1e-4
    expected['traj_opt']['switch']['save_log_en'] = True
    if yaml.safe_load(updated) != expected:
        raise ValueError('Unexpected configuration differences; no changes made.')

    target = source.with_name('mid360_ground_diagnostic_ros2.yaml')
    backup = Path(tempfile.mkdtemp(prefix='ground_diagnostic_backup_', dir=str(source.parent)))
    shutil.copy2(source, backup / source.name)
    if target.exists():
        shutil.copy2(target, backup / target.name)
    log_dir = source.parent.parent / 'log'
    for name in ('exp_opt_log.csv', 'exp_opt_penna.csv', 'back_opt_log.csv',
                 'back_opt_penna.csv', 'time_consuming.csv'):
        log = log_dir / name
        if log.exists():
            shutil.copy2(log, backup / name)
    target.write_text(updated)
    print('Created:', target)
    print('Backup:', backup)
    print('Changes: exp_traj.opt_accuracy=1e-4; save_log_en=true')
    return target, backup


if __name__ == '__main__':
    create(sys.argv[1] if len(sys.argv) > 1 else
           Path.home() / 'super_ws/src/SUPER/super_planner/config/mid360_real_ros2.yaml')
