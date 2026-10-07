#!/usr/bin/env python3
"""Install a newly measured session quaternion in the ground preview only."""
import argparse
import ast
import math
from pathlib import Path
import shutil
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('quaternion', nargs=4, type=float, metavar='Q')
    parser.add_argument('--preview', type=Path,
                        default=Path.home() / 'Downloads/aligned_pose_preview.py')
    args = parser.parse_args()
    norm = math.sqrt(sum(value * value for value in args.quaternion))
    if not all(math.isfinite(value) for value in args.quaternion) or abs(norm - 1) > .02:
        parser.error('Expected a finite, approximately unit quaternion in xyzw order.')
    text = args.preview.read_text()
    assignments = [node for node in ast.parse(text).body
                   if isinstance(node, ast.Assign) and any(
                       isinstance(target, ast.Name) and target.id == 'Q_WC'
                       for target in node.targets)]
    if len(assignments) != 1:
        parser.error('Expected exactly one top-level Q_WC assignment; no file written.')
    assignment = assignments[0]
    lines = text.splitlines(keepends=True)
    q = [value / norm for value in args.quaternion]
    lines[assignment.lineno - 1:assignment.end_lineno] = [f'Q_WC = np.array({q!r})\n']
    updated = ''.join(lines)
    compile(updated, str(args.preview), 'exec')
    backup = Path(tempfile.mkdtemp(prefix='preview_alignment_backup_',
                                  dir=str(args.preview.parent))) / args.preview.name
    shutil.copy2(args.preview, backup)
    args.preview.write_text(updated)
    print('Updated ground-preview session quaternion xyzw:', q)
    print('Backup:', backup)
    print('Restart the preview and ground relay. This does not calibrate the floor or venue.')


if __name__ == '__main__':
    main()
