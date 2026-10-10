#!/usr/bin/env python3
"""Derive a separate disarmed height-test relay, preserving the XY relay."""
import ast
from pathlib import Path
import shutil
import sys
import tempfile


def create(directory):
    directory = Path(directory)
    source = directory / 'ground_xy_fusion_transport.py'
    target = directory / 'ground_height_fusion_transport.py'
    tree = ast.parse(source.read_text())
    matches = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Compare) and len(node.ops) == 1
                and isinstance(node.ops[0], ast.In)
                and isinstance(node.left, ast.Attribute)
                and node.left.attr == 'integer_value'):
            try:
                values = ast.literal_eval(node.comparators[0])
            except (ValueError, TypeError):
                continue
            if values == (0, 1, 9):
                matches.append(node)
    if len(matches) != 1:
        raise SystemExit('Expected one EV_CTRL 0/1/9 guard; no file written.')
    matches[0].comparators[0] = ast.parse('(0, 1, 9, 11)', mode='eval').body
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == 'GroundXYFusionTransport':
            node.name = 'GroundHeightFusionTransport'
        elif isinstance(node, ast.Name) and node.id == 'GroundXYFusionTransport':
            node.id = 'GroundHeightFusionTransport'
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            if node.value == 'ground_xy_fusion_transport':
                node.value = 'ground_height_fusion_transport'
            elif 'DISARMED GROUND' in node.value and 'EV_CTRL' in node.value:
                node.value = ('DISARMED GROUND XYZ/YAW TEST ONLY: EV_CTRL 0/1/9/11, '
                              'zero EV_POS and QMIN=0. Original stamps retained; '
                              'no flight/setpoint output.')
            else:
                node.value = (node.value.replace('{0,1,9}', '{0,1,9,11}')
                              .replace('Ground XY', 'Ground XYZ')
                              .replace('ground XY', 'ground XYZ'))
    text = '#!/usr/bin/env python3\n' + ast.unparse(ast.fix_missing_locations(tree)) + '\n'
    compile(text, str(target), 'exec')
    if target.exists() and target.read_text() != text:
        backup_dir = Path(tempfile.mkdtemp(prefix='ground_height_backup_', dir=directory))
        shutil.copy2(target, backup_dir / target.name)
        print('Previous target backup:', backup_dir / target.name)
    target.write_text(text)
    print('Created:', target)
    print('Original XY relay preserved:', source)
    return target


if __name__ == '__main__':
    create(sys.argv[1] if len(sys.argv) == 2 else Path.home() / 'Downloads')
