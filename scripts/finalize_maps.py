#!/usr/bin/env python3
"""Validate and copy selected artifacts into maps/, refusing any overwrite."""
import argparse
from pathlib import Path
import shutil
from validate_maps import validate

parser = argparse.ArgumentParser()
parser.add_argument('source', type=Path)
parser.add_argument('target', type=Path)
args = parser.parse_args()
try:
    source, target = args.source.expanduser().resolve(), args.target.expanduser().resolve()
    if source == target:
        raise ValueError('源目录和目标目录必须不同')
    validate(source)
    paths = [(source / name, target / name) for name in ('map.pcd', 'map.pgm', 'map.yaml')]
    if any(dest.exists() or dest.is_symlink() for _, dest in paths):
        raise ValueError('目标已有地图，拒绝覆盖；请先自行确认现有成果')
    target.mkdir(parents=True, exist_ok=True)
    for src, dest in paths:
        with src.open('rb') as reader, dest.open('xb') as writer:
            shutil.copyfileobj(reader, writer)
    validate(target)
except (ValueError, KeyError, OSError, TypeError) as error:
    parser.exit(1, f'整理失败: {error}\n')
