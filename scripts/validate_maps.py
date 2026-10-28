#!/usr/bin/env python3
"""Validate PCD header, P5 PGM payload and YAML metadata; no ROS required."""
import argparse
from pathlib import Path
import math
import yaml


def validate(folder):
    pcd, pgm, metadata = (folder / ('map.' + ext) for ext in ('pcd', 'pgm', 'yaml'))
    for p in (pcd, pgm, metadata):
        if not p.is_file() or p.stat().st_size == 0:
            raise ValueError(f'文件不存在或为空: {p}')
    header = {}
    with pcd.open('rb') as f:
        for _ in range(100):
            line = f.readline(4096).decode('ascii').strip()
            if line and not line.startswith('#'):
                key, *values = line.split()
                header[key] = values
                if key == 'DATA':
                    break
        else:
            raise ValueError('PCD 头部不完整')
        offset = f.tell()
    fields = header['FIELDS']
    if not {'x', 'y', 'z'}.issubset(fields):
        raise ValueError('PCD 缺少 XYZ')
    points = int(header['POINTS'][0])
    if points <= 0 or int(header['WIDTH'][0]) * int(header['HEIGHT'][0]) != points:
        raise ValueError('PCD 点数无效或与宽高不一致')
    mode = header['DATA'][0]
    if mode == 'binary':
        sizes = list(map(int, header['SIZE']))
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        if len(sizes) != len(fields) or len(counts) != len(fields):
            raise ValueError('PCD 字段长度不一致')
        expected = points * sum(s * c for s, c in zip(sizes, counts))
        if pcd.stat().st_size - offset != expected:
            raise ValueError('PCD binary 数据长度与头部不一致')
    elif mode not in ('ascii', 'binary_compressed'):
        raise ValueError(f'未知 PCD 编码: {mode}')
    # PGM header token reader; preserve all pixel bytes after maxval delimiter.
    with pgm.open('rb') as f:
        def token():
            result = bytearray()
            while True:
                c = f.read(1)
                if not c:
                    raise ValueError('PGM 头部提前结束')
                if c == b'#':
                    f.readline()
                elif not c.isspace():
                    result.extend(c)
                    break
            while True:
                c = f.read(1)
                if not c or c.isspace():
                    if c == b'\r':
                        pos = f.tell()
                        if f.read(1) != b'\n':
                            f.seek(pos)
                    return bytes(result)
                result.extend(c)
        if token() != b'P5':
            raise ValueError('需要 P5 binary PGM')
        width, height, maxval = int(token()), int(token()), int(token())
        if width <= 0 or height <= 0 or maxval != 255:
            raise ValueError('PGM 尺寸/maxval 不支持')
        pixels = f.read()
    if len(pixels) != width * height:
        raise ValueError('PGM 像素长度与宽高不一致')
    config = yaml.safe_load(metadata.read_text())
    if (metadata.parent / config['image']).resolve() != pgm.resolve():
        raise ValueError('YAML image 没有指向本目录 map.pgm')
    resolution = float(config['resolution'])
    origin = config['origin']
    if not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3:
        raise ValueError('YAML 分辨率/原点无效')
    if not all(math.isfinite(float(x)) for x in origin):
        raise ValueError('YAML 原点含无效数值')
    if not 0 <= float(config['free_thresh']) < float(config['occupied_thresh']) <= 1:
        raise ValueError('YAML 占据阈值无效')
    histogram = {v: pixels.count(v) for v in set(pixels)}
    print(f'通过文件结构检查: {folder.resolve()}')
    print(f'PCD: {points:,} points, {mode}')
    print(f'PGM: {width} x {height}, resolution={resolution} m, origin={origin}')
    print(f'像素统计: {histogram}')
    if not histogram.get(0):
        raise ValueError('未发现黑色占据像素，请检查地图')
    print('此检查不证明地图精度、坐标对齐或导航可用性。')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    try:
        validate(args.folder)
    except (ValueError, KeyError, OSError, UnicodeError, TypeError) as error:
        parser.exit(1, f'验证失败: {error}\n')
