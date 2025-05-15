#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

def main():
    # 1) 准备文件夹
    map_dir = '/tmp/rpi5_auto_map'
    os.makedirs(map_dir, exist_ok=True)

    # 2) 生成 PGM（480×180 格，每 60 像素画一条格线）
    map_pgm = os.path.join(map_dir, 'auto_map.pgm')
    with open(map_pgm, 'w') as f:
        f.write('P2\n480 180\n255\n')
        for y in range(180):
            row = ['0' if (x % 60 == 0 or y % 60 == 0) else '255' for x in range(480)]
            f.write(' '.join(row) + '\n')

    # 3) 生成 YAML（注意：**行首绝对不要缩进**）
    map_yaml = os.path.join(map_dir, 'auto_map.yaml')
    yaml_content = f"""\
image: {map_pgm}
resolution: 0.01
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(map_yaml, 'w') as f:
        f.write(yaml_content)

if __name__ == '__main__':
    main()
