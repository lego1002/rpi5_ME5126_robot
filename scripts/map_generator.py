#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generate a 480×180 ASCII PGM map with 60×60 grid cells, plus YAML.
"""

import os

def main():
    # output paths
    map_dir   = '/tmp/rpi5_auto_map'
    pgm_path  = os.path.join(map_dir, 'auto_map.pgm')
    yaml_path = os.path.join(map_dir, 'auto_map.yaml')

    # make dir
    os.makedirs(map_dir, exist_ok=True)

    # dimensions
    width, height = 480, 180

    # write PGM
    with open(pgm_path, 'w') as f:
        f.write('P2\n{} {}\n255\n'.format(width, height))
        for y in range(height):
            row = ['0' if (x % 60 == 0 or y % 60 == 0) else '255'
                   for x in range(width)]
            f.write(' '.join(row) + '\n')

    # write YAML
    content = (
        f"image: {pgm_path}\n"
        "resolution: 0.01\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    with open(yaml_path, 'w') as f:
        f.write(content)

    print("Generated:", pgm_path)
    print("Generated:", yaml_path)

if __name__ == '__main__':
    main()
