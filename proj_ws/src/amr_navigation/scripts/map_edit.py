#!/usr/bin/env python3
"""Hand-edit a saved map in an ordinary paint program (Pinta, GIMP on a PC).

GIMP's file plug-ins crash on this Jetson (GLib pthread_setspecific abort),
and Pinta cannot write PGM, so editing goes through PNG:

  map_edit.py to-png maps/map_20261019_143205.yaml
      -> maps/map_20261019_143205_edit.png   (edit this one)

  Paint with the Pencil tool, antialiasing OFF:
      black  #000000  obstacle / keep-out
      white  #FEFEFE  free
      grey   #CDCDCD  unknown
  Do NOT crop, resize or rotate - the yaml origin would no longer match.

  map_edit.py to-pgm maps/map_20261019_143205.yaml maps/map_20261019_143205_edit.png
      -> maps/map_20261019_143205_e1.pgm + .yaml (+ _stations.yaml copy)

to-pgm never overwrites: it writes a new map name (_e1, _e2, ...), so the
original stays usable and is easy to go back to. Every pixel is snapped to
0 / 205 / 254 with the yaml's own occupied/free thresholds (what map_server
would decide anyway), so antialiased edges cannot leave stray grey values.
Refuses if the image size differs from the original. Prints how many cells
changed, and lists the obstacle -> free areas, which must be checked on site.
"""
import argparse
import os
import shutil
import sys

import numpy as np
import yaml
from PIL import Image

OCC, UNK, FREE = 0, 205, 254


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    return meta, np.array(Image.open(img_path).convert('L'))


def snap(a, meta):
    # map_server (negate 0): p_occ = (255 - v) / 255
    p = (255.0 - a.astype(np.float32)) / 255.0
    out = np.full(a.shape, UNK, np.uint8)
    out[p > meta.get('occupied_thresh', 0.65)] = OCC
    out[p < meta.get('free_thresh', 0.25)] = FREE
    return out


def cmd_to_png(args):
    meta, a = load_map(args.yaml)
    out = args.out or os.path.splitext(args.yaml)[0] + '_edit.png'
    if os.path.exists(out) and not args.force:
        sys.exit(f'{out} exists (edits in progress?) - use --force to overwrite')
    Image.fromarray(snap(a, meta)).save(out)
    print(f'wrote {out}  ({a.shape[1]} x {a.shape[0]} px) - edit it, then run to-pgm')


def next_name(base_dir, stem):
    i = 1
    while os.path.exists(os.path.join(base_dir, f'{stem}_e{i}.yaml')):
        i += 1
    return f'{stem}_e{i}'


def regions(mask, res, origin):
    """Bounding boxes (world coords) of 8-connected blobs in mask
    (plain flood fill - scipy is not installed on the Jetson)."""
    h, w = mask.shape
    seen = np.zeros_like(mask, bool)
    out = []
    for r, c in zip(*np.nonzero(mask)):
        if seen[r, c]:
            continue
        seen[r, c] = True
        stack, cnt = [(r, c)], 0
        r0 = r1 = r
        c0 = c1 = c
        while stack:
            y, x = stack.pop()
            cnt += 1
            r0, r1, c0, c1 = min(r0, y), max(r1, y), min(c0, x), max(c1, x)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and not seen[yy, xx]:
                        seen[yy, xx] = True
                        stack.append((yy, xx))
        # image row 0 = top = max y
        x0, x1 = origin[0] + c0 * res, origin[0] + (c1 + 1) * res
        y0, y1 = origin[1] + (h - r1 - 1) * res, origin[1] + (h - r0) * res
        out.append((cnt, x0, y0, x1, y1))
    return sorted(out, reverse=True)


def cmd_to_pgm(args):
    meta, orig = load_map(args.yaml)
    src = os.path.dirname(os.path.abspath(args.yaml))
    stem = os.path.splitext(os.path.basename(args.yaml))[0]

    edit = Image.open(args.png)
    if edit.mode in ('RGBA', 'LA', 'P'):
        edit = edit.convert('RGBA')
        bg = Image.new('RGBA', edit.size, (UNK, UNK, UNK, 255))
        edit = Image.alpha_composite(bg, edit)  # transparent = unknown
    a = np.array(edit.convert('L'))
    if a.shape != orig.shape:
        sys.exit(f'size changed: original {orig.shape[1]} x {orig.shape[0]}, '
                 f'edited {a.shape[1]} x {a.shape[0]} px - cropped/resized? '
                 'Not saved (origin would be wrong).')
    new = snap(a, meta)
    old = snap(orig, meta)

    res = meta['resolution']
    cell = res * res
    occ_to_free = (old == OCC) & (new == FREE)
    print('changed cells (m^2):')
    for name, m in (('obstacle -> free', occ_to_free),
                    ('obstacle -> unknown', (old == OCC) & (new == UNK)),
                    ('free/unknown -> obstacle', (old != OCC) & (new == OCC)),
                    ('unknown -> free', (old == UNK) & (new == FREE)),
                    ('free -> unknown', (old == FREE) & (new == UNK))):
        n = int(m.sum())
        if n:
            print(f'  {name:26s} {n:7d} cells  {n * cell:7.2f} m^2')
    if ((old == OCC) & (new == UNK)).sum() > 50:
        print('WARNING: many obstacle cells became unknown - walls thinned? '
              '(antialiasing/blur left on, or a soft brush used)')
    if not (old != new).any():
        sys.exit('no changes - nothing written')
    if occ_to_free.any():
        blobs = regions(occ_to_free, res, meta['origin'])
        print('obstacle -> free areas (largest first) - CHECK ON SITE that these are really gone:')
        for cnt, x0, y0, x1, y1 in blobs[:15]:
            print(f'  {cnt:6d} cells  x {x0:6.2f}..{x1:6.2f}  y {y0:6.2f}..{y1:6.2f} m')
        if len(blobs) > 15:
            print(f'  ... {len(blobs) - 15} more small areas')

    if args.dry_run:
        print('dry run - nothing written')
        return
    name = args.name or next_name(src, stem)
    for ext in ('.yaml', '.pgm'):
        if os.path.exists(os.path.join(src, name + ext)):
            sys.exit(f'{name}{ext} already exists - pick another --name')
    Image.fromarray(new).save(os.path.join(src, name + '.pgm'))
    meta_out = dict(meta, image=name + '.pgm')
    with open(os.path.join(src, name + '.yaml'), 'w') as f:
        yaml.safe_dump(meta_out, f, default_flow_style=None, sort_keys=False)
    print(f'wrote {name}.pgm / {name}.yaml in {src}')
    st = os.path.join(src, stem + '_stations.yaml')
    if os.path.exists(st):
        shutil.copy(st, os.path.join(src, name + '_stations.yaml'))
        print(f'copied stations -> {name}_stations.yaml')
    print('Test-drive every route on the new map before line use.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('to-png', help='map yaml -> PNG to edit')
    p.add_argument('yaml')
    p.add_argument('-o', '--out')
    p.add_argument('--force', action='store_true', help='overwrite an existing _edit.png')
    p.set_defaults(fn=cmd_to_png)
    p = sub.add_parser('to-pgm', help='edited PNG -> new map (never overwrites)')
    p.add_argument('yaml', help='original map yaml (size / thresholds / origin)')
    p.add_argument('png')
    p.add_argument('--name', help='new map name (default <map>_eN)')
    p.add_argument('-n', '--dry-run', action='store_true', help='report only')
    p.set_defaults(fn=cmd_to_pgm)
    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
