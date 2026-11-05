#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Import an edited PNG/PGM as a delta; leave the original saved map untouched."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from mapping_rbnx.grid_edits import Store, map_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps-dir', required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--image', required=True)
    args = parser.parse_args()
    try:
        store = Store(map_directory(args.maps_dir, args.map_id))
        revision = store.revision()
        result = store.save(Path(args.image).read_bytes(), revision, store.fingerprint)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(1, f'import failed: {exc}\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
