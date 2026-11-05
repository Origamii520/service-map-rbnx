# SPDX-License-Identifier: MulanPSL-2.0
"""Saved occupancy edits. The SLAM graph and original grid are never rewritten.

A revision records absolute pixel overrides relative to a fingerprinted base,
not differences from the preceding revision. Exported images are derived caches.
"""
from __future__ import annotations

import contextlib
import fcntl
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import uuid

import numpy as np
from PIL import Image
import yaml

MAX_PIXELS = 16_000_000
MAX_BODY = 24_000_000


class Conflict(ValueError):
    """The base or active revision changed since the editor was opened."""


def _sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def map_directory(root, map_id):
    if not isinstance(map_id, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9._-]*', map_id):
        raise ValueError('invalid map_id')
    root = Path(root).resolve()
    path = (root / map_id).resolve()
    if path.parent != root:
        raise ValueError('map is outside the map directory')
    return path


def _image(data):
    with Image.open(io.BytesIO(data)) as im:
        if im.width * im.height > MAX_PIXELS:
            raise ValueError('map exceeds editor pixel limit')
        if im.mode != 'L':
            # Browser canvas exports opaque RGBA. Do not silently flatten masks.
            rgba = np.asarray(im.convert('RGBA'))
            if np.any(rgba[:, :, 3] != 255) or not np.array_equal(rgba[:, :, 0], rgba[:, :, 1]) or not np.array_equal(rgba[:, :, 1], rgba[:, :, 2]):
                raise ValueError('expected an opaque grayscale map')
        return np.array(im.convert('L'))


class Store:
    def __init__(self, map_dir):
        self.path = Path(map_dir).resolve()
        self.root = self.path / '.grid-edits'
        self.meta = yaml.safe_load((self.path / 'occupancy.yaml').read_text())
        image = (self.path / self.meta['image']).resolve()
        if image.parent != self.path:
            raise ValueError('occupancy image must be inside the saved map')
        self.base = _image(image.read_bytes())
        self.height, self.width = self.base.shape
        self.resolution = float(self.meta['resolution'])
        self.origin = [float(x) for x in self.meta['origin']]
        self.free = float(self.meta.get('free_thresh', .25))
        self.occupied = float(self.meta.get('occupied_thresh', .65))
        self.negate = int(self.meta.get('negate', 0))
        if (not math.isfinite(self.resolution) or self.resolution <= 0 or len(self.origin) != 3
                or not all(math.isfinite(x) for x in self.origin) or self.negate not in (0, 1)
                or not 0 <= self.free < self.occupied <= 1 or self.meta.get('mode', 'trinary') != 'trinary'):
            raise ValueError('editor requires a valid trinary occupancy map')
        description = json.dumps(self.meta, sort_keys=True).encode()
        self.fingerprint = hashlib.sha256(description + self.base.tobytes() + str(self.base.shape).encode()).hexdigest()

    def revision(self):
        try:
            revision = (self.root / 'active').read_text().strip()
        except FileNotFoundError:
            return 'original'
        if not re.fullmatch(r'[0-9a-f]{32}', revision):
            raise ValueError('invalid active edit revision')
        return revision

    def pixels(self, revision=None):
        revision = self.revision() if revision is None else revision
        if revision == 'original':
            return self.base.copy()
        if not re.fullmatch(r'[0-9a-f]{32}', revision):
            raise ValueError('invalid edit revision')
        data = json.loads((self.root / revision / 'delta.json').read_text())
        if data.get('schema') != 1 or data.get('base_sha256') != self.fingerprint:
            raise Conflict('original grid changed; edits need review against the new base')
        result = self.base.copy().reshape(-1)
        for index, value in data['cells']:
            if type(index) is not int or type(value) is not int or not 0 <= index < result.size or not 0 <= value <= 255:
                raise ValueError('invalid cell override')
            result[index] = value
        return result.reshape(self.base.shape)

    def metadata(self):
        revision = self.revision()
        pixels = self.pixels(revision)
        unknown = round(255 * ((self.free + self.occupied) / 2 if self.negate else 1 - (self.free + self.occupied) / 2))
        palette = {'wall': 255 if self.negate else 0, 'free': 0 if self.negate else 255, 'unknown': unknown}
        return dict(map_id=self.path.name, width=self.width, height=self.height,
                    resolution=self.resolution, origin=self.origin, version=revision,
                    base_sha256=self.fingerprint, changed_cells=int(np.count_nonzero(pixels != self.base)),
                    palette=palette)

    @contextlib.contextmanager
    def locked(self):
        self.root.mkdir(exist_ok=True)
        with (self.root / 'lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def save(self, png, expected, base_sha256, *, restore=False):
        pixels = self.base.copy() if restore else _image(png)
        if pixels.shape != self.base.shape:
            raise ValueError('map dimensions must remain unchanged')
        with self.locked():
            # Re-read the base under the lock as well as checking the caller's token.
            fresh = Store(self.path)
            if base_sha256 != fresh.fingerprint or self.fingerprint != fresh.fingerprint or expected != self.revision():
                raise Conflict('map changed; reload before saving (unsaved drawing is retained)')
            revision = uuid.uuid4().hex
            stage = self.root / ('.' + revision)
            stage.mkdir()
            indices = np.flatnonzero(pixels.reshape(-1) != self.base.reshape(-1))
            delta = dict(schema=1, base_sha256=self.fingerprint,
                         cells=[[int(i), int(pixels.reshape(-1)[i])] for i in indices])
            try:
                (stage / 'delta.json').write_text(json.dumps(delta, separators=(',', ':')))
                Image.fromarray(pixels).save(stage / 'occupancy.pgm')
                Image.fromarray(pixels).save(stage / 'occupancy.png')
                meta = dict(self.meta, image='occupancy.pgm')
                (stage / 'occupancy.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
                for artifact in stage.iterdir():
                    with artifact.open('rb') as stream:
                        os.fsync(stream.fileno())
                _sync_directory(stage)
                stage.rename(self.root / revision)
                _sync_directory(self.root)
                temp = self.root / ('.active-' + revision)
                with temp.open('w') as stream:
                    stream.write(revision)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp, self.root / 'active')
                _sync_directory(self.root)
            finally:
                if stage.exists():
                    shutil.rmtree(stage)
            return dict(ok=True, version=revision, changed_cells=len(indices))

    def export(self, kind, revision=None):
        pixels = self.pixels(revision)
        if kind == 'yaml':
            return yaml.safe_dump(dict(self.meta, image='occupancy.pgm'), sort_keys=False).encode()
        output = io.BytesIO()
        Image.fromarray(pixels).save(output, format='PNG' if kind == 'png' else 'PPM')
        return output.getvalue()

    def occupancy(self, revision=None):
        pixels = self.pixels(revision).astype(float) / 255
        probability = pixels if self.negate else 1 - pixels
        grid = np.full(pixels.shape, -1, dtype=np.int8)
        grid[probability > self.occupied] = 100
        grid[probability < self.free] = 0
        return np.flipud(grid).reshape(-1).tolist()


def effective_directory(map_dir):
    """Resolve only occupancy inputs; callers must keep the original graph path."""
    path = Path(map_dir)
    if not (path / '.grid-edits' / 'active').exists():
        return str(path)
    store = Store(path)
    revision = store.revision()
    # Validate the delta and the derived cache before giving it to map_server.
    pixels = store.pixels(revision)
    directory = store.root / revision
    if not np.array_equal(_image((directory / 'occupancy.pgm').read_bytes()), pixels):
        raise ValueError('edited occupancy cache does not match its delta')
    expected = dict(store.meta, image='occupancy.pgm')
    if yaml.safe_load((directory / 'occupancy.yaml').read_text()) != expected:
        raise ValueError('edited occupancy metadata does not match the base')
    return str(directory)
