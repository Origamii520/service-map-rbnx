# Occupancy grid editing

Open **Maps → Edit grid** on the Mapping page (port 8091 by default). The
editor is also available at `/editor?map_id=<saved-map-id>`. It uses the
service's `MAPPING_MAPS_DIR`; no robot-specific paths, extra web server,
Scene database or Go2 package are required.

Paint obstacles, erase false obstacles to free space, mark unknown cells, draw
straight walls or filled rectangles, and undo/redo before saving. Brush size is
in metres. PGM and YAML exports refer to the same selected revision. Restore
creates an empty delta against the original while retaining previous revisions.
An outdated browser receives a conflict rather than overwriting newer work;
its unsaved drawing remains visible until the operator reloads or discards it.

## What each consumer uses

Automatic application is integrated with the **slam_toolbox** backend:

| Consumer | Input |
| --- | --- |
| Initial/global candidate search | Original saved occupancy + selected user delta |
| Optional AMCL initialization | The same revision, pinned for that initialization run |
| Initial candidate verification | The same edited occupancy |
| Continuous localization after handover | Original slam_toolbox pose graph and scans |
| Verification after handover / continuous localization trust | Original occupancy corresponding to the pose graph |
| Mapping Web UI and navigation occupancy capability | Effective edited occupancy during saved-map localization; live raw map during mapping |

The graph directory passed to slam_toolbox is unchanged. The editor never
rewrites `posegraph.posegraph`, `posegraph.data`, original occupancy files or
map-frame identity. No AMCL tracking mode, motion command, relocalization request,
watchdog policy or additional motion gate is introduced by saving an edit.
Saving during initial localization takes effect on the **next** initialization;
the current run finishes with the revision it started with.

The occupancy capability now resolves to `/robonix/map/navigation_grid` for
slam_toolbox. `/map` remains owned by SLAM / its initialization map server.
Consumers resolving `robonix/service/map/occupancy_grid` through Atlas pick up
the endpoint on restart/rebind. Deployments that explicitly override a map topic
must update that override to `/robonix/map/navigation_grid`, including Nav2's
static layer and any Scene/map displays. Do not point them at the raw `/map`
when edited navigation is intended. The relay runs with the existing Mapping
supervisor even if the web page is closed or the HTTP UI is disabled.

Other backends can edit and export saved grids, but the page explicitly labels
these **export only**; their running navigation and localization are not changed.
The storage/editor is robot independent. Automatic application currently follows
the slam_toolbox initialization/handover lifecycle, not every SLAM backend.

## Persistence

```text
<map-id>/
  occupancy.{pgm,png,yaml}       # original saved map, unchanged
  posegraph.{posegraph,data}    # original SLAM artifacts, unchanged
  .grid-edits/
    active                     # atomically replaced revision identifier
    lock                       # serializes writers across processes
    <revision>/
      delta.json               # authoritative sparse overrides against base
      occupancy.{pgm,png,yaml}  # derived, self-contained export/map_server input
```

`delta.json` schema 1 contains `base_sha256` and `cells: [[index, value], ...]`.
Indices address row-major image pixels **from the top left**, unlike the ROS
OccupancyGrid data array which starts at the bottom left. Values are grayscale
bytes. Every revision is relative to the immutable original, not chained to
another edit. Unpainted grayscale values are preserved exactly. The base hash
covers pixels, shape and YAML metadata, including origin, resolution, thresholds
and negate. A changed base cannot silently inherit edits from an older map.

Exports are prepared and synced before the revision and active pointer are
published. Reads use a complete revision. The initializer checks derived caches
against the delta before passing them to map_server. Back up the original map
and `.grid-edits` together. Do not copy only the active pointer.

To import an existing full edited PNG/PGM without modifying the original:

```bash
python3 scripts/import_grid_edit.py --maps-dir /path/to/maps \
  --map-id lab --image /path/to/edited/occupancy.pgm
```

The import prints the new revision and changed-cell count. It rejects dimension
changes and concurrent edits. Before changing deployments, verify the exported
image and map geometry match the intended version.

## Scope and verification

This editor preserves dimensions, resolution and origin (including yaw), and
supports opaque trinary grayscale maps with either negate value. The existing
coarse/fine global matcher assumes axis-aligned grids; this PR does not add
rotated-grid localization. Export/ROS publishing preserve the saved yaw.
No new Python dependency is needed beyond the existing Pillow, NumPy and
PyYAML runtime.

Removing a false obstacle can reopen a route. It cannot repair a warped pose
graph, odometry drift, wrong extrinsics or real obstacles detected by the live
sensors. Checkpoints, local SLAM rebuilding and Explore integration are separate
work. This change does not claim physical navigation acceptance.

Tests cover sparse reconstruction/restart, original artifact preservation,
revision conflicts, concurrent saves, failed activation, base changes, negate,
export geometry, HTTP routes, initial-map pinning and graph handover. The relay
can be exercised on an isolated localhost ROS domain using synthetic occupancy
messages; no chassis or navigation commands are required.


## Deployment size limits

Configure these positive integers in the mapping service's deployment `config`
block. Omitting them keeps the existing defaults; zero, negative, fractional,
boolean and string values are rejected at initialization.

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    config:
      algo: slam_toolbox
      sensor_providers: {scan_converter: pcld2lscan}
      editor_max_pixels: 16000000
      editor_max_body_bytes: 24000000
```

| Setting | Default | Unit and scope |
|---|---:|---|
| `editor_max_pixels` | 16,000,000 | Decoded width × height in pixels/cells; checked before image decoding, for original and uploaded images and edited-cache validation. |
| `editor_max_body_bytes` | 24,000,000 | Bytes of the complete save/restore JSON body, including base64 PNG, revision and base hash; excludes HTTP headers. Oversized requests receive HTTP 413 before reading the body. |

The upload must fit **both** limits. Pixels are not bytes: PNG size depends on
compression, image content and color mode. If the PNG contains `B` bytes, its
base64 field needs `4 × ceil(B / 3)` bytes, plus the data-URL prefix (if present)
and the other JSON fields. Size `editor_max_body_bytes` for that complete body,
with margin. There is no fixed body-to-pixel ratio: a larger pixel allowance
alone does not guarantee that every image of that size can be uploaded.
The default body allowance is 24 decimal MB (about 22.9 MiB).

For example, a deployment allowing 32,000,000 pixels can choose a 192,000,000-byte
body budget when its available memory and expected PNGs justify it. This is a
configuration example, not a memory reservation or a promise that processing
uses only that much RAM. Decoding, RGBA validation, NumPy arrays, base64/JSON and
sparse edits can coexist in memory; allow additional RAM. Pillow's own image
safety checks also remain active. Configure limits before loading a large map;
reducing the pixel limit below a saved map's dimensions makes that map unreadable
through the editor/edited-grid path until the allowance is raised again.

The standalone import script is not a running deployment and retains the
16,000,000-pixel default. The HTTP body limit does not apply to its local file
input.
