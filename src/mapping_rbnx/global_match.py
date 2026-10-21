# SPDX-License-Identifier: MulanPSL-2.0
"""Find where a robot is on a saved map from one stationary laser scan.

A particle filter cannot do this. Its update is driven by motion -- with the
robot still, the cloud never resamples and the estimate never sharpens -- so
relocalizing with it means driving the robot first, which on a real site means
asking someone to push it around before the map is usable.

Matching a scan against the map directly has no such requirement. The scan is
scored at every pose the map allows, coarsely and then finely, and the answer is
accepted only when one pose is clearly better than every other pose that is not
next to it. When two distant poses score alike the room genuinely looks the same
from both, and the honest response is to say so and ask for a nudge -- that is
the only case where the robot has to move at all.
"""
from __future__ import annotations

import math
from typing import Optional

# Coarse pass: every 0.2 m and 10 degrees. Fine pass: 0.1 m and 2 degrees around
# the best coarse candidates. Beams are subsampled because a 640-beam scan says
# little more about a pose than 90 well-spread beams do, and costs seven times
# as much to score.
COARSE_STEP_M = 0.2
COARSE_YAW_STEP_RAD = math.radians(10.0)
FINE_STEP_M = 0.1
FINE_YAW_STEP_RAD = math.radians(2.0)
FINE_CANDIDATES = 24
TOP_K = 5
BEAMS = 90
# A rival pose this far away is a different place, not the same answer nudged.
DISTINCT_M = 0.8
# ...and so is one standing in the same spot facing somewhere else. Comparing
# rivals on position alone let a pose 90 degrees from the truth win with a
# 28-point margin and no doubt reported: the pose that WAS right sat at the same
# coordinates and was dismissed as "the same answer".
DISTINCT_RAD = math.radians(25.0)
# How far ahead of its nearest distinct rival the winner has to be.
MARGIN = 0.06
# Width of the likelihood field. A beam landing this far from the nearest mapped
# wall is worth about 60% of one landing on it.
SIGMA_M = 0.10
COARSE_SIGMA_M = 0.25
# What a beam ending in ground the map never observed is worth. Not zero -- the
# map's silence is not evidence against a pose -- and not one, or a pose facing
# unmapped space would score as well as one facing a wall it matches.
UNKNOWN_WEIGHT = 0.35


class Candidate:
    """One global-localization hypothesis."""

    def __init__(self, pose, score: float) -> None:
        self.pose = pose
        self.score = score


class Match:
    """A scored pose. `ok` is the caller's verdict, not the matcher's."""

    def __init__(self, pose, score: float, runner_up: float, detail: str,
                 candidates=None) -> None:
        self.pose = pose
        self.score = score
        self.runner_up = runner_up
        self.detail = detail
        self.candidates = candidates or [Candidate(pose, score)]

def _scan_beams(msg, limit: int = BEAMS):
    """(ranges, angles) of up to `limit` valid beams, evenly spread."""
    import numpy as np

    ranges = np.asarray(msg.ranges, dtype=np.float32)
    angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
    good = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
    ranges, angles = ranges[good], angles[good]
    if len(ranges) > limit:
        idx = np.linspace(0, len(ranges) - 1, limit).astype(np.int64)
        ranges, angles = ranges[idx], angles[idx]
    return ranges, angles


def distance_field(occ, resolution: float):
    """Metres from each cell to the nearest occupied cell (chamfer, two passes).

    A binary "did the beam land within N cells of a wall" test cannot rank
    poses: in a furnished room most poses put most beams near something, and
    every candidate scored a flat 100%. Distance is what separates a pose whose
    beams sit ON the walls from one whose beams merely sit near them.
    """
    import numpy as np

    h, w = occ.shape
    big = float(h + w)
    d = np.where(occ, 0.0, big).astype(np.float32)
    diag = float(np.sqrt(2.0))
    for r in range(h):
        row = d[r]
        if r > 0:
            prev = d[r - 1]
            row = np.minimum(row, prev + 1.0)
            row = np.minimum(row, np.concatenate(([big], prev[:-1])) + diag)
            row = np.minimum(row, np.concatenate((prev[1:], [big])) + diag)
        for c in range(1, w):
            v = row[c - 1] + 1.0
            if v < row[c]:
                row[c] = v
        d[r] = row
    for r in range(h - 1, -1, -1):
        row = d[r]
        if r < h - 1:
            nxt = d[r + 1]
            row = np.minimum(row, nxt + 1.0)
            row = np.minimum(row, np.concatenate(([big], nxt[:-1])) + diag)
            row = np.minimum(row, np.concatenate((nxt[1:], [big])) + diag)
        for c in range(w - 2, -1, -1):
            v = row[c + 1] + 1.0
            if v < row[c]:
                row[c] = v
        d[r] = row
    return d * resolution


def _score(grid, field, known, ranges, angles, xs, ys, yaws,
           sensor_xy=(0.0, 0.0), sigma_m: float = SIGMA_M):
    """Likelihood of the scan at every (x, y, yaw) given, in [0, 1].

    (x, y, yaw) is the ROBOT's pose and `sensor_xy` is where the laser is
    mounted in the robot's frame, so the beams are cast from the laser, not
    from the base. Casting from the base searched for where the LASER was and
    returned it as where the ROBOT was: every fix came out short by the mount
    offset, 0.202 m forward on this robot, and the overlay drew the scan that
    far off the wall.

    Each beam contributes exp(-d^2 / 2 sigma^2) for the distance from where it
    ended to the nearest mapped wall; a beam ending where the map never looked
    contributes a fixed middling amount, because the map's silence is neither
    evidence for the pose nor against it. Beams that leave the map entirely
    contribute nothing.
    """
    import numpy as np

    sx, sy = sensor_xy
    cyaw, syaw = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
    lx = xs[:, None] + sx * cyaw - sy * syaw
    ly = ys[:, None] + sx * syaw + sy * cyaw
    cos = np.cos(yaws[:, None] + angles[None, :])
    sin = np.sin(yaws[:, None] + angles[None, :])
    ex = lx + ranges[None, :] * cos
    ey = ly + ranges[None, :] * sin
    # Truncating a negative fraction toward zero silently maps a beam just
    # beyond the lower/left boundary onto the first cell.
    col = np.floor((ex - grid.origin[0]) / grid.resolution).astype(np.int32)
    row = np.floor((ey - grid.origin[1]) / grid.resolution).astype(np.int32)
    inside = (col >= 0) & (col < grid.width) & (row >= 0) & (row < grid.height)
    col = np.clip(col, 0, grid.width - 1)
    row = np.clip(row, 0, grid.height - 1)
    d = field[row, col]
    like = np.exp(-(d * d) / (2.0 * sigma_m * sigma_m))
    seen = known[row, col]
    like = np.where(seen, like, UNKNOWN_WEIGHT)
    like = np.where(inside, like, 0.0)
    return like.mean(axis=1), inside.sum(axis=1)

def _yaw_distance(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

def _top_distinct_candidates(refined, limit: int):
    """Return distinct hypotheses without changing the search itself."""
    kept = []

    for sc, x, y, yaw in refined:
        duplicate = False

        for candidate in kept:
            px, py, pyaw = candidate.pose

            close_pos = math.hypot(x - px, y - py) < DISTINCT_M
            close_yaw = _yaw_distance(yaw, pyaw) < DISTINCT_RAD

            if close_pos and close_yaw:
                duplicate = True
                break

        if duplicate:
            continue

        normalized_yaw = math.atan2(
            math.sin(yaw),
            math.cos(yaw),
        )

        kept.append(
            Candidate(
                (x, y, normalized_yaw),
                sc,
            )
        )

        if len(kept) >= limit:
            break

    return kept


def _allowed_positions(grid, standable, xs, ys):
    """Keep the coarse search's existing domain during refinement as well."""
    import numpy as np

    col = (np.asarray(xs, dtype=np.float64) - grid.origin[0]) / grid.resolution
    row = (np.asarray(ys, dtype=np.float64) - grid.origin[1]) / grid.resolution
    inside = (np.isfinite(col) & np.isfinite(row) &
              (col >= 0) & (col < grid.width) & (row >= 0) & (row < grid.height))
    indices = np.flatnonzero(inside)
    allowed = np.zeros(xs.shape, dtype=bool)
    allowed[indices] = standable[np.floor(row[indices]).astype(np.intp),
                                 np.floor(col[indices]).astype(np.intp)]
    return xs[allowed], ys[allowed]


def global_scan_match(grid, msg, tolerance_cells: int = 3,
                     sensor_xy: tuple = (0.0, 0.0)) -> Optional[Match]:
    """Best pose for `msg` on `grid`, or None when the scan says nothing.

    `grid` is `localizers._Grid`; the search runs over its free cells, since a
    robot cannot be standing in a wall or somewhere the map never saw.
    """
    try:
        import numpy as np
        from numpy.lib.stride_tricks import sliding_window_view
    except Exception:  # noqa: BLE001
        return None
    if grid is None or msg is None:
        return None

    occ = np.frombuffer(bytes(grid.cells), dtype=np.uint8).reshape(grid.height, grid.width).astype(bool)
    known = np.frombuffer(bytes(grid.known), dtype=np.uint8).reshape(grid.height, grid.width).astype(bool)
    field = distance_field(occ, grid.resolution)
    k = 2 * tolerance_cells + 1
    near = sliding_window_view(np.pad(occ, tolerance_cells), (k, k)).any(axis=(2, 3))

    ranges, angles = _scan_beams(msg)
    if len(ranges) < 20:
        return None

    # Candidate positions: free cells on a coarse lattice, with room for the
    # robot's own footprint so it is not placed against a wall it would be
    # touching.
    step = max(1, int(round(COARSE_STEP_M / grid.resolution)))
    room = sliding_window_view(np.pad(~near, 1), (3, 3)).all(axis=(2, 3))
    standable = known & ~near & room
    rows, cols = np.nonzero(standable[::step, ::step])
    if not len(rows):
        return None
    px = grid.origin[0] + (cols * step + 0.5) * grid.resolution
    py = grid.origin[1] + (rows * step + 0.5) * grid.resolution
    yaws = np.arange(-math.pi, math.pi, COARSE_YAW_STEP_RAD, dtype=np.float32)

    best = []
    for yaw in yaws:
        s, _ = _score(grid, field, known, ranges, angles,
                      px.astype(np.float32), py.astype(np.float32),
                      np.full(len(px), yaw, dtype=np.float32), sensor_xy,
                      sigma_m=COARSE_SIGMA_M)
        for i in np.argsort(s)[-FINE_CANDIDATES:]:
            best.append((float(s[i]), float(px[i]), float(py[i]), float(yaw)))
    best.sort(reverse=True)
    best = best[:FINE_CANDIDATES]

    # Fine pass around each surviving candidate.
    offs = np.arange(-COARSE_STEP_M, COARSE_STEP_M + 1e-6, FINE_STEP_M, dtype=np.float32)
    dyaws = np.arange(-COARSE_YAW_STEP_RAD, COARSE_YAW_STEP_RAD + 1e-6,
                      FINE_YAW_STEP_RAD, dtype=np.float32)
    refined = []
    for _, cx, cy, cyaw in best:
        gx, gy = np.meshgrid(offs + cx, offs + cy)
        gx, gy = gx.ravel(), gy.ravel()
        gx, gy = _allowed_positions(grid, standable, gx, gy)
        if not len(gx):
            continue
        for dy in dyaws:
            s, _ = _score(grid, field, known, ranges, angles, gx, gy,
                          np.full(len(gx), cyaw + dy, dtype=np.float32), sensor_xy)
            i = int(np.argmax(s))
            refined.append((float(s[i]), float(gx[i]), float(gy[i]), float(cyaw + dy)))
    refined.sort(reverse=True)
    if not refined:
        return None

    # Pick the best distinct fine hypotheses first. Unlike the original
    # matcher, polish every Top-K hypothesis instead of polishing only the
    # current winner.
    fine_candidates = _top_distinct_candidates(refined, TOP_K)

    # One more pass at a quarter of the fine step. The fine grid alone leaves a
    # residual of its own size, and this is the cheapest place to spend it.
    polish = np.arange(-FINE_STEP_M, FINE_STEP_M + 1e-6, FINE_STEP_M / 4.0, dtype=np.float32)
    pyaws = np.arange(-FINE_YAW_STEP_RAD, FINE_YAW_STEP_RAD + 1e-6,
                      FINE_YAW_STEP_RAD / 4.0, dtype=np.float32)

    polished = []

    for candidate in fine_candidates:
        px0, py0, pyaw0 = candidate.pose
        best_local = (candidate.score, px0, py0, pyaw0)

        gx, gy = np.meshgrid(polish + px0, polish + py0)
        gx, gy = gx.ravel(), gy.ravel()
        gx, gy = _allowed_positions(grid, standable, gx, gy)

        for dy in pyaws:
            if not len(gx):
                break
            sc, _ = _score(grid, field, known, ranges, angles, gx, gy,
                           np.full(len(gx), pyaw0 + dy, dtype=np.float32), sensor_xy)
            i = int(np.argmax(sc))

            local = (
                float(sc[i]),
                float(gx[i]),
                float(gy[i]),
                float(pyaw0 + dy),
            )

            if local[0] > best_local[0]:
                best_local = local

        polished.append(best_local)

    # Keep the original fine hypotheses in the pool as well. This matters if
    # two polished hypotheses converge to the same basin: another distinct fine
    # hypothesis can still fill the remaining Top-K slot.
    final_pool = refined + polished
    final_pool.sort(reverse=True)

    candidates = _top_distinct_candidates(final_pool, TOP_K)

    top = candidates[0]

    rival = candidates[1].score if len(candidates) > 1 else 0.0
    rival_where = ""

    if len(candidates) > 1:
        rx, ry, ryaw = candidates[1].pose
        rival_where = (" at (%.1f, %.1f, %.0f deg)"
                       % (rx, ry, math.degrees(ryaw)))

    return Match(top.pose, top.score, rival,
                 f"{top.score:.0%} likelihood over {len(ranges)} beams, "
                 f"best rival{rival_where} {rival:.0%}",
                 candidates=candidates)
