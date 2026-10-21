# SPDX-License-Identifier: MulanPSL-2.0
"""Real scan matching must retain the coarse search's map-domain rules."""
import math
from types import SimpleNamespace
import unittest

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from mapping_rbnx import global_match, localizers


def open_room(truth=(-.05, 1.7), unknown_columns=0):
    """Three walls, open left boundary; forward laser can see mapped walls."""
    occupied = np.zeros((100, 80), dtype=np.uint8)
    occupied[0, :] = occupied[-1, :] = 1
    occupied[:, -1] = 1
    known = np.ones_like(occupied)
    known[:, :unknown_columns] = 0
    grid = localizers._Grid(80, 100, bytearray(occupied.ravel()),
                           bytearray(known.ravel()), .05, (0., 0.))
    x, y = truth
    angles = np.linspace(-1.4, 1.4, 91)
    ranges = []
    for angle in angles:
        dx, dy = math.cos(angle), math.sin(angle)
        distance = (3.975-x)/dx
        if dy > 0:
            distance = min(distance, (4.975-y)/dy)
        if dy < 0:
            distance = min(distance, (.025-y)/dy)
        ranges.append(distance)
    scan = SimpleNamespace(ranges=ranges, angle_min=float(angles[0]),
                           angle_increment=float(angles[1]-angles[0]),
                           range_min=.03, range_max=15.)
    return grid, scan, occupied, known


class CandidateValidityTests(unittest.TestCase):
    def test_fine_search_cannot_place_a_candidate_outside_the_map(self):
        grid, scan, _, _ = open_room()
        match = global_match.global_scan_match(grid, scan)
        self.assertIsNotNone(match)
        for candidate in match.candidates:
            x, y, _ = candidate.pose
            self.assertGreaterEqual(x, 0., candidate.pose)
            self.assertGreaterEqual(y, 0., candidate.pose)
            self.assertLess(x, grid.width*grid.resolution, candidate.pose)
            self.assertLess(y, grid.height*grid.resolution, candidate.pose)

    def test_fine_search_cannot_place_a_candidate_in_unknown_ground(self):
        grid, scan, _, _ = open_room((.075, 1.7), unknown_columns=4)
        match = global_match.global_scan_match(grid, scan)
        self.assertIsNotNone(match)
        for candidate in match.candidates:
            self.assertTrue(grid.is_known(*candidate.pose[:2]), candidate.pose)

    def test_all_top_candidates_preserve_existing_coarse_clearance(self):
        grid, scan, occupied, known = open_room()
        near = sliding_window_view(np.pad(occupied.astype(bool), 3), (7, 7)).any(axis=(2, 3))
        room = sliding_window_view(np.pad(~near, 1), (3, 3)).all(axis=(2, 3))
        existing_allowed = known.astype(bool) & ~near & room
        match = global_match.global_scan_match(grid, scan)
        self.assertIsNotNone(match)
        for candidate in match.candidates:
            x, y, _ = candidate.pose
            col, row = math.floor(x/grid.resolution), math.floor(y/grid.resolution)
            self.assertTrue(0 <= col < grid.width and 0 <= row < grid.height, candidate.pose)
            self.assertTrue(existing_allowed[row, col], candidate.pose)

    def test_negative_fractional_cells_are_outside_saved_grid(self):
        grid = localizers._Grid(2, 2, bytearray([1, 0, 0, 0]),
                               bytearray([1, 1, 1, 1]), .05, (1., 2.))
        self.assertIsNone(grid._index(.999, 2.025))
        self.assertIsNone(grid._index(1.025, 1.999))
        self.assertFalse(grid.is_known(.999, 2.025))
        self.assertFalse(grid.occupied_near(.999, 2.025, 0))
        self.assertEqual(grid._index(1., 2.), 0)

    def test_a_beam_just_outside_the_map_cannot_match_its_edge_cell(self):
        grid = localizers._Grid(2, 2, bytearray([1, 0, 0, 0]),
                               bytearray([1, 1, 1, 1]), .05, (0., 0.))
        score, inside = global_match._score(grid, np.zeros((2, 2)),
            np.ones((2, 2), dtype=bool), np.array([.05]), np.array([0.]),
            np.array([-.075]), np.array([.025]), np.array([0.]))
        self.assertEqual(inside[0], 0)
        self.assertEqual(score[0], 0.)


if __name__ == '__main__':
    unittest.main()
