#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Visualise the DUFOMap before/after diff for M5R-4 (Issue #49).

The acceptance criterion for Issue #49 is "pedestrian trails in the
dynamic bag are visibly gone after DUFOMap" (m5r-execution plan §6 B2).
That check is by-eye on the overlaid clouds — there is no scalar metric
short of hand-segmented ground truth that captures "dynamic streaks are
removed".  This script provides the overlay:

* ``--before`` (typically a raw merged GLIM cloud) is rendered in red.
* ``--after`` (the DUFOMap-cleaned static cloud) is rendered in blue.
* Without ``--screenshot``, an interactive Open3D viewer opens for
  manual inspection.
* With ``--screenshot <path>``, an off-screen render is captured as PNG
  for inclusion in the M5-R bench-data folder.

This is an interactive / forensic tool, not a CI tool — ``open3d`` is
imported only here, so CI and headless build environments do not pay
its install cost.

Usage:

    scripts/m5r_dufomap_diff.py \\
        --before raw.pcd --after static.pcd

    scripts/m5r_dufomap_diff.py \\
        --before raw.pcd --after static.pcd \\
        --screenshot diff.png
"""

import argparse
import sys
from pathlib import Path


def _import_open3d():
    """Defer the open3d import until we actually need it.

    The wheel is ~100 MB and pulls in a graphics stack; pushing the
    import behind a function keeps --help cheap on hosts without it.
    """
    try:
        import open3d as o3d  # type: ignore
    except ImportError as exc:
        print(
            "ERROR: open3d is not installed. Install with "
            "`pip install open3d` (note: large wheel, ~100 MB). "
            f"Underlying error: {exc}",
            file=sys.stderr,
        )
        raise
    return o3d


def load_colored(o3d, path: Path, rgb: tuple[float, float, float]):
    """Read a PCD and uniformly paint it for the overlay."""
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found")
    pcd = o3d.io.read_point_cloud(str(path))
    if len(pcd.points) == 0:
        # open3d returns an empty cloud rather than raising when the
        # file is unreadable. Surface the failure so the user does not
        # stare at an empty viewer wondering why nothing rendered.
        raise ValueError(
            f"{path} loaded but contains 0 points — file may be "
            "malformed or written by an incompatible PCD producer."
        )
    pcd.paint_uniform_color(list(rgb))
    return pcd


def render_interactive(o3d, before, after) -> None:
    """Open a blocking Open3D window for manual inspection."""
    o3d.visualization.draw_geometries(
        [before, after],
        window_name="DUFOMap before (red) / after (blue)",
    )


def render_screenshot(o3d, before, after, out_png: Path) -> None:
    """Render off-screen and save to PNG.

    Uses the OffscreenRenderer interface so headless hosts (no display)
    can still produce a screenshot for embedding in the bench-data
    write-up.  Falls back to a windowed capture if OffscreenRenderer
    is unavailable on the platform.
    """
    out_png.parent.mkdir(parents=True, exist_ok=True)

    try:
        # Modern open3d (>=0.13) offers OffscreenRenderer.
        renderer = o3d.visualization.rendering.OffscreenRenderer(1280, 960)
        scene = renderer.scene
        scene.set_background([1.0, 1.0, 1.0, 1.0])
        # Material setup: defaultLit for shaded view of the painted
        # clouds. Point size 2 makes sparse outdoor returns legible.
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = 2.0
        scene.add_geometry("before", before, mat)
        scene.add_geometry("after", after, mat)

        # Frame both clouds in view.
        bbox = before.get_axis_aligned_bounding_box() + \
            after.get_axis_aligned_bounding_box()
        scene.camera.look_at(
            bbox.get_center(),
            bbox.get_center() + [0, 0, max(bbox.get_extent()) * 1.5],
            [0, 1, 0],
        )
        img = renderer.render_to_image()
        o3d.io.write_image(str(out_png), img)
    except Exception as exc:  # noqa: BLE001
        # Some Open3D builds (e.g. wheels missing EGL) fail at
        # OffscreenRenderer construction time. Retry with a hidden
        # interactive window so the user still gets a screenshot.
        print(f"WARNING: OffscreenRenderer failed ({exc}); falling back "
              "to hidden window capture", file=sys.stderr)
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=1280, height=960)
        vis.add_geometry(before)
        vis.add_geometry(after)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(str(out_png), do_render=True)
        vis.destroy_window()

    print(f"Screenshot written to {out_png}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Overlay a before/after PCD pair (raw vs DUFOMap-cleaned) "
            "for visual verification of dynamic-object removal."
        ),
    )
    parser.add_argument("--before", required=True, type=Path,
                        help="PCD before dynamic removal (rendered red)")
    parser.add_argument("--after", required=True, type=Path,
                        help="PCD after dynamic removal (rendered blue)")
    parser.add_argument("--screenshot", type=Path, default=None,
                        help="If set, render off-screen and save to PNG "
                             "instead of opening an interactive viewer")
    args = parser.parse_args()

    try:
        o3d = _import_open3d()
        before = load_colored(o3d, args.before, (1.0, 0.0, 0.0))
        after = load_colored(o3d, args.after, (0.0, 0.4, 1.0))
    except (FileNotFoundError, ValueError, ImportError) as exc:
        if isinstance(exc, (FileNotFoundError, ValueError)):
            print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if args.screenshot is not None:
        render_screenshot(o3d, before, after, args.screenshot)
    else:
        render_interactive(o3d, before, after)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
