#!/usr/bin/env python3
"""
fetch_env_data.py — download real bathymetry and ocean-current data,
preprocess them, and write the result in the bundled binary grid format
that the C++ server consumes.

Bathymetry source:
  GMT earth_relief from generic-mapping-tools.org. The grids are public-domain
  global topography/bathymetry at multiple resolutions (15-arc-minute, 10-arc-
  minute, 6-arc-minute, etc.), in NetCDF-4 (HDF5) format. We download the
  whole grid (the file is small at 15m–10m) and write our flat binary.

Current source:
  HYCOM GLBy0.08 expt_93.0 via the public OPeNDAP server at tds.hycom.org.
  HYCOM is 1/12-degree resolution (4251 x 4500 cells) at 40 standard depth
  levels. We subset to a coarser grid (every Nth cell in lat/lon) at a chosen
  subset of depth levels, fetch via OPeNDAP ASCII subsetting, and write our
  flat binary.

Output formats are documented in server/include/physics/raster_grid.hpp.

Dependencies:
    pip install numpy netCDF4 h5py requests
(netCDF4 is preferred; we fall back to h5py if it's not available.)
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import tempfile
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

# --- File-format constants (must match physics/raster_grid.hpp) -----------

BATHY_MAGIC = 0x59485442  # 'BTHY'
CURRENTS_MAGIC = 0x52525543  # 'CURR'
FORMAT_VERSION = 1

# Bathymetry header: 64 bytes (matches physics/raster_grid.hpp::BathyHeader)
#   uint32 magic, version, width, height        : 16 bytes
#   double lat_min, lat_max, lon_min, lon_max   : 32 bytes
#   float  nodata                                :  4 bytes
#   uint8  pad[12]                               : 12 bytes
BATHY_HEADER_FMT = "<IIIIdddd f 12s"
assert struct.calcsize(BATHY_HEADER_FMT) == 64, struct.calcsize(BATHY_HEADER_FMT)

# Currents header: 96 bytes (matches physics/raster_grid.hpp::CurrentsHeader)
#   uint32 magic, version, width, height, n_levels, reserved : 24 bytes
#   double lat_min, lat_max, lon_min, lon_max                : 32 bytes
#   float  nodata                                             :  4 bytes
#   uint8  pad[36]                                            : 36 bytes
CURRENTS_HEADER_FMT = "<IIIIII dddd f 36s"
assert struct.calcsize(CURRENTS_HEADER_FMT) == 96, struct.calcsize(CURRENTS_HEADER_FMT)

# Surface + standard mission depths. We keep this list short on purpose:
# the C++ trilinear interpolator handles arbitrary depths between samples,
# and most AUV missions stay above ~500 m.
DEFAULT_DEPTH_INDICES = [0, 5, 10, 14, 19, 22, 27]
# Corresponds to HYCOM depths_m = [0, 10, 30, 50, 100, 200, 500].

# --- Logging helpers ------------------------------------------------------

def log(msg: str) -> None:
    print(f"[fetch_env_data] {msg}", flush=True)

# --- Bathymetry: GMT earth_relief -----------------------------------------

GMT_BATHY_URLS = {
    "15m": "https://oceania.generic-mapping-tools.org/server/earth/earth_relief/earth_relief_15m_p.grd",
    "10m": "https://oceania.generic-mapping-tools.org/server/earth/earth_relief/earth_relief_10m_p.grd",
    "06m": "https://oceania.generic-mapping-tools.org/server/earth/earth_relief/earth_relief_06m_p.grd",
    "20m": "https://oceania.generic-mapping-tools.org/server/earth/earth_relief/earth_relief_20m_p.grd",
    "30m": "https://oceania.generic-mapping-tools.org/server/earth/earth_relief/earth_relief_30m_p.grd",
}


def http_get(url: str, dst_path: Path, timeout: int = 240) -> None:
    log(f"GET {url}")
    req = urllib.request.Request(url, headers={"User-Agent": "bathyscaphe-fetch/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        if resp.status != 200:
            raise RuntimeError(f"HTTP {resp.status} for {url}")
        with dst_path.open("wb") as f:
            while True:
                chunk = resp.read(1 << 20)
                if not chunk:
                    break
                f.write(chunk)
    log(f"  -> {dst_path} ({dst_path.stat().st_size} bytes)")


def open_netcdf_dataset(path: Path):
    """Return (elev_array, lat_array, lon_array). Tries netCDF4 then h5py."""
    try:
        import netCDF4  # type: ignore
    except ImportError:
        netCDF4 = None  # noqa: N806

    if netCDF4 is not None:
        ds = netCDF4.Dataset(path)  # noqa: F841 - keep alive
        try:
            z = _find_variable(ds.variables, ["z", "elevation", "topo", "Band1"])
            lat = _find_variable(ds.variables, ["lat", "latitude", "y"])
            lon = _find_variable(ds.variables, ["lon", "longitude", "x"])
            return (z[:].astype("float32"), lat[:].astype("float64"), lon[:].astype("float64"))
        finally:
            pass

    try:
        import h5py  # type: ignore
        with h5py.File(path, "r") as h:
            keys = {k.lower() for k in h.keys()}
            z_key = next((k for k in h.keys() if k.lower() in ("z", "elevation", "topo", "band1")), None)
            lat_key = next((k for k in h.keys() if k.lower() in ("lat", "latitude", "y")), None)
            lon_key = next((k for k in h.keys() if k.lower() in ("lon", "longitude", "x")), None)
            if not (z_key and lat_key and lon_key):
                raise RuntimeError(f"could not find z/lat/lon in HDF5 keys: {keys}")
            import numpy as np
            return (np.asarray(h[z_key]).astype("float32"),
                    np.asarray(h[lat_key]).astype("float64"),
                    np.asarray(h[lon_key]).astype("float64"))
    except ImportError:
        raise RuntimeError(
            "Neither netCDF4 nor h5py is installed. "
            "Install one: pip install netCDF4   (preferred), or pip install h5py"
        )


def _find_variable(vmap, candidates):
    for name in candidates:
        if name in vmap:
            return vmap[name]
    raise RuntimeError(f"could not find any of {candidates} in NetCDF variables {list(vmap.keys())}")


def write_bathy(out: Path, elev, lat, lon) -> None:
    import numpy as np

    elev = np.asarray(elev, dtype="float32")
    lat = np.asarray(lat, dtype="float64")
    lon = np.asarray(lon, dtype="float64")

    # Some GMT grids index y from north to south; our format expects j=0 at
    # the minimum latitude. Normalize.
    if lat[0] > lat[-1]:
        log("  bathy: latitude axis is descending; flipping")
        lat = lat[::-1]
        elev = elev[::-1, :]
    # Same possibility for longitude, though much rarer.
    if lon[0] > lon[-1]:
        log("  bathy: longitude axis is descending; flipping")
        lon = lon[::-1]
        elev = elev[:, ::-1]

    height, width = elev.shape
    if (height, width) != (lat.size, lon.size):
        raise RuntimeError(f"elev shape {elev.shape} doesn't match axes ({lat.size}, {lon.size})")

    nodata = -1.0e30  # well outside any plausible elevation
    elev = np.nan_to_num(elev, nan=nodata, posinf=nodata, neginf=nodata).astype("float32")

    header = struct.pack(
        BATHY_HEADER_FMT,
        BATHY_MAGIC,
        FORMAT_VERSION,
        width,
        height,
        float(lat[0]),
        float(lat[-1]),
        float(lon[0]),
        float(lon[-1]),
        nodata,
        b"\x00" * 12,
    )
    with out.open("wb") as f:
        f.write(header)
        f.write(elev.tobytes(order="C"))
    log(f"  wrote {out} (width={width} height={height}, "
        f"lat [{lat[0]:.3f},{lat[-1]:.3f}], lon [{lon[0]:.3f},{lon[-1]:.3f}], "
        f"min_elev={float(elev[elev > nodata].min()):.0f} m, "
        f"max_elev={float(elev[elev > nodata].max()):.0f} m, "
        f"{out.stat().st_size} bytes)")


def cmd_bathy(args: argparse.Namespace) -> None:
    if args.resolution not in GMT_BATHY_URLS:
        raise SystemExit(f"unknown resolution {args.resolution!r}; pick from {sorted(GMT_BATHY_URLS)}")
    url = GMT_BATHY_URLS[args.resolution]

    with tempfile.NamedTemporaryFile(suffix=".grd", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        http_get(url, tmp_path)
        elev, lat, lon = open_netcdf_dataset(tmp_path)
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        write_bathy(out, elev, lat, lon)
    finally:
        try:
            tmp_path.unlink()
        except FileNotFoundError:
            pass

# --- Currents: HYCOM via OPeNDAP ASCII -----------------------------------

HYCOM_BASE = "https://tds.hycom.org/thredds/dodsC/GLBy0.08/expt_93.0/uv3z"

# Full HYCOM grid is 4251 lat × 4500 lon × 40 depth.
HYCOM_LAT_N = 4251
HYCOM_LON_N = 4500
HYCOM_DEPTHS_M = [
    0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0,
    30.0, 35.0, 40.0, 45.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0,
    125.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 500.0, 600.0, 700.0,
    800.0, 900.0, 1000.0, 1250.0, 1500.0, 2000.0, 2500.0, 3000.0, 4000.0, 5000.0,
]
assert len(HYCOM_DEPTHS_M) == 40


def hycom_ascii_subset(var: str, time_idx: int, depth_idx: int,
                       lat_start: int, lat_stop: int, lat_stride: int,
                       lon_start: int, lon_stop: int, lon_stride: int,
                       timeout: int = 180):
    """Fetch one variable, one time slice, one depth, with strided lat/lon
    bounding-box subsetting via OPeNDAP ASCII. Returns (lat_axis, lon_axis,
    values) where values is a 2-D numpy array shaped (n_lat, n_lon) of scaled
    floats in m/s (NaN where the source had missing data)."""
    import numpy as np

    spec = (
        f"{var}[{time_idx}][{depth_idx}]"
        f"[{lat_start}:{lat_stride}:{lat_stop}]"
        f"[{lon_start}:{lon_stride}:{lon_stop}]"
    )
    qs = urllib.parse.quote(spec, safe=":[],")
    url = f"{HYCOM_BASE}.ascii?{qs}"
    log(f"  HYCOM {var} t={time_idx} d_idx={depth_idx} ({HYCOM_DEPTHS_M[depth_idx]} m)")
    req = urllib.request.Request(url, headers={"User-Agent": "bathyscaphe-fetch/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        if resp.status != 200:
            raise RuntimeError(f"HTTP {resp.status} for {url}")
        body = resp.read().decode("ascii", errors="replace")

    # OPeNDAP ASCII format:
    #   Dataset { ... }
    #   ---------------------------------------------
    #   var.var[a][b][c][d]
    #   [0][0][0], v00, v01, v02, ...
    #   [0][0][1], v10, v11, ...
    #   ...
    #
    #   var.time[1]
    #   t0
    #
    #   var.depth[1]
    #   d0
    #
    #   var.lat[N]
    #   l0, l1, ..., lN-1
    #
    #   var.lon[M]
    #   m0, m1, ..., mM-1
    sections = body.split("---------------------------------------------", 1)
    if len(sections) < 2:
        raise RuntimeError(f"unexpected OPeNDAP ASCII payload for {url}:\n{body[:500]}")
    payload = sections[1]

    # Split into blocks. Each block starts with a header line "name[...]" then
    # comma-separated values.
    blocks: dict[str, str] = {}
    current_header = None
    current_lines: list[str] = []
    for raw_line in payload.splitlines():
        line = raw_line.strip()
        if not line:
            if current_header is not None:
                blocks[current_header] = "\n".join(current_lines).strip()
                current_header = None
                current_lines = []
            continue
        # Heading line: contains a letter and brackets but no leading [0][0]...
        if (line[0].isalpha() and "[" in line and "]" in line
                and "." in line.split("[", 1)[0] or line.startswith(f"{var}")):
            if current_header is not None:
                blocks[current_header] = "\n".join(current_lines).strip()
            current_header = line.split("[", 1)[0].strip()
            current_lines = []
            continue
        current_lines.append(line)
    if current_header is not None:
        blocks[current_header] = "\n".join(current_lines).strip()

    # The variable block is the one named exactly `var.var` (OPeNDAP convention
    # for Grid containers) or `var`. Resolve robustly.
    var_block = None
    for k, v in blocks.items():
        kk = k.split(".")[-1]
        if kk == var:
            var_block = v
            break
    if var_block is None:
        raise RuntimeError(f"could not find {var} block in ASCII response. keys: {list(blocks)}")

    # Parse the variable block: each line looks like
    #   "[t][d][lat_i], v_lon0, v_lon1, ..., v_lonN-1"
    rows = []
    for line in var_block.splitlines():
        parts = line.split(",")
        if len(parts) < 2:
            continue
        if not parts[0].startswith("["):
            continue
        try:
            vals = [int(p) for p in parts[1:]]
        except ValueError:
            continue
        rows.append(vals)
    if not rows:
        raise RuntimeError(f"no rows parsed for {var} block")

    n_lat = len(rows)
    n_lon = len(rows[0])
    arr = np.zeros((n_lat, n_lon), dtype="float32")
    for j, row in enumerate(rows):
        if len(row) != n_lon:
            raise RuntimeError(
                f"ragged row {j}: expected {n_lon} values, got {len(row)}"
            )
        arr[j] = row

    # Decode HYCOM's Int16 packing: missing = -30000, scale = 0.001.
    missing = -30000
    scale = 0.001
    mask = arr == missing
    arr = arr * scale
    arr[mask] = float("nan")

    # Pull the lat/lon axes by looking for their blocks.
    lat_block = blocks.get("lat") or _find_axis_block(blocks, "lat")
    lon_block = blocks.get("lon") or _find_axis_block(blocks, "lon")
    if not lat_block or not lon_block:
        raise RuntimeError(f"missing lat/lon axis blocks in response. keys: {list(blocks)}")
    lat_axis = np.array(_parse_csv_floats(lat_block), dtype="float64")
    lon_axis = np.array(_parse_csv_floats(lon_block), dtype="float64")
    if lat_axis.size != n_lat or lon_axis.size != n_lon:
        raise RuntimeError(
            f"axis sizes ({lat_axis.size}x{lon_axis.size}) "
            f"don't match data ({n_lat}x{n_lon})"
        )
    return lat_axis, lon_axis, arr


def _find_axis_block(blocks, name):
    for k, v in blocks.items():
        if k.split(".")[-1] == name:
            return v
    return None


def _parse_csv_floats(text):
    out = []
    for line in text.splitlines():
        for tok in line.split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                out.append(float(tok))
            except ValueError:
                pass
    return out


def cmd_currents(args: argparse.Namespace) -> None:
    import numpy as np

    if args.depth_indices:
        depth_indices = [int(s) for s in args.depth_indices.split(",")]
    else:
        depth_indices = DEFAULT_DEPTH_INDICES

    for di in depth_indices:
        if di < 0 or di >= len(HYCOM_DEPTHS_M):
            raise SystemExit(f"depth index {di} out of range [0, {len(HYCOM_DEPTHS_M) - 1}]")

    stride = max(1, int(args.stride))
    lat_start = 0
    lat_stop = HYCOM_LAT_N - 1
    lon_start = 0
    lon_stop = HYCOM_LON_N - 1
    n_lat = (lat_stop - lat_start) // stride + 1
    n_lon = (lon_stop - lon_start) // stride + 1
    log(f"target grid: {n_lon} lon × {n_lat} lat × {len(depth_indices)} depths "
        f"(stride={stride}, time_idx={args.time_idx})")

    lat_axis = None
    lon_axis = None
    u_stack = np.zeros((len(depth_indices), n_lat, n_lon), dtype="float32")
    v_stack = np.zeros((len(depth_indices), n_lat, n_lon), dtype="float32")
    depths_out = np.zeros(len(depth_indices), dtype="float32")

    for k, di in enumerate(depth_indices):
        for retries in range(3):
            try:
                la_u, lo_u, U = hycom_ascii_subset(
                    "water_u", args.time_idx, di,
                    lat_start, lat_stop, stride,
                    lon_start, lon_stop, stride,
                )
                la_v, lo_v, V = hycom_ascii_subset(
                    "water_v", args.time_idx, di,
                    lat_start, lat_stop, stride,
                    lon_start, lon_stop, stride,
                )
                break
            except Exception as e:
                log(f"    error: {e}; retry {retries + 1}/3")
                time.sleep(2)
        else:
            raise SystemExit(f"failed to fetch depth idx {di} after retries")

        if lat_axis is None:
            lat_axis = la_u
            lon_axis = lo_u
        u_stack[k] = U
        v_stack[k] = V
        depths_out[k] = HYCOM_DEPTHS_M[di]

    # Convert HYCOM nan-on-land to our sentinel.
    nodata = -1.0e30
    u_stack = np.where(np.isnan(u_stack), nodata, u_stack).astype("float32")
    v_stack = np.where(np.isnan(v_stack), nodata, v_stack).astype("float32")

    # HYCOM longitude is in [-180, 180) but may run 0..360 in some variants.
    # Normalize to ascending order matching lat axis.
    assert lat_axis is not None and lon_axis is not None
    if lat_axis[0] > lat_axis[-1]:
        lat_axis = lat_axis[::-1].copy()
        u_stack = u_stack[:, ::-1, :]
        v_stack = v_stack[:, ::-1, :]
    if lon_axis[0] > lon_axis[-1]:
        lon_axis = lon_axis[::-1].copy()
        u_stack = u_stack[:, :, ::-1]
        v_stack = v_stack[:, :, ::-1]

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    header = struct.pack(
        CURRENTS_HEADER_FMT,
        CURRENTS_MAGIC,
        FORMAT_VERSION,
        n_lon,
        n_lat,
        len(depth_indices),
        0,
        float(lat_axis[0]),
        float(lat_axis[-1]),
        float(lon_axis[0]),
        float(lon_axis[-1]),
        nodata,
        b"\x00" * 36,
    )
    with out.open("wb") as f:
        f.write(header)
        f.write(depths_out.tobytes(order="C"))
        f.write(u_stack.tobytes(order="C"))
        f.write(v_stack.tobytes(order="C"))
    log(f"wrote {out} ({out.stat().st_size} bytes)")

    # Quick sanity stats on the surface slice.
    surface_u = u_stack[0]
    surface_v = v_stack[0]
    valid = (surface_u != nodata) & (surface_v != nodata)
    if valid.any():
        mag = np.hypot(surface_u[valid], surface_v[valid])
        log(f"  surface |c| stats: mean={mag.mean():.3f}, "
            f"max={mag.max():.3f}, valid_cells={int(valid.sum())}")

# --- Bundle command (default dataset) -------------------------------------

def cmd_bundle(args: argparse.Namespace) -> None:
    """Convenience: fetch a sensible default bathymetry and current set."""
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1) Bathymetry at 15-arc-minute (≈4 MB binary)
    bathy_args = argparse.Namespace(
        resolution="15m",
        output=str(out_dir / "earth_relief_15m.bath"),
    )
    cmd_bathy(bathy_args)

    # 2) Currents at 1-degree (every 12th HYCOM cell) at 7 standard depths
    cur_args = argparse.Namespace(
        stride=12,
        time_idx=args.time_idx,
        depth_indices="",
        output=str(out_dir / "hycom_1deg_7depths.curr"),
    )
    cmd_currents(cur_args)

# --- CLI ------------------------------------------------------------------

def main() -> None:
    here = Path(__file__).resolve().parent
    default_data_dir = here.parent / "server" / "data"

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    pb = sub.add_parser("bathymetry", help="Download + pack global bathymetry.")
    pb.add_argument("--resolution", default="15m",
                    choices=sorted(GMT_BATHY_URLS),
                    help="GMT earth_relief grid resolution. 15m ≈ 4 MB binary, "
                         "10m ≈ 9 MB, 06m ≈ 25 MB. Default: 15m.")
    pb.add_argument("--output", default=str(default_data_dir / "earth_relief_15m.bath"))
    pb.set_defaults(func=cmd_bathy)

    pc = sub.add_parser("currents", help="Download + pack HYCOM currents.")
    pc.add_argument("--stride", type=int, default=12,
                    help="Lat/lon subsampling stride into the 1/12° HYCOM grid. "
                         "stride=12 gives ≈1° resolution. Default: 12.")
    pc.add_argument("--time-idx", type=int, default=100,
                    help="HYCOM time-axis index (a single snapshot). Default: 100.")
    pc.add_argument("--depth-indices", default="",
                    help="Comma-separated depth indices (0..39). "
                         "Empty = surface + 10/30/50/100/200/500 m.")
    pc.add_argument("--output", default=str(default_data_dir / "hycom_1deg_7depths.curr"))
    pc.set_defaults(func=cmd_currents)

    pba = sub.add_parser("bundle", help="Fetch both default datasets.")
    pba.add_argument("--output-dir", default=str(default_data_dir))
    pba.add_argument("--time-idx", type=int, default=100)
    pba.set_defaults(func=cmd_bundle)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
