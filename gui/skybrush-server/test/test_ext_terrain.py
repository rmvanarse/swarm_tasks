import gzip
import struct
import zlib

import numpy as np
import pytest

from flockwave.server.ext.terrain import extension as terrain
from flockwave.server.ext.terrain.dem import (
    TerrainStore,
    tile_name,
    tiles_in_circle,
)
from flockwave.server.ext.terrain.hillshade import (
    MIN_ZOOM,
    render_tile,
    tile_bounds,
)

SIDE = 101  # synthetic tile: 101 x 101 samples over one degree


def make_tile(func) -> bytes:
    """A gzipped .hgt whose sample at (row, col) is func(row, col)."""
    rows, cols = np.mgrid[0:SIDE, 0:SIDE]
    values = np.asarray(func(rows, cols)).astype(">i2")
    return gzip.compress(values.tobytes())


def store_with(tmp_path, tiles: dict[str, bytes | None], **kwargs) -> TerrainStore:
    requested = []

    def fetch(url):
        requested.append(url)
        for name, data in tiles.items():
            if f"/{name}.hgt.gz" in url:
                return data
        return None

    store = TerrainStore(tmp_path, fetch=fetch, **kwargs)
    store.requested = requested
    return store


def decode_png(data: bytes) -> np.ndarray:
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, width, height, idat = 8, 0, 0, b""
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        kind = data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + length]
        if kind == b"IHDR":
            width, height = struct.unpack(">II", body[:8])
        elif kind == b"IDAT":
            idat += body
        pos += 12 + length
    raw = np.frombuffer(zlib.decompress(idat), dtype=np.uint8)
    return raw.reshape(height, 1 + width * 4)[:, 1:].reshape(height, width, 4)


# -- naming ----------------------------------------------------------------------


def test_tile_names():
    assert tile_name(12.93, 80.05) == "N12E080"
    assert tile_name(-0.5, -0.5) == "S01W001"
    assert tile_name(47.2, 19.9) == "N47E019"
    assert tile_name(12.0, 80.0) == "N12E080"  # boundary belongs to the tile it starts


def test_tiles_in_circle_small_and_large():
    assert tiles_in_circle(12.93, 80.05, 5) == ["N12E080"]
    # 60 km reaches lat 12.39..13.47 and lon 79.5..80.6: a 2 x 2 block of tiles
    assert set(tiles_in_circle(12.93, 80.05, 60)) == {
        "N12E079",
        "N12E080",
        "N13E079",
        "N13E080",
    }
    assert len(tiles_in_circle(12.93, 80.05, 150)) > 4


# -- elevation -------------------------------------------------------------------


def test_elevation_interpolates_and_has_correct_orientation(tmp_path):
    # Height grows to the east (columns) by 1 m per sample, to the north (rows
    # go south, so decrease with row) by 10 m per sample.
    store = store_with(
        tmp_path, {"N12E080": make_tile(lambda r, c: 1000 + c * 1 + (SIDE - 1 - r) * 10)}
    )
    assert store.download("N12E080") == "downloaded"

    # South-west corner: row SIDE-1, col 0 -> 1000
    assert store.elevation(12.0, 80.0) == pytest.approx(1000.0)
    # North-west corner: +10 * 100 rows
    assert store.elevation(12.999999, 80.0) == pytest.approx(2000.0, abs=0.1)
    # South-east corner: +100
    assert store.elevation(12.0, 80.999999) == pytest.approx(1100.0, abs=0.1)
    # Half way between samples is the average
    mid = store.elevation(12.0 + 0.005, 80.0 + 0.005)  # half a sample north and east
    assert mid == pytest.approx(1000 + 0.5 + 5.0)


def test_elevation_unknown_without_tile_and_offline_does_not_download(tmp_path):
    store = store_with(tmp_path, {"N12E080": make_tile(lambda r, c: r * 0 + 5)}, offline=True)
    assert store.elevation(12.5, 80.5) is None
    assert store.download("N12E080") == "offline"
    assert store.requested == []  # nothing was fetched
    assert np.isnan(store.elevations([12.5], [80.5], download=True)[0])


def test_download_missing_and_failure_are_remembered(tmp_path):
    store = store_with(tmp_path, {})
    assert store.download("N00E000") == "missing"
    assert store.has_tile("N00E000") is False

    def boom(url):
        raise OSError("no network")

    broken = TerrainStore(tmp_path / "x", fetch=boom)
    assert broken.download("N12E080") == "failed"
    assert broken.download("N12E080") == "failed"  # remembered, not retried at once


def test_void_samples_give_no_data(tmp_path):
    store = store_with(
        tmp_path, {"N12E080": make_tile(lambda r, c: np.where(c > 50, -32768, 7))}
    )
    store.download("N12E080")
    assert store.elevation(12.5, 80.2) == pytest.approx(7.0)
    assert store.elevation(12.5, 80.8) is None


def test_vector_lookup_across_two_tiles(tmp_path):
    store = store_with(
        tmp_path,
        {
            "N12E080": make_tile(lambda r, c: r * 0 + 100),
            "N12E081": make_tile(lambda r, c: r * 0 + 300),
        },
    )
    store.prefetch(["N12E080", "N12E081"])
    h = store.elevations([12.5, 12.5, 12.5], [80.5, 81.5, 82.5])
    assert h[0] == pytest.approx(100) and h[1] == pytest.approx(300)
    assert np.isnan(h[2])


def test_extra_data_folder_is_read_first_never_written_and_works_offline(tmp_path):
    import numpy as np

    extra = tmp_path / "mine"
    extra.mkdir()
    own = np.full((SIDE, SIDE), 777, dtype=">i2")
    (extra / "N12E080.hgt").write_bytes(own.tobytes())

    store = store_with(
        tmp_path / "cache",
        {"N12E080": make_tile(lambda r, c: r * 0 + 5), "N13E080": make_tile(lambda r, c: r * 0 + 9)},
        data_dirs=[extra],
        offline=True,
    )
    # the ready-made tile is used even with downloads switched off ...
    assert store.has_tile("N12E080") and store.download("N12E080") == "present"
    assert store.elevation(12.5, 80.5) == pytest.approx(777.0)
    # ... and a tile it does not have is not fetched in offline mode
    assert store.elevation(13.5, 80.5) is None and store.requested == []
    assert store.available_tiles() == ["N12E080"]
    assert store.stats() == (1, extra.joinpath("N12E080.hgt").stat().st_size)

    # online, it still prefers the extra folder and downloads only what is missing
    online = store_with(
        tmp_path / "cache2",
        {"N12E080": make_tile(lambda r, c: r * 0 + 5), "N13E080": make_tile(lambda r, c: r * 0 + 9)},
        data_dirs=[extra],
    )
    assert online.prefetch(["N12E080", "N13E080"]) == {"present": ["N12E080"], "downloaded": ["N13E080"]}
    assert online.elevation(12.5, 80.5) == pytest.approx(777.0)
    assert online.elevation(13.5, 80.5) == pytest.approx(9.0)
    assert not (tmp_path / "cache2" / "N12E080.hgt").exists()  # nothing copied or written there
    assert (extra / "N12E080.hgt").read_bytes() == own.tobytes()  # untouched


def test_prefetch_summary(tmp_path):
    store = store_with(tmp_path, {"N12E080": make_tile(lambda r, c: r * 0)})
    result = store.prefetch(["N12E080", "N12E080", "N13E080"])
    assert result == {"downloaded": ["N12E080"], "missing": ["N13E080"]}
    assert store.prefetch(["N12E080"]) == {"present": ["N12E080"]}
    assert store.stats()[0] == 1


# -- path sampling and the exported helpers ---------------------------------------


def test_sample_path_spacing_and_distance():
    # ~0.01 degree of latitude = ~1113 m
    dist, lats, lons = terrain.sample_path([(12.0, 80.0), (12.01, 80.0)], 100)
    assert dist[0] == 0 and lats[0] == 12.0 and lats[-1] == pytest.approx(12.01)
    assert dist[-1] == pytest.approx(1113, rel=0.01)
    assert np.all(np.diff(dist) <= 100 + 1e-6)
    assert len(dist) >= 12


def test_ground_helpers_clamp_sea_floor(tmp_path, monkeypatch):
    store = store_with(tmp_path, {"N12E080": make_tile(lambda r, c: r * 0 - 50)})
    store.download("N12E080")
    monkeypatch.setattr(terrain, "_store", store)
    assert terrain.get_ground_height(12.5, 80.5) == 0.0  # below sea level -> 0
    assert terrain.get_ground_height(40.5, 10.5) is None  # no tile
    profile = terrain.get_path_profile([(12.4, 80.5), (12.6, 80.5)], 500)
    assert profile["max"] == 0.0 and profile["unknown"] == 0


# -- map tiles --------------------------------------------------------------------


def _tile_covering(lat, lon, z):
    n = 2**z
    import math

    x = int((lon + 180) / 360 * n)
    y = int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)
    return x, y


def test_png_is_valid_and_transparent_outside_zoom_range(tmp_path):
    store = store_with(tmp_path, {})
    for z in (0, MIN_ZOOM - 1, 18):
        pixels = decode_png(render_tile(store, z, 0, 0))
        assert pixels.shape == (256, 256, 4) and pixels[..., 3].max() == 0


def test_tile_without_data_is_transparent(tmp_path):
    store = store_with(tmp_path, {})
    x, y = _tile_covering(12.5, 80.5, 12)
    assert decode_png(render_tile(store, 12, x, y))[..., 3].max() == 0


def test_flat_land_is_transparent_and_slope_is_shaded_by_direction(tmp_path):
    flat = store_with(tmp_path / "flat", {"N12E080": make_tile(lambda r, c: r * 0 + 400)})
    flat.download("N12E080")
    x, y = _tile_covering(12.5, 80.5, 12)
    assert decode_png(render_tile(flat, 12, x, y))[..., 3].max() <= 1

    # Ground rising to the east faces west: toward the north-west light.
    east_up = store_with(
        tmp_path / "east", {"N12E080": make_tile(lambda r, c: 200 + c * 12)}
    )
    east_up.download("N12E080")
    lit = decode_png(render_tile(east_up, 12, x, y))
    bright = lit[..., 3] > 0
    assert bright.any()
    assert (lit[..., :3][bright] == 255).all()  # white highlight, not dark

    # Ground rising to the west faces east: away from the light, so dark.
    west_up = store_with(
        tmp_path / "west", {"N12E080": make_tile(lambda r, c: 200 + (SIDE - c) * 12)}
    )
    west_up.download("N12E080")
    shadow = decode_png(render_tile(west_up, 12, x, y))
    dark = shadow[..., 3] > 0
    assert dark.any()
    assert (shadow[..., :3][dark] == 0).all()  # black shadow


def test_relief_style_tints_by_height(tmp_path):
    hill = store_with(tmp_path, {"N12E080": make_tile(lambda r, c: 300 + c * 12)})
    hill.download("N12E080")
    x, y = _tile_covering(12.5, 80.5, 12)
    png = decode_png(render_tile(hill, 12, x, y, "relief"))
    assert (png[..., 3] > 0).all()  # all land is coloured, even where flat-ish


def test_tile_bounds_sanity():
    north, south, west, east = tile_bounds(0, 0, 0)
    assert west == -180 and east == 180 and north == pytest.approx(85.0511, abs=1e-3)


# -- exaggeration and contour lines -------------------------------------------------


def test_exaggeration_grows_with_zoom_and_contours_get_finer():
    from flockwave.server.ext.terrain.hillshade import (
        contour_interval,
        vertical_exaggeration,
    )

    assert vertical_exaggeration(8) == vertical_exaggeration(11) == 2.0
    assert vertical_exaggeration(17) > vertical_exaggeration(14) > 2.0
    steps = [contour_interval(z) for z in range(8, 18)]
    assert steps == sorted(steps, reverse=True) and steps[0] > steps[-1]


def test_gentle_slope_is_visible_when_zoomed_in(tmp_path):
    # 1 m of rise per 30 m (3 %): each synthetic sample is ~1.1 km apart, so 37 m
    # per sample. Without exaggeration that shades to under 8/255.
    gentle = store_with(tmp_path, {"N12E080": make_tile(lambda r, c: 20 + c * 37)})
    gentle.download("N12E080")
    x, y = _tile_covering(12.5, 80.5, 17)
    alpha = decode_png(render_tile(gentle, 17, x, y))[..., 3]
    assert alpha.max() > 20


def test_contours_are_drawn_only_when_asked_and_only_on_land(tmp_path):
    hill = store_with(tmp_path, {"N12E080": make_tile(lambda r, c: 100 + c * 12)})
    hill.download("N12E080")
    x, y = _tile_covering(12.5, 80.5, 12)
    plain = decode_png(render_tile(hill, 12, x, y))
    lined = decode_png(render_tile(hill, 12, x, y, contours=True))
    dark = (lined[..., :3].sum(axis=-1) < 200) & (lined[..., 3] > 100)
    assert dark.any()
    assert (plain != lined).any()  # the lines are an addition, not the default
    # a flat plateau has no lines at all
    flat = store_with(tmp_path / "f", {"N12E080": make_tile(lambda r, c: r * 0 + 300)})
    flat.download("N12E080")
    assert decode_png(render_tile(flat, 12, x, y, contours=True))[..., 3].max() <= 1
