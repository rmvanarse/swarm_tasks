import pytest

from flockwave.server.ext.map_cache.extension import (
    DEFAULT_ALLOWED_HOSTS,
    RejectedURL,
    cache_key_url,
    check_url,
    with_server_token,
)
from flockwave.server.ext.map_cache.store import TileStore

MAPBOX = "https://api.mapbox.com/v4/mapbox.satellite/3/4/5@2x.jpg90?access_token=KEY"


@pytest.mark.parametrize(
    "url",
    [
        MAPBOX,
        "https://a.tiles.mapbox.com/v4/x/1/2/3.vector.pbf?access_token=K",
        "https://tile.openstreetmap.org/1/2/3.png",
    ],
)
def test_allowed_urls(url):
    check_url(url, DEFAULT_ALLOWED_HOSTS)


@pytest.mark.parametrize(
    "url",
    [
        "",
        "http://127.0.0.1:8081/x",
        "https://evil.example.com/a.png",
        "https://api.mapbox.com.evil.com/x",
        "https://user:pw@api.mapbox.com/x",
        "file:///etc/passwd",
    ],
)
def test_rejected_urls(url):
    with pytest.raises(RejectedURL):
        check_url(url, DEFAULT_ALLOWED_HOSTS)


def test_cache_key_ignores_credentials():
    assert cache_key_url(MAPBOX) == cache_key_url(MAPBOX.replace("KEY", "OTHER"))
    assert "KEY" not in cache_key_url(MAPBOX)


def test_server_token_replaces_client_token_for_mapbox_only():
    url = with_server_token(MAPBOX, "SERVER")
    assert "access_token=SERVER" in url and "KEY" not in url
    maptiler = "https://api.maptiler.com/a?key=1"
    assert with_server_token(maptiler, "SERVER") == maptiler
    assert with_server_token(MAPBOX, "") == MAPBOX


def test_store_roundtrip_prune_and_clear(tmp_path):
    store = TileStore(tmp_path, max_bytes=100)
    a, b = "ab" + "0" * 62, "cd" + "0" * 62
    store.put(a, b"x" * 60, "image/png")
    store.put(b, b"y" * 60, "image/jpeg")
    assert store.stats() == (2, 120)

    store.prune()
    assert store.stats() == (1, 60)

    survivor = store.get(a) or store.get(b)
    assert survivor is not None and len(survivor.data) == 60
    assert store.get("ef" + "0" * 62) is None

    assert store.clear() == 1
    assert store.stats() == (0, 0)
