# -*- mode: python ; coding: utf-8 -*-
#
# PyInstaller spec for the Skybrush server (skybrushd.exe).
#
# Build it from an environment that has the server and PyInstaller installed
# (the .venv-build one made with `uv sync --frozen` + `uv pip install
# pyinstaller`; do not use the .venv you run the server from):
#
#   cd gui/skybrush-server
#   .venv-build/Scripts/python.exe -m PyInstaller skybrushd.spec --noconfirm \
#       --distpath ../../release --workpath ../../build/skybrush-server
#
# The server loads its extensions by name at run time, and its dependencies
# bring data files (schemas, the web UI, magnetic-model data...), so every
# installed package is collected whole instead of being guessed at.

from importlib.metadata import distributions

from PyInstaller.utils.hooks import collect_all, copy_metadata

#: Installed packages that only exist for building or testing.
SKIP = {
    "pyinstaller",
    "pyinstaller-hooks-contrib",
    "altgraph",
    "pefile",
    "pywin32-ctypes",
    "packaging",
    "setuptools",
    "pip",
    "wheel",
}

datas, binaries, hiddenimports = [], [], []
seen_top_level = set()

for dist in distributions():
    name = (dist.metadata["Name"] or "").lower()
    if not name or name in SKIP or name.startswith(("pytest", "ruff", "ty", "mypy")):
        continue

    datas += copy_metadata(dist.metadata["Name"])

    top_level = (dist.read_text("top_level.txt") or "").split()
    if not top_level:
        for entry in dist.files or []:
            first = entry.parts[0]
            if first.endswith((".dist-info", ".data")) or first in ("..", "__pycache__"):
                continue
            top_level.append(first.split(".")[0])

    for package in sorted(set(top_level)):
        if package in seen_top_level or package.startswith("_") or package == "tests":
            continue
        seen_top_level.add(package)
        try:
            d, b, h = collect_all(package)
        except Exception as exc:  # a distribution with no importable package
            print(f"skybrushd.spec: skipping {package}: {exc!r}")
            continue
        datas += d
        binaries += b
        hiddenimports += h

a = Analysis(
    ["src/flockwave/server/__main__.py"],
    pathex=["src"],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["tkinter", "pytest", "ruff", "mypy"],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="skybrushd",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="skybrush-server",
)
