# -*- mode: python ; coding: utf-8 -*-
import os
import sys

import setuptools
from PyInstaller.utils.hooks import collect_all, collect_submodules

# pkg_resources (pulled in by dronekit/pymavlink) imports jaraco.* from the
# private setuptools/_vendor folder, which PyInstaller 6.7 does not bundle on
# its own: without this the exe stops at start-up with "No module named
# 'jaraco'". Put that folder on the search path and bundle what is there.
vendor_dir = os.path.join(os.path.dirname(setuptools.__file__), '_vendor')
sys.path.insert(0, vendor_dir)

datas = [('swarm_tasks\\envs\\worlds', 'swarm_tasks\\envs\\worlds')]
binaries = []
hiddenimports = ['swarm_tasks.utils', 'swarm_tasks.envs', 'swarm_tasks.simulation.simulation', 'swarm_tasks.controllers.command', 'swarm_tasks.controllers', 'swarm_tasks.controllers.base_control', 'swarm_tasks.modules.dispersion', 'swarm_tasks.modules.exploration', 'swarm_tasks.modules.formations.line', 'swarm_tasks.modules.formations.circle', 'swarm_tasks.utils.weight_functions', 'locatePosition', 'geopy.distance', 'geopy.point', 'simplekml', 'matplotlib.pyplot', 'numpy', 'mavproxy', 'lxml', 'bezier_curve', 'groupsplitauto', 'bezier_curve_multiple', 'groupsplitspecific', 'netifaces']
tmp_ret = collect_all('dronekit')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('pymavlink')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('lxml')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
for vendored in ('jaraco', 'autocommand', 'inflect', 'more_itertools', 'typeguard', 'zipp', 'backports'):
    try:
        hiddenimports += collect_submodules(vendored)
    except Exception:
        pass

a = Analysis(
    ['main.py'],
    pathex=[vendor_dir],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='swarm-main',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
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
    upx=True,
    upx_exclude=[],
    name='swarm-main',
)
