# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_all

datas = [('swarm_tasks\\envs\\worlds', 'swarm_tasks\\envs\\worlds')]
binaries = []
hiddenimports = ['swarm_tasks.utils', 'swarm_tasks.envs', 'swarm_tasks.simulation.simulation', 'swarm_tasks.controllers.command', 'swarm_tasks.controllers', 'swarm_tasks.controllers.base_control', 'swarm_tasks.modules.dispersion', 'swarm_tasks.modules.exploration', 'swarm_tasks.modules.formations.line', 'swarm_tasks.modules.formations.circle', 'swarm_tasks.utils.weight_functions', 'locatePosition', 'geopy.distance', 'geopy.point', 'simplekml', 'matplotlib.pyplot', 'numpy', 'mavproxy', 'lxml', 'bezier_curve', 'groupsplitauto', 'bezier_curve_multiple', 'groupsplitspecific', 'netifaces']
tmp_ret = collect_all('dronekit')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('pymavlink')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('lxml')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]

a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=[],
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
    name='main',
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
    name='main',
)
