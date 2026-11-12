#!/usr/bin/env python3
"""Check tools/hive-promote-defaults.py on a scratch copy of PollenHiveSettings.java."""
import pathlib, shutil, subprocess, sys, tempfile
root = pathlib.Path(__file__).resolve().parent.parent
java = root / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenHiveSettings.java'
with tempfile.TemporaryDirectory() as d:
    d = pathlib.Path(d)
    copy = d / 'PollenHiveSettings.java'; shutil.copy(java, copy)
    hub = d / 'pollen-hive'; hub.mkdir()
    (hub / 'camera.properties').write_text('#Pollen hive camera\n#Wed Nov 11 20:00:00 PST 2026\ntilt=41.8\nsideways=-0.6\n')
    (hub / 'primary.properties').write_text('launcher=1200.0\nexecution=0.0\nnoSuchRow=3\n')
    (hub / 'secondary.properties').write_text('park3Left=-110.5\n')
    run = lambda *a: subprocess.run([sys.executable, str(root / 'tools/hive-promote-defaults.py'), str(hub), '--java', str(copy), *a],
                                    capture_output=True, text=True, check=True).stdout
    shown = run(); assert copy.read_text() == java.read_text(), 'show-only must not write'
    run('--write'); src = copy.read_text()
    for want in ['row("tilt", "Camera tilt up deg", 41.8, 41.8,', 'row("sideways", "Camera sideways correction in", -0.6, -0.6,',
                 'row("launcher", "Launcher ticks/s", 1200, 1180,', 'row("park3Left", "Park3 left cm", 109.8762319108, -110.5,',
                 'row("execution", "Execution", 2, 2,']:
        assert want in src, want
    assert 'SKIP primary.noSuchRow' in shown, shown
    assert 'Nothing to promote' in run(), 'second run finds nothing'
print('PASS hive-promote-defaults: Hub changes become code defaults (execution never promoted)')
