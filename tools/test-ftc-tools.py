#!/usr/bin/env python3
"""Black-box desktop-tool tests. Never use a real adb, SDK, JDK or robot."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parent.parent
BASH = shutil.which('bash')
FAKE_ADB = r'''#!/usr/bin/env python3
import json, os, sys, time
from pathlib import Path
a = sys.argv[1:]
with open(os.environ['TEST_ADB_LOG'], 'a') as f:
    f.write(json.dumps(a) + '\n')
mode = os.environ.get('TEST_MODE', 'online')
state = Path(os.environ['TEST_STATE'])
if mode == 'hang': time.sleep(2)
if a == ['devices', '-l']:
    print('List of devices attached\nemulator-5554 device\nother-hub:5555 device')
elif a == ['version']: print('fixture adb')
elif a[:1] == ['connect']:
    if mode == 'connect' or (mode == 'restart' and state.exists()):
        state.write_text('online')
    print('connected (intentionally misleading when still offline)')
elif a == ['start-server']: state.write_text('started')
elif a == ['kill-server'] or a[:1] == ['disconnect']: pass
elif len(a) >= 3 and a[0] == '-s':
    if a[2] == 'get-state':
        online = mode == 'online' or (state.exists() and state.read_text() == 'online')
        print('device' if online else 'unauthorized' if mode == 'unauthorized' else 'offline')
    elif a[2:4] == ['install', '-r']:
        if os.environ.get('TEST_INSTALL_FAIL'): sys.exit(1)
        print('Success')
    elif a[2:] == ['shell', 'getprop', 'ro.product.model']: print('fixture hub')
    else: sys.exit(2)
else: sys.exit(2)
'''
FAKE_GRADLE = '''#!/usr/bin/env bash
printf '%s\\n' "$PWD" "$JAVA_HOME" "$@" > "$TEST_GRADLE_LOG"
if [ "${TEST_BUILD_FAIL:-0}" = 1 ]; then exit 7; fi
mkdir -p TeamCode/build/outputs/apk/debug
printf 'fresh APK' > TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
'''


class DesktopTools(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ftc tools ')
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        self.repo = self.folder / 'project with spaces'
        (self.repo / 'tools').mkdir(parents=True)
        for name in ('ftc.sh', 'ftc-common.sh'):
            shutil.copyfile(ROOT / 'tools' / name, self.repo / 'tools' / name)
        # A hermetic PATH makes discovery tests safe even on a developer laptop
        # which actually has adb installed or a robot connected.
        safe_bin = self.folder / 'bin'
        safe_bin.mkdir()
        for name in ('bash', 'python3', 'git', 'dirname', 'tr', 'cat', 'ls', 'date', 'zip', 'mkdir'):
            target = shutil.which(name)
            if target:
                (safe_bin / name).symlink_to(target)
        self.adb = self.folder / 'SDK with spaces/platform-tools/adb'
        self.executable(self.adb, FAKE_ADB)
        self.jdk = self.folder / 'JDK with spaces'
        self.executable(self.jdk / 'bin/javac', '#!/usr/bin/env bash\necho "javac 21.0.1"\n')
        self.executable(self.jdk / 'bin/java', '#!/usr/bin/env bash\necho "openjdk 21"\n')
        self.executable(self.repo / 'gradlew', FAKE_GRADLE)
        self.log = self.folder / 'adb.jsonl'
        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith(('FTC_', 'TEST_', 'ANDROID_', 'GIT_')) and k != 'JAVA_HOME'}
        self.env.update(PATH=str(safe_bin), HOME=str(self.folder),
                        GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1',
                        FTC_ADB=str(self.adb), FTC_JAVA_HOME=str(self.jdk),
                        ANDROID_HOME=str(self.adb.parent.parent),
                        TEST_ADB_LOG=str(self.log), TEST_STATE=str(self.folder / 'state'),
                        TEST_GRADLE_LOG=str(self.folder / 'gradle.txt'))

    def executable(self, path, text):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        path.chmod(0o755)

    def run_tool(self, *args, ok=True, **env):
        result = subprocess.run([BASH, str(self.repo / 'tools/ftc.sh'), *args],
                                cwd=self.folder, env=dict(self.env, **env),
                                text=True, capture_output=True, timeout=15)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_online_does_not_connect_or_restart(self):
        self.run_tool('connect')
        self.assertEqual(self.calls(), [['devices', '-l'], ['-s', '192.168.43.1:5555', 'get-state']])

    def test_normal_connect_before_restart(self):
        self.run_tool('connect', TEST_MODE='connect')
        self.assertIn(['connect', '192.168.43.1:5555'], self.calls())
        self.assertNotIn(['kill-server'], self.calls())

    def test_restart_once_then_reconnect(self):
        self.run_tool('connect', '10.0.0.2:5555', TEST_MODE='restart', FTC_ROBOT_SERIAL='ignored:5555')
        calls = self.calls()
        self.assertEqual(calls.count(['kill-server']), 1)
        self.assertEqual(calls.count(['connect', '10.0.0.2:5555']), 2)
        self.assertLess(calls.index(['disconnect', '10.0.0.2:5555']), calls.index(['kill-server']))
        self.assertLess(calls.index(['kill-server']), calls.index(['start-server']))
        self.assertTrue(all(c[1] == '10.0.0.2:5555' for c in calls if c[0] == '-s'))

    def test_offline_is_failure_despite_connect_success_message(self):
        self.run_tool('connect', ok=False, TEST_MODE='offline')
        self.assertEqual(self.calls().count(['kill-server']), 1)

    def test_server_restart_opt_out(self):
        self.run_tool('connect', ok=False, TEST_MODE='offline', FTC_ADB_RESTART='0')
        self.assertNotIn(['kill-server'], self.calls())

    def test_unauthorized_stops_without_reset(self):
        self.run_tool('connect', ok=False, TEST_MODE='unauthorized')
        self.assertNotIn(['kill-server'], self.calls())

    def test_usb_serial_does_not_become_tcp_connect(self):
        self.run_tool('connect', 'USB123', ok=False, TEST_MODE='offline')
        self.assertFalse(any(c[0] in ('connect', 'disconnect') for c in self.calls()))

    def test_sdk_and_mac_default_adb_discovery(self):
        self.env.pop('FTC_ADB')
        self.run_tool('connect')  # ANDROID_HOME discovery
        self.env.pop('ANDROID_HOME')
        mac_adb = self.folder / 'Library/Android/sdk/platform-tools/adb'
        self.executable(mac_adb, FAKE_ADB)
        self.run_tool('connect')  # default macOS path, without aliases

    def test_bad_override_is_not_silently_ignored(self):
        self.run_tool('connect', ok=False, FTC_ADB='/nonexistent/adb')
        self.assertEqual(self.calls(), [])

    def test_timeout_is_bounded(self):
        start = time.monotonic()
        result = self.run_tool('adb', 'shell', 'getprop', 'ro.product.model',
                               ok=False, FTC_ADB_TIMEOUT='0.1', TEST_MODE='hang')
        self.assertEqual(result.returncode, 124)
        self.assertLess(time.monotonic() - start, 5)

    def test_build_jdk_sdk_paths_and_offline_flag(self):
        self.run_tool('build', FTC_OFFLINE='1')
        args = (self.folder / 'gradle.txt').read_text().splitlines()
        self.assertEqual(args[:2], [str(self.repo), str(self.jdk)])
        self.assertIn('-Dorg.gradle.java.home=' + str(self.jdk), args)
        self.assertIn('--offline', args)
        self.assertIn(':TeamCode:assembleDebug', args)
        self.assertEqual(self.calls(), [])

    def test_java_tool_options_banner_does_not_break_jdk_check(self):
        self.executable(self.jdk / 'bin/javac', '#!/usr/bin/env bash\n'
                        'echo "Picked up JAVA_TOOL_OPTIONS: -Dx=1" >&2\necho "javac 17.0.9"\n')
        self.run_tool('build')
        self.assertTrue((self.folder / 'gradle.txt').exists())

    def test_invalid_jdk_stops_build(self):
        self.run_tool('build', ok=False, FTC_JAVA_HOME=str(self.folder / 'missing jdk'))
        self.assertFalse((self.folder / 'gradle.txt').exists())

    def test_deploy_builds_and_selects_one_device(self):
        self.run_tool('deploy', FTC_ROBOT_SERIAL='10.1.2.3:5555')
        installs = [c for c in self.calls() if 'install' in c]
        self.assertEqual(installs, [['-s', '10.1.2.3:5555', 'install', '-r',
                                    str(self.repo / 'TeamCode/build/outputs/apk/debug/TeamCode-debug.apk')]])

    def test_failed_build_never_installs_stale_apk(self):
        self.run_tool('build')  # an old APK exists
        self.run_tool('deploy', ok=False, TEST_BUILD_FAIL='1')
        self.assertEqual(self.calls(), [])

    def test_install_missing_apk_or_emulator_fails_before_adb(self):
        self.run_tool('install', ok=False)
        self.run_tool('deploy', 'emulator-5554', ok=False)
        self.assertEqual(self.calls(), [])

    def test_install_existing_no_build_and_failure_no_uninstall(self):
        self.run_tool('build')
        (self.folder / 'gradle.txt').unlink()
        self.run_tool('install', ok=False, TEST_INSTALL_FAIL='1')
        self.assertFalse((self.folder / 'gradle.txt').exists())
        self.assertFalse(any('uninstall' in c for c in self.calls()))

    def test_targeted_adb_and_invalid_arguments(self):
        self.run_tool('adb', 'shell', 'getprop', 'ro.product.model', FTC_ROBOT_SERIAL='chosen:5555')
        self.assertEqual(self.calls(), [['-s', 'chosen:5555', 'shell', 'getprop', 'ro.product.model']])
        self.run_tool('adb', '-s', 'other', 'shell', ok=False)
        self.run_tool('connect', '-bad', ok=False)
        self.run_tool('build', 'unexpected', ok=False)
        self.run_tool('unknown', ok=False)

    def git(self, *args):
        return subprocess.check_output([shutil.which('git'), '-C', str(self.repo), *args],
                                       env=self.env, text=True, stderr=subprocess.STDOUT).strip()

    def init_repo(self):
        self.git('init', '-b', 'pollen-hive-ready')
        self.git('config', 'user.name', 'Fixture')
        self.git('config', 'user.email', 'fixture@example.invalid')
        self.git('add', '.')
        self.git('commit', '-m', 'Fixture base')

    def test_doctor_reports_but_never_restarts(self):
        self.init_repo()
        result = self.run_tool('doctor', TEST_MODE='offline')
        self.assertIn('pollen-hive-ready', result.stdout)
        self.assertEqual(self.calls(), [['version'], ['devices', '-l']])

    def test_patch_guards_backup_and_mail_apply(self):
        self.init_repo()
        base = self.git('rev-parse', 'HEAD')
        (self.repo / 'change.txt').write_text('a change\n')
        self.git('add', 'change.txt')
        self.git('commit', '-m', 'Test mail patch')
        patch = self.folder / 'change.patch'
        patch.write_text(self.git('format-patch', '-1', '--stdout') + '\n')
        self.git('reset', '--hard', base)  # Only this throwaway fixture repo.
        (self.repo / 'local-work.txt').write_text('keep me\n')
        self.run_tool('patch', str(patch), ok=False)
        self.assertEqual(self.git('rev-parse', 'HEAD'), base)
        (self.repo / 'local-work.txt').unlink()
        self.git('switch', '-c', 'wrong-branch')
        self.run_tool('patch', str(patch), ok=False)
        self.git('switch', 'pollen-hive-ready')
        plain = self.folder / 'change.diff'  # Same change without mail headers, as `git diff` writes it.
        body = patch.read_text()
        plain.write_text(body[body.index('diff --git'):body.rindex('\n-- \n') + 1])
        self.run_tool('patch', str(plain), ok=False)  # Plain diff: refuse before any backup branch.
        self.assertEqual(self.git('for-each-ref', 'refs/heads/backup/'), '')
        self.assertEqual(self.git('rev-parse', 'HEAD'), base)
        self.run_tool('patch', str(patch))
        self.assertEqual((self.repo / 'change.txt').read_text(), 'a change\n')
        backup = self.git('for-each-ref', '--format=%(objectname)', 'refs/heads/backup/')
        self.assertEqual(backup, base)
        self.assertEqual(self.git('status', '--porcelain'), '')
        self.assertEqual(self.calls(), [])
        self.run_tool('patch', str(patch), ok=False)  # Already applied: do not double-commit.

    def test_storage_and_prune_select_the_hub_and_guard_arguments(self):
        # The fake adb answers no shell questions, so this checks routing and guards only;
        # tools/test-hive-storage.py covers the report and the checksum comparison.
        shutil.copyfile(ROOT / 'tools/hive-storage.py', self.repo / 'tools/hive-storage.py')
        result = self.run_tool('storage')
        self.assertIn('could not read df', result.stdout)
        shells = [c for c in self.calls() if c[2:3] == ['shell']]
        self.assertTrue(shells and all(c[:2] == ['-s', '192.168.43.1:5555'] for c in shells))
        self.assertFalse(any(c[3].startswith('rm ') for c in shells))
        self.log.unlink()
        self.run_tool('storage', ok=False, TEST_MODE='offline', FTC_ADB_RESTART='0')
        self.run_tool('prune', ok=False)
        self.run_tool('prune', str(self.folder / 'missing'), '--delete', ok=False)
        downloaded = self.folder / 'session'
        (downloaded / 'pollen-hive/teleop').mkdir(parents=True)
        self.run_tool('prune', str(downloaded), '--force', ok=False)
        self.assertFalse(any(c[2:3] == ['shell'] for c in self.calls()))
        result = self.run_tool('prune', str(downloaded), '--delete')
        self.assertIn('Nothing was removed', result.stdout)
        self.assertFalse(any(c[2:3] == ['shell'] and c[3].startswith('rm ') for c in self.calls()))


if __name__ == '__main__':
    unittest.main(verbosity=2)
