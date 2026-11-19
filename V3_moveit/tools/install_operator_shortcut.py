"""Install a GNOME menu entry and a local launcher for this workspace."""
import os
import tempfile
from pathlib import Path
from gi.repository import GLib

root = Path(__file__).resolve().parents[1]
launcher = str(root / 'START_OPERATOR.sh').replace('%', '%%')
for char in ('\\', '"', '$', '`'):
    launcher = launcher.replace(char, '\\' + char)

entry = GLib.KeyFile()
for key, value in {
    'Type': 'Application', 'Name': 'V3 遥操作工作台', 'Comment': 'JAKA S5 · Quest · DH116',
    'Exec': f'/usr/bin/bash "{launcher}"', 'Path': str(root),
    'Icon': str(root / 'tools/operator_icon.svg'), 'Terminal': 'false',
    'Categories': 'Science;Robotics;', 'StartupWMClass': 'operator_panel.py',
}.items():
    entry.set_string('Desktop Entry', key, value)
content, _ = entry.to_data()
data_home = Path(os.environ.get('XDG_DATA_HOME') or Path.home() / '.local/share')
for target in (root / 'V3-Operator.desktop', data_home / 'applications/v3-operator-3.1.desktop'):
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=target.parent, delete=False) as out:
        out.write(content)
    os.chmod(out.name, 0o755)
    os.replace(out.name, target)
