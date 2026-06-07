"""PPM Modbus desktop controller entrypoint."""

from __future__ import annotations

import os
from pathlib import Path
import sys


def _ensure_repo_venv() -> None:
    repo_root = Path(__file__).resolve().parent
    scripts_dir = repo_root / ".venv" / "Scripts"
    exe_name = "pythonw.exe" if Path(sys.executable).name.lower() == "pythonw.exe" else "python.exe"
    venv_python = scripts_dir / exe_name
    if not venv_python.exists():
        return

    current_python = Path(sys.executable).resolve()
    target_python = venv_python.resolve()
    if current_python == target_python:
        return

    os.execv(str(target_python), [str(target_python), *sys.argv])


def main() -> None:
    from modbus_app.app import main as app_main

    app_main()


if __name__ == "__main__":
    _ensure_repo_venv()
    main()
