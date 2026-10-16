#!/usr/bin/env python3
"""Open or reuse the onboard QGroundControl application; no vehicle commands."""
import os
from pathlib import Path
import shutil
import subprocess


_launched=[]  # Keep the detached application's handle; the runner never closes QGC.


def running_qgc():
    """Inspect same-user process names and executable arguments without pgrep self-matches."""
    for folder in Path('/proc').iterdir():
        if not folder.name.isdigit():continue
        try:
            if folder.stat().st_uid!=os.getuid():continue
            names=[(folder/'comm').read_text().strip()]
            names.extend(os.fsdecode(a) for a in (folder/'cmdline').read_bytes().split(b'\0')[:2])
            try:names.append(os.readlink(folder/'exe'))
            except OSError:pass
            if any(Path(name).name.lower().startswith('qgroundcontrol') for name in names):
                return int(folder.name)
        except (OSError,ValueError):continue
    return None


def open_qgc(folder,app_path=None,enabled=True):
    """Return launch evidence independently of GUI rendering or MAVLink readiness."""
    status=dict(status='disabled',connection_verified=False,rendering_verified=False)
    if not enabled:return status
    pid=running_qgc()
    if pid is not None:
        status.update(status='already_running',pid=pid)
        print('QGC: already running; reusing PID '+str(pid),flush=True);return status
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        status.update(status='skipped',reason='No graphical desktop is available')
        print('QGC: skipped; '+status['reason'],flush=True);return status
    candidates=([Path(app_path).expanduser()] if app_path is not None else
        [Path.home()/'Downloads'/name for name in ('QGroundControl-x86_64.AppImage','QGroundControl.AppImage')])
    if app_path is None:
        candidates.extend(Path(path) for name in ('QGroundControl','qgroundcontrol')
                          if (path:=shutil.which(name)))
    app=next((p for p in candidates if p.is_file() and os.access(p,os.X_OK)),None)
    if app is None:
        status.update(status='unavailable',reason='No executable QGroundControl found; use --qgc-path for its installed location')
        print('QGC: unavailable; '+status['reason'],flush=True);return status
    app=app.resolve()
    log=Path(folder)/'qgc.log';status.update(application=str(app),log=str(log))
    try:
        log.parent.mkdir(parents=True,exist_ok=True)
        with log.open('a') as handle:
            process=subprocess.Popen([str(app)],stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        _launched.append(process)
        code=process.poll()
        if code is None:status.update(status='launched',pid=process.pid)
        else:status.update(status='unavailable',returncode=code,reason='QGroundControl exited during launch')
    except OSError as exc:status.update(status='unavailable',reason=str(exc))
    print('QGC: '+status['status']+'; log: '+str(log),flush=True)
    return status
