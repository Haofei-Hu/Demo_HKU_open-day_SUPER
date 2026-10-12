"""Real tmux lifecycle with dummy nodes only; no ROS graph or aircraft."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import argparse
import types
import time
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('session',ROOT/'scripts/maintenance/super_session.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def wait(predicate):
    end=time.monotonic()+8
    while not predicate():
        assert time.monotonic()<end,'tmux fixture timeout'
        time.sleep(.05)


class Ground:
    def __init__(self): self.node=argparse.Namespace(count_publishers=lambda topic:0)
    def localization_ready(self): return True
    def check(self): pass
    def grounded(self): return True
    def wait(self,predicate,description): assert predicate(),description
    def close(self): pass


with tempfile.TemporaryDirectory(prefix='tmux smoke ') as temporary:
    folder=Path(temporary);scripts=folder/'Downloads';scripts.mkdir()
    ledger=folder/'calls.jsonl';ledger.write_text('')
    prefix=folder/'prefix';prefix.mkdir()
    binpath=folder/'bin';binpath.mkdir()
    dummy='''#!/usr/bin/env python3
import json,os,sys,time
from pathlib import Path
with Path(os.environ['FIXTURE_LEDGER']).open('a') as f: f.write(json.dumps(sys.argv)+'\\n')
if sys.argv[1:]==['pkg','prefix','fast_lio']:
    print(os.environ['FIXTURE_PREFIX']);sys.exit()
print('DUMMY NODE READY',flush=True)
try:
    while True: time.sleep(.05)
except KeyboardInterrupt: print('DUMMY NODE STOPPED',flush=True)
'''
    (binpath/'ros2').write_text(dummy);(binpath/'ros2').chmod(0o755)
    for name in m.SCRIPTS: (scripts/name).write_text(dummy)
    setups=[]
    for index in range(4):
        setup=folder/f'setup{index}.bash'
        setup.write_text('export PATH='+m.shlex.quote(str(binpath))+':"$PATH"\n'
            +'export FIXTURE_LEDGER='+m.shlex.quote(str(ledger))+'\n'
            +'export FIXTURE_PREFIX='+m.shlex.quote(str(prefix))+'\n')
        setups.append(str(setup))
    args=m.arguments(['start','--session','test-'+str(os.getpid()),'--scripts-dir',str(scripts)])
    session=m.Session(args);session.folder=folder/'state';session.metadata=session.folder/'session.json'
    with patch.object(m,'SETUPS',setups):
        try:
            session.start()
            wait(lambda:len(ledger.read_text().splitlines())==5)
            wait(lambda:all('DUMMY NODE READY' in (Path(session.config['logs'])/(name+'.log')).read_text() for name in m.CORE))
            assert session.windows()==dict.fromkeys(m.CORE,'0')
            before=ledger.read_text();session.start();time.sleep(.1)
            assert ledger.read_text()==before,'start silently duplicated nodes'
            calls=[json.loads(line) for line in before.splitlines()]
            lio=next(call for call in calls if 'fastlio_mapping' in call)
            assert lio[lio.index('--params-file')+1]==str(prefix)+'/share/fast_lio/config/mid360.yaml'
            session.launch('preview',m.shell_command(['python3','-u',scripts/'aligned_pose_preview.py']))
            session.launch('vision',m.shell_command(['python3','-u',scripts/'flight_pose_cov_transport.py']))
            wait(lambda:len(ledger.read_text().splitlines())==7)
            package=types.ModuleType('mars_quadrotor_msgs')
            messages=types.ModuleType('mars_quadrotor_msgs.msg');messages.PositionCommand=lambda:object()
            with patch.object(m,'GroundGate',Ground), patch.dict('sys.modules',
                    {'mars_quadrotor_msgs':package,'mars_quadrotor_msgs.msg':messages}), \
                    patch.object(session,'planner_profile',return_value=Path('/tmp/mid360_ground_diagnostic_ros2.yaml')):
                session.mutate('planner')
                wait(lambda:len(ledger.read_text().splitlines())==9)
                assert session.windows()==dict.fromkeys((*m.CORE,'preview','vision','planner','cmd_preview'),'0')
                session.mutate('planner-stop')
                assert session.windows()==dict.fromkeys((*m.CORE,'preview','vision'),'0'),'planning stop interrupted localization'
                session.mutate('stop')
            assert not session.exists()
            assert not m.unmanaged_processes(),'dummy node leaked after cooperative stop'
            print('PASS: real tmux core windows, shell/path quoting, startup logs, idempotent start, ground planning windows/local stop, cooperative shutdown, no leaked nodes.')
        finally:
            if session.exists(): session.tmux('kill-session','-t',session.name)
