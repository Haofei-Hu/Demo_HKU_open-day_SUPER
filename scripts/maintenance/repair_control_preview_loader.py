#!/usr/bin/env python3
import ast
import base64
import hashlib
from pathlib import Path
import shutil
import tempfile
import zlib

payload = """eNqtVllv4zYQfs+vIFCgohRZaPsYwECzbbdYoN0jWdRJDEOgpUkiRKJkkoolB/nvOzx0+MwWWL3YJGfm++YkU7gnecnSOCn5MwgF
glZMPfoXZwS/n8inPAVBUqjysoWUVAKeM1hLIhUTilx9uiZMkaJM6xyITMoKInIFLCUlz1vyAGUBSrSRMZYVVYk6TCqzVAKATPUy
qpiQQD8jrgWPBJqIFTSK+v6ZkU6RpyhLRRH3AZTjp7/1Y4bYmcw4cuIJOImQUG36ul7KRGQVrvXyUimRLWsF/siCYWOUkI/9Ez2z
vIZeQoCqBe/OspRk94cQNcJHVoBPIJdAPpYcBvYcDyTlZQoj6G07+tBaeV/zRGUl/xPud4g6Ki9aNtI2X980dill9sCP2BkHldyX
ootExomBsEv5SibkRTt0Gs7E/LJ+sJAOnnPH4BSFEZh/AMzJSsCSsDFFXAUFcCUxa3OtbuibP0hel1e0LNNW8xzFfmGUH5nsKl7X
IG/pm2lAqZT0USfTKfGcBW/Lq+Eb0xnY+kaaA6TYT1Py0lt51UzHxEwRvXhf4tkfXki8K/f7Jf7wTv9+jd998GyEbA98FTUMETat
WtY6OhYswvZsqX92mt52irZjR352pi72PHYQdZWiIZogn1T3icYwCw2iI7pm+ZM1diRoh6vL2Bg3mE7G7nmUqMbK/IMTzffHldrF
e9rHZduHJY6cJ1tXkEOiTG7mtlgOltXbPfzBzLudmmd5xiSYkmXGMOs7zQRaW6MsYtIUmT43+5Gs8kxRL/L8+S8L3+gYjxa76dK1
fEgHMUxTU6/AEavrh9dF1Xr+wMJx8/ezK1iGpfifnol/CVEK6v3tRjs25qrOBBJnnNRc1pV2GqNnp73n7/GzIPsYXdwjVlXAUzNJ
dMXGeZkw3YR0CCo1sZp2hEMTwFHCIT+dlPeiLH5MYk6m4pir2pSx7C5OtOZ5/nfm6+Jg4/yoDP3fTOhI0oOMrG/TkZ8h2UpbSHJ4
htxKmL9HEvn2BNplPAwYo1uxBMyojWO9jmPvgnjIQKIvcfdIcaO0W7qnyX3WxEWGkvyhd18a//81PlE9C6YdfkhUW0GMd12JAZ/O
F84VaCChSVlUOKZphxDi1HWvLcyuFvH8cCBsNbevhIvd+7CXnvfXyMK93K5LXafm+WZfQFjCPM84ECupnTcT7REyYV5yCcvzJUue
og5ZlyDumW4zdd4H04xEt0tP31DjW3+/Sj/bByXJEVd2pZoSl5whG8hZ97KS3uhBWNS5yqocB15IliMYvAUwupuQrHUW++2bkNyG
5C4kM9xe7oWyipgQrKXzdXBz3gSz8za4m2wCVFkHt5MmuMON2fkmuAmPX11rlGpQukUTm2CmVWeoeoMbt2jrbrH1nFW2m1bHqK+O
c5z/OvktoG3QIswGywYXTdAixLpbbJDu2l+cIDu3SudWyRhsgmYw2AYb5P49NjaTtgNGpfNm2yCy9Bdjz12p0iokqxBXz0xkek6P
4qDbKl5hDPocr8JxtZsSWwyDotLPnOHYVOKC/E5oNenjbG36uDuSNDU6MuQijdR65C1YbTd09PwxeeueU3cenn0DqtD9eA=="""
replacement = zlib.decompress(base64.b64decode(payload)).decode()
if hashlib.sha256(replacement.encode()).hexdigest() != '5771e7ade10f97decf03d5faa9d1a0124c46304eb3b1e1872dd056f0b760b25e':
    raise SystemExit('Repair checksum mismatch; no changes made.')
path = Path.home() / 'Downloads/control_bridge_preview.py'
source = path.read_text()
functions = [n for n in ast.parse(source).body
             if isinstance(n, ast.FunctionDef) and n.name == 'load_converter']
if len(functions) != 1:
    raise SystemExit('Expected exactly one load_converter; no changes made.')
node = functions[0]
lines = source.splitlines(keepends=True)
updated = ''.join(lines[:node.lineno-1]) + replacement + ''.join(lines[node.end_lineno:])
compile(updated, str(path), 'exec')
if source == updated:
    print('Repair already applied:', path)
else:
    backup = Path(tempfile.mkdtemp(prefix='control_preview_backup_', dir=str(path.parent))) / path.name
    shutil.copy2(path, backup)
    path.write_text(updated)
    print('Fixed:', path)
    print('Backup:', backup)
