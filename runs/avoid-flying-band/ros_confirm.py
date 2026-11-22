from pathlib import Path
import subprocess,sys
p=Path('runs/avoid-flying-band')
original='models/avoid-corridor-v2.pt'
candidate=str(p/'damp0.75.pt')
runs=[(original,'extended',['3.0,3.3','3.3,3.6','3.6,3.9','3.9,4.2','4.2,4.5','4.5,4.8']),
      (candidate,'wsp',['2.7,3.0','3.0,3.3','3.3,3.6']),
      (original,'repeat',['1.5,1.9','2.7,3.0','3.0,3.3']),
      (candidate,'wsp-repeat',['2.7,3.0','3.0,3.3'])]
for model,tag,bands in runs:
 subprocess.run([sys.executable,str(p/'ros.py'),model,tag,*bands],check=True)
