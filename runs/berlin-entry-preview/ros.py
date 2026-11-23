from pathlib import Path
import os,runpy
root=Path(__file__).resolve().parents[2]
layouts=runpy.run_path(str(root/'runs/berlin-forward-holdout/random_layouts.py'))['layouts']((39204,39401,39402,39403,39404))
runpy.run_path(str(root/'runs/berlin-brake-merge/ros.py'))['main'](layouts,domain_base=int(os.environ.get('EL_TRIAL_DOMAIN_BASE','140')))
