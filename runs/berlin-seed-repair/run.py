from pathlib import Path
import os
import runpy

root = Path(__file__).resolve().parents[2]
generate = runpy.run_path(str(root / 'runs/berlin-forward-holdout/random_layouts.py'))['layouts']
catalog = generate((39103, 39104, 39201, 39202, 39203, 39204, 39301, 39302))
runpy.run_path(str(root / 'runs/berlin-brake-merge/ros.py'))['main'](
    catalog, domain_base=int(os.environ.get('EL_TRIAL_DOMAIN_BASE', '140')))
