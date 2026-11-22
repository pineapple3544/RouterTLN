from pathlib import Path
import subprocess,os,sys,csv,json
from cases import levels
p=Path(__file__).resolve().parent
model,tag=sys.argv[1:3]
bands=[tuple(map(float,a.split(','))) for a in sys.argv[3:]] or levels[1:]
for lo,hi in bands:
    name=tag+f'-{lo:.1f}'
    env=dict(os.environ,ROS_DOMAIN_ID='158',EL_LAPS='3',EL_LAP_PLANNING='true',EL_SPEED_CAP=str(max(3.0,hi)),
             EL_AVOID_CHECKPOINT=model,EL_SIM_AVOID_MIN=str(float(lo)),EL_SIM_AVOID_MAX=str(float(hi)))
    with (p/(name+'.log')).open('x') as log:
        (p/(name+'-settings.json')).write_text(json.dumps(dict(low=lo,high=hi,checkpoint=model),indent=2))
        # Independent witness for contact classification on the ceiling series.
        witness=None
        with (p/(name+'-contact.log')).open('w') as contact:
            if tag.startswith(('ceiling','impact-ros-confirm')):
                witness=subprocess.Popen(['bash','-c',
                    'source /opt/ros/humble/setup.bash && exec ros2 topic echo --once --filter "m.data != \'none\'" /sim/collision std_msgs/msg/String'],
                    env=dict(env,PYTHONUNBUFFERED='1'),stdout=contact,stderr=contact)
            try:
                res=subprocess.run(['bash','scripts/berlin_router_lap.sh',p.name+'/'+name],env=env,stdout=log,stderr=subprocess.STDOUT)
            finally:
                if witness is not None:
                    witness.terminate()
                    witness.wait()
    with (p/(name+'.csv')).open() as stream:rows=list(csv.DictReader(stream))
    reject=next((r for r in rows if r['mode']=='3'),None)
    print(name,res.returncode,(reject['reason'],reject['speed'],reject['progress_m']) if reject else rows[-1]['progress_m'],flush=True)
