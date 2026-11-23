"""Re-render consumed scans at recorded physics poses. RNG offset 0 is the reset scan."""
from pathlib import Path
import json,sys
import numpy as np
import yaml
from f110_gym.envs.f110_env import F110Env
D=Path(__file__).resolve().parent
ROOT=D.parents[1]
def make_env(boxes):
    params=yaml.safe_load(Path('/home/misys/forza_ws/race_stack/stack_master/config/SIM/sim_params.yaml').read_text())
    env=F110Env(map=str(ROOT/'maps/berlin/berlin'),map_ext='.png',seed=12345,num_agents=1,params=params)
    car=env.sim.agents[0]
    type(car).static_obstacles=boxes
    return env,car
def install(env,z,buffer,index,offset):
    car=env.sim.agents[0]
    car.state=np.asarray(z,dtype=float);car.steer_buffer=np.asarray(buffer,dtype=float)
    env.sim.agent_poses[0]=car.state[[0,1,4]]
    car.in_collision=False;car.collision_kind=None
    car.scan_rng=np.random.default_rng(12345)
    car.scan_rng.normal(0.,.01,size=(index+offset+1)*1080)
def render(key,offset,beams):
    dat=json.loads((D/(key+'.json')).read_text())
    env,car=make_env(dat['boxes']);rng=np.random.default_rng(12345)
    ids=np.floor(np.arange(beams)*1079/(beams-1)+.5).astype(int)
    last=-1;out=[]
    for row in dat['controls']:
        idx=row['scan'];t,x,y,h=dat['physics'][idx]
        skip=idx-last-1+(offset if last==-1 else 0)
        rng.normal(0.,.01,size=skip*1080)
        car.state[[0,1,4]]=[x,y,h]
        ranges=car.scan_simulator.scan(np.array([x+.27*np.cos(h),y+.27*np.sin(h),h]),rng)
        out.append(car.ray_cast_static(ranges)[ids].astype(np.float32))
        last=idx
    np.stack(out).tofile(D/f'{key}-{offset}.bin')
if __name__=='__main__': render(sys.argv[1],int(sys.argv[2]),int(sys.argv[3]))
