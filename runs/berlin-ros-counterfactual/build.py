"""Build the offline diagnostic executable without touching the ROS controller or evaluator."""
from pathlib import Path
import subprocess,shlex
D=Path(__file__).resolve().parent
R=D.parents[1]
router=(R/'include/el/section_router.hpp').read_text()
router=router.replace('namespace el {','''#include <functional>
namespace el {
inline std::function<float(float,float)> study_steering;
inline bool study_direct=false;''',1)
router=router.replace('  bool avoiding() const', '''  void follow_record(const State& state, Control sent) {
    stop_reason_=nullptr; last_pose_=state; have_pose_=true; last_command_=sent;
  }
  bool avoiding() const''')
line='    if(trace)trace->policy_steer=cmd.steer;'
router=router.replace(line,line+'\n    if(avoiding_ && study_steering)cmd.steer=study_steering(cmd.steer,cmd.speed);')
router=router.replace('cmd.steer=corridor_.preview_steer','if(!study_direct)cmd.steer=corridor_.preview_steer')
(D/'study_router.hpp').write_text(router)
cmds=subprocess.check_output(['ninja','-C','/build','-t','commands','el_eval_avoid'],text=True).splitlines()
compile=shlex.split(next(l for l in cmds if ' -c /workspace/src/eval_avoid.cpp' in l))
cmd=compile[:compile.index('-MD')]+['-I'+str(R/'src'),'-c',str(D/'replay.cpp'),'-o',str(D/'replay.o')]
subprocess.run(cmd,cwd='/build',check=True)
link=shlex.split(cmds[-1].split('&&')[1].strip());link[link.index('CMakeFiles/el_eval_avoid.dir/src/eval_avoid.cpp.o')]=str(D/'replay.o')
link[link.index('-o')+1]=str(D/'replay')
subprocess.run(link,cwd='/build',check=True)
