from pathlib import Path
import subprocess,shlex,sys,shutil
D=Path(__file__).resolve().parent
version=sys.argv[1] if len(sys.argv)>1 else "v84"
if version not in ("v84","v87"):raise ValueError(version)
headers=D/"old"
if version=="v87":
    headers=D/"inc87"
    (headers/"el").mkdir(exist_ok=True,parents=True)
    shutil.copy2(D/"v87.hpp",headers/"el/avoid_corridor.hpp")
    # section_router.hpp includes its neighbours with the el/ prefix.
    shutil.copy2(D/"router-before.hpp",headers/"el/section_router.hpp")
cmds=subprocess.check_output(["ninja","-C","/build","-t","commands","el_eval_avoid"],text=True).splitlines()
args=shlex.split(next(l for l in cmds if " -c /workspace/src/eval_avoid.cpp" in l))
subprocess.run(args[:1]+["-I"+str(headers)]+args[1:args.index("-MD")]+["-c","/workspace/src/eval_avoid.cpp","-o",str(D/"old.o")],cwd="/build",check=True)
args=shlex.split(cmds[-1].split("&&")[1]);args[args.index("CMakeFiles/el_eval_avoid.dir/src/eval_avoid.cpp.o")]=str(D/"old.o")
args[args.index("-o")+1]=str(D/(version+"-eval"))
subprocess.run(args,cwd="/build",check=True)
