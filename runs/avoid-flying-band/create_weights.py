"""CPU-only, explicit weight edits to schema-4 Avoid. No optimizer."""
from pathlib import Path
import torch
root=Path(__file__).resolve().parent
source=root.parents[1]/'models/avoid-corridor-v2.pt'
variants=[('steer-zero',0.),('surgery',0.),('preview2',2.),('preview6',6.),('damp3',3.)]
for kind in ('damp','preview','lead'):
    variants.extend((kind+str(g),g) for g in (.25,.75,-.5,-1.5))
for kind in ('damp','preview','lead'):
    variants.extend(('high-'+kind+str(g),g) for g in (-1.,.75,3.))
for name,gain in variants:
    kind=name.removeprefix('high-')
    if (root/(name+'.pt')).exists():continue
    m=torch.jit.load(str(source),map_location='cpu')
    t=m.state_dict()
    a,b,c,d,e,f=[t[k] for k in ['head.0.weight','head.0.bias','head.2.weight','head.2.bias','head.4.weight','head.4.bias']]
    with torch.no_grad():
        if name=='steer-zero':
            e[0].zero_();f[0].zero_()
        else:
            # Reuse weak outgoing channels. The surgery control makes the
            # same disconnections and leaves the additive steering branch at zero.
            u=c.norm(dim=0).argsort()[:3].tolist()
            v=e.norm(dim=0).argsort()[:4].tolist()
            c[:,u]=0.; e[:,v]=0.
            a[u]=0.; b[u]=0.; c[v]=0.; d[v]=0.
            # Signed correction: 1 m corridor preview relative to the short
            # reference guide, or damping based on measured yaw rate.
            if kind.startswith('damp'):
                a[u[0],1025]=-1.;a[u[1],1025]=1.
            elif kind.startswith('lead'):
                # Phase lead: anticipate reversal when reference and actuator disagree.
                a[u[0],1028]=1.;a[u[0],1026]=-1.
                a[u[1],1028]=-1.;a[u[1],1026]=1.
            else:
                a[u[0],1030]=16.5;a[u[0],1028]=-1.
                a[u[1],1030]=-16.5;a[u[1],1028]=1.
            a[u[2],1024]=8.;b[u[2]]=-(3.3 if name.startswith('high-') else 2.2)
            # clip signed correction to +/- max(0, speed - 2.2 m/s).
            for i in range(4):c[v[i],u[i//2]]=1.
            c[v[1],u[2]]=-1.;c[v[3],u[2]]=-1.
            e[0,v]=torch.tensor([gain,-gain,-gain,gain])
    target=root/(name+'.pt')
    if target.exists():raise FileExistsError(target)
    torch.jit.save(m,str(target))
