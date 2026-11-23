from prepare import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(3,1,figsize=(9,7.4),constrained_layout=True)
r=np.genfromtxt(D/'spatial.csv',delimiter=',',names=True)
ax=axs[0]
ax.plot(r['s'],100*(r['fail_d']-r['pass_d']),color='#265b9b',lw=1.3)
ax.axhline(0,color='0.7',lw=.7);ax.axvline(37.59,color='#a23626',ls=':',label='Third Avoid entry')
ax.set(xlabel='Distance from start (m)',ylabel='Lateral difference (cm)',title='9006203  |  Recorded failure minus recorded success')
ax.legend(loc='lower left')
ax=axs[1]
raw=json.loads((D/'failed.json').read_text())['controls']
shadow=[json.loads(l) for l in (D/'failed-shadow-0.jsonl').open()]
inds=np.arange(894,916)
ax.step(inds,[raw[i]['steer'] for i in inds],where='post',label='Published steering',color='#a23626')
ax.plot(inds,[shadow[i]['policy'] for i in inds],'.-',label='Reconstructed TLN output',color='#265b9b')
ax.axvline(908,color='0.45',ls=':');ax.set(xlabel='Controller step',ylabel='Steering (rad)',title='9006203  |  Steering before the first safety stop at step 916')
ax.legend(ncol=2)
ax=axs[2];q=json.loads((D/'braking-geometry.json').read_text());b=json.loads((D/'recorded-brake-beams.json').read_text())[:len(q)]
ax.plot([z['t']*1000 for z in q],[z['box_sat_m']*100 for z in q],label='Body-to-box SAT gap',color='#265b9b')
ax.plot([z['time']*1000 for z in b],[z['forward_envelope_gap_m']*100 for z in b],label='Gap to simulator TTC envelope',color='#a23626')
ax.axvline(260,ls=':',color='0.4');ax.set(xlabel='Time since brake decision sample (ms)',ylabel='Gap (cm)',title='5557202  |  Braking approach in recorded poses');ax.legend()
for ax in axs:ax.grid(axis='y',alpha=.20);ax.spines[['top','right']].set_visible(False)
fig.savefig(D/'comparison.png',dpi=145)
