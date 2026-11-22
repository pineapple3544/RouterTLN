"""Build deterministic, gradient-free steering edits of the existing Race TLN."""
import json
from pathlib import Path

import torch

root = Path(__file__).resolve().parent
native = root.parents[1]
source = native / "models/race-teachers-expanded-v5.pt"
base = json.loads((root / "diagnosis.json").read_text())
for label, gain in (("ablated", 0.0), ("guided", 12.0)):
    model = torch.jit.load(str(source), map_location="cpu")
    state = model.state_dict()
    w0, b0, w1, b1, w2 = [state[key] for key in (
        "head.0.weight", "head.0.bias", "head.2.weight", "head.2.bias", "head.4.weight")]
    # Reuse units with the lowest outgoing weight norms to limit disruption.
    first = w1.norm(dim=0).argsort()[:2].tolist()
    second = w2.norm(dim=0).argsort()[:2].tolist()
    with torch.no_grad():
        w1[:, first] = 0
        w2[:, second] = 0
        for i, (a, b) in enumerate(zip(first, second)):
            sign = 1 if i == 0 else -1
            w0[a].zero_()
            w0[a, 1024 + 6] = sign
            b0[a] = 0
            w1[b].zero_()
            w1[b, a] = 1
            b1[b] = 0
            w2[0, b] = sign * gain
    output = root / (label + ".pt")
    if output.exists():
        raise FileExistsError(output)
    torch.jit.save(model, str(output))
    reloaded = torch.jit.load(str(output)).state_dict()
    original = torch.jit.load(str(source)).state_dict()
    diff = {k: int((v != original[k]).sum()) for k, v in reloaded.items()}
    print(label, first, second, {k:v for k,v in diff.items() if v}, sum(diff.values()))
    config = dict(base, checkpoint="/workspace/runs/race-wsp-berlin/" + output.name,
                  output="/workspace/runs/race-wsp-berlin/" + label, shadow_teacher=False)
    (root / (label + ".json")).write_text(json.dumps(config, indent=2))
