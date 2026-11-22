"""Compare unconditional and curvature-dependent edits of the Avoid speed output."""
from pathlib import Path
import torch

root = Path(__file__).resolve().parent
source = root.parents[1] / "models/avoid-corridor-v2.pt"
for name in ["uniform", "control", "straight"]:
    model = torch.jit.load(str(source), map_location="cpu")
    state = model.state_dict()
    w0, b0, w1, b1, w2, b2 = [state[k] for k in (
        "head.0.weight", "head.0.bias", "head.2.weight",
        "head.2.bias", "head.4.weight", "head.4.bias")]
    with torch.no_grad():
        if name == "uniform":
            b2[1] += 0.18
        else:
            # Build relu(1 - abs(normalized reference steering) / 0.65).
            pair = w1.norm(dim=0).argsort()[:2].tolist()
            unit = int(w2.norm(dim=0).argmin())
            w1[:, pair] = 0
            w0[pair] = 0
            b0[pair] = 0
            w0[pair[0], 1024 + 4] = 1
            w0[pair[1], 1024 + 4] = -1
            w1[unit].zero_()
            w1[unit, pair] = -1 / 0.65
            b1[unit] = 1
            w2[:, unit] = 0
            if name == "straight":
                w2[1, unit] = 0.18
    target = root / (name + ".pt")
    if target.exists():
        raise FileExistsError(target)
    torch.jit.save(model, str(target))
