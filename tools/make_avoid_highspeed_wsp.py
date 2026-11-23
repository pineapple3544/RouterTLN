"""Add a bounded high-speed steering residual to the original schema-4 Avoid archive."""
import argparse
import hashlib
import tempfile
from pathlib import Path

import torch

SOURCE_SHA = "6754bf58549b40854652abd1f4d26af833453905342e8ecec5c4dc8982ec673f"
GAIN = .25
START_MPS = 2.7
FULL_MPS = 3.7
HEAD = ("head.0.weight", "head.0.bias", "head.2.weight", "head.2.bias", "head.4.weight")
ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def edit(state):
    """Keep the original head in its upper-left block; new paths reach steering only."""
    a, b, c, d, out = [state[name] for name in HEAD]
    assert tuple(a.shape) == (128, 1040) and tuple(c.shape) == (64, 128) and tuple(out.shape) == (2, 64)
    h1, h2 = len(b), len(d)
    # Speed and angles follow observe() and AvoidCorridor::condition().
    ctx = a.shape[1] - 16
    first = a.new_zeros((4, a.shape[1]))
    first[0, ctx+4], first[0, ctx+2] = 1., -1.
    first[1] = -first[0]
    first[2:, ctx] = 8. / (FULL_MPS-START_MPS)
    bias = b.new_tensor([0, 0, -START_MPS/(FULL_MPS-START_MPS), -FULL_MPS/(FULL_MPS-START_MPS)])
    middle = c.new_zeros((h2+4, h1+4))
    middle[:h2, :h1] = c
    # For positive and negative error, R(x)-R(x-gate) is min(x,gate).
    middle[h2:, h1:] = c.new_tensor([[1,0,0,0], [1,0,-1,1], [0,1,0,0], [0,1,-1,1]])
    extension = out.new_zeros((2,4))
    extension[0] = out.new_tensor([GAIN, -GAIN, -GAIN, GAIN])
    return dict(zip(HEAD, (torch.cat((a,first)), torch.cat((b,bias)), middle,
                           torch.cat((d,d.new_zeros(4))), torch.cat((out,extension),dim=1))))


def verify(path, original, updates):
    saved = torch.jit.load(str(path), map_location="cpu").state_dict()
    assert set(saved)==set(original)
    for key, old in original.items():
        assert torch.equal(saved[key], updates.get(key, old)), key
        prefix = tuple(slice(0,n) for n in old.shape)
        assert torch.equal(saved[key][prefix],old), key
    first, bias, middle, _, out = [saved[key] for key in HEAD]
    assert torch.count_nonzero(middle[:64,128:])==0
    assert torch.count_nonzero(middle[64:,:128])==0
    assert torch.count_nonzero(out[1,64:])==0
    # Check only the added coefficients. No model inference or simulator runs.
    for speed in (0., 2.2, 2.7, 3.0, 3.2, 3.7, 5., 10.):
        for error in (-2., -.25, 0., .25, 2.):
            features=first.new_zeros(16)
            features[0],features[2],features[4]=speed/8.,-error/2.,error/2.
            h=(first[128:,-16:]@features+bias[128:]).relu()
            delta=out[:,64:]@(middle[64:,128:]@h).relu()
            gate=max(0.,min(1.,(speed-START_MPS)/(FULL_MPS-START_MPS)))
            assert abs(delta[0].item()-GAIN*max(-gate,min(gate,error)))<1e-6
            assert delta[1].item()==0.


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,default=ROOT/"models/avoid-corridor-v2.pt")
    parser.add_argument("--output",type=Path,default=ROOT/"models/avoid-corridor-v2-wsp-highspeed-lead.pt")
    args=parser.parse_args()
    if digest(args.source)!=SOURCE_SHA:
        parser.error("expected the original avoid-corridor-v2.pt")
    if args.output.exists():
        parser.error("output already exists")
    torch.set_num_threads(1)
    archive=torch.jit.load(str(args.source),map_location="cpu")
    state=archive.state_dict()
    assert state["schema"].tolist()==[4,181,4,32,16]
    with torch.no_grad():
        original={k:v.clone() for k,v in state.items()}
        updates=edit(original)
        params=dict(archive.named_parameters())
        for name,value in updates.items():
            params[name].set_(value)
        with tempfile.TemporaryDirectory(dir=args.output.parent) as tmp:
            pending=Path(tmp)/args.output.name
            torch.jit.save(archive,str(pending))
            verify(pending,original,updates)
            # Creating the final file with an exclusive link also protects parallel runs.
            args.output.hardlink_to(pending)
    print(f"{args.output.name} {digest(args.output)}")


if __name__=="__main__":
    main()
