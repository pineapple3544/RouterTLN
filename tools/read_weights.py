"""Print CPU checkpoint metadata or selected tensor values as JSON."""
import argparse
import json
from pathlib import Path

import torch


def main():
    models = Path(__file__).resolve().parents[1] / "models"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", nargs="?", type=Path,
                        default=models / "avoid-corridor-v2.pt")
    parser.add_argument("--tensor", help="Tensor name from the metadata listing")
    parser.add_argument("--row", type=int, help="Select one row of a named matrix")
    args = parser.parse_args()
    if args.row is not None and not args.tensor:
        parser.error("--row requires --tensor")
    if not args.checkpoint.is_file():
        parser.error(f"Checkpoint not found: {args.checkpoint}")

    model = torch.jit.load(str(args.checkpoint), map_location="cpu")
    state = model.state_dict()
    result = {"checkpoint": args.checkpoint.name, "schema": state["schema"].tolist()}
    if args.tensor:
        if args.tensor not in state:
            parser.error(f"Unknown tensor: {args.tensor}")
        tensor = state[args.tensor]
        result.update(name=args.tensor, shape=list(tensor.shape), dtype=str(tensor.dtype))
        if args.row is not None:
            if tensor.ndim != 2 or not 0 <= args.row < tensor.shape[0]:
                parser.error("--row requires a matrix and a valid zero-based row index")
            tensor = tensor[args.row]
            result["row"] = args.row
        result["values"] = tensor.tolist()
    else:
        result["parameters"] = sum(parameter.numel() for parameter in model.parameters())
        result["tensors"] = {name: list(tensor.shape) for name, tensor in state.items()}
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
