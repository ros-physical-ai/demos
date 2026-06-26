#!/usr/bin/env python3
"""Generate a deterministic fixture ONNX for unit tests.

Produces identity_policy.onnx: a 2-layer MLP that maps
  state(1, 6)  ->  action(1, 1, 6)
where output = clamp(state, -1, 1). The clamp is arbitrary — we only need
a graph with the right I/O shape and dtype that runs cheaply on CPU.
"""
import sys
from pathlib import Path

import numpy as np
from onnx import TensorProto, helper, save

OUT = Path(__file__).parent / "identity_policy.onnx"


def main() -> None:
    state = helper.make_tensor_value_info("observation.state", TensorProto.FLOAT, [1, 6])
    action = helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, 1, 6])

    w = helper.make_tensor("W", TensorProto.FLOAT, [6, 6],
                            np.eye(6, dtype=np.float32))
    b = helper.make_tensor("b", TensorProto.FLOAT, [6],
                            np.zeros((6,), dtype=np.float32))

    matmul = helper.make_node("MatMul", ["observation.state", "W"], ["matmul_out"])
    add    = helper.make_node("Add",    ["matmul_out", "b"],       ["add_out"])
    reshape_shape = helper.make_tensor("reshape_shape", TensorProto.INT64, [3],
                                        np.array([1, 1, 6], dtype=np.int64))
    reshape_in = helper.make_node("Reshape", ["add_out", "reshape_shape"],
                                   ["reshape_in"])
    clip_min = helper.make_tensor("clip_min", TensorProto.FLOAT, [],
                                    np.array(-1.0, dtype=np.float32))
    clip_max = helper.make_tensor("clip_max", TensorProto.FLOAT, [],
                                    np.array( 1.0, dtype=np.float32))
    clip = helper.make_node("Clip", ["reshape_in", "clip_min", "clip_max"], ["action"])

    graph = helper.make_graph(
        [matmul, add, reshape_in, clip], "identity_policy",
        [state],
        [action],
        initializer=[w, b, reshape_shape, clip_min, clip_max],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 18)])
    model.ir_version = 9
    save(model, str(OUT))
    print(f"wrote {OUT} ({OUT.stat().st_size} bytes)")


if __name__ == "__main__":
    sys.exit(main())