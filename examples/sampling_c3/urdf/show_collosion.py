#!/usr/bin/env python3
"""
Show an SDF/URDF in MeshCat, with both illustration and proximity geometry.

Example
-------
# Publish the geometry once:
python3 show_collision_meshcat.py examples/sampling_c3/urdf/T_vertical.sdf

# Run a 3-second simulation:
python3 show_collision_meshcat.py examples/sampling_c3/urdf/T_vertical.sdf --sim_time 3
"""

import argparse
import os
import time

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Parser,
    Role,
    Simulator,
    StartMeshcat,
)


def _path_to_file_uri(path: str) -> str:

    return "file://" + os.path.abspath(path)


def build_diagram(model_spec: str):

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)


    parser = Parser(plant)
    uri = (
        model_spec
        if model_spec.startswith(("package://", "file://", "model://"))
        else _path_to_file_uri(model_spec)
    )
    parser.AddModelsFromUrl(uri)
    plant.Finalize()


    meshcat = StartMeshcat()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    prox_params = MeshcatVisualizerParams()
    prox_params.role = Role.kProximity
    prox_params.prefix = "proximity"
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, prox_params)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    return diagram, context


def main():
    ap = argparse.ArgumentParser(
        description="Show an SDF/URDF in MeshCat with illustration and collision geometry"
    )
    ap.add_argument(
        "model_path",
        help="Path or URI to .sdf/.urdf (package://, file://, or plain path)",
    )
    ap.add_argument(
        "--sim_time",
        type=float,
        default=0.0,
        help="Seconds to simulate (0 = just publish once)",
    )
    args = ap.parse_args()

    diagram, context = build_diagram(args.model_path)

    if args.sim_time == 0.0:
        diagram.ForcedPublish(context)
        print("Geometry published to MeshCat.  Press Ctrl-C to quit.")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    else:
        sim = Simulator(diagram, context)
        sim.set_target_realtime_rate(1.0)
        print(f"Simulating {args.sim_time} s …")
        sim.Initialize()
        sim.AdvanceTo(args.sim_time)


if __name__ == "__main__":
    main()
