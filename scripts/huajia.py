"""The main CLI entry point."""

from dataclasses import dataclass
from pathlib import Path
import json


@dataclass
class Pose:
    x: float
    y: float
    angle: float
    name: None | str = None

    def export(self):
        return {
            **(
                {
                    "x": {"exp": f"{self.x} m", "val": self.x},
                    "y": {"exp": f"{self.y} m", "val": self.y},
                    "heading": {"exp": f"{self.angle} rad", "val": self.angle},
                }
                if self.name is None
                else {
                    "x": {"exp": f"{self.name}.x", "val": self.x},
                    "y": {"exp": f"{self.name}.y", "val": self.y},
                    "heading": {"exp": f"{self.name}.heading", "val": self.angle},
                }
            ),
            "intervals": 40,
            "split": False,
            "fixTranslation": True,
            "fixHeading": True,
            "overrideIntervals": False,
        }


KEEP_OUT_OF_REEF = {
    "from": "first",
    "to": "last",
    "data": {
        "type": "KeepOutCircle",
        "props": {
            "x": {
                "exp": "4.5 m",
                "val": 4.5,
            },
            "y": {
                "exp": "4 m",
                "val": 4,
            },
            "r": {"exp": ".88 m", "val": 0.88},
        },
    },
    "enabled": True,
}


def template(waypoints, name="New Path"):
    return {
        "name": name,
        "version": 1,
        "snapshot": {"waypoints": [], "constraints": [], "targetDt": 0.05},
        "params": {
            "waypoints": waypoints,
            "constraints": [
                {
                    "from": "first",
                    "to": None,
                    "data": {"type": "StopPoint", "props": {}},
                    "enabled": True,
                },
                {
                    "from": "last",
                    "to": None,
                    "data": {"type": "StopPoint", "props": {}},
                    "enabled": True,
                },
                KEEP_OUT_OF_REEF,
            ],
            "targetDt": {"exp": "0.05 s", "val": 0.05},
        },
        "trajectory": {
            "sampleType": None,
            "waypoints": [],
            "samples": [],
            "splits": [],
        },
        "events": [],
    }


def load_poses(raw):
    for pose in raw:
        yield pose, Pose(
            x=raw[pose]["x"]["val"],
            y=raw[pose]["y"]["val"],
            angle=raw[pose]["heading"]["val"],
            name=pose,
        )


def export(path):
    name = f"{'-'.join(p.name for p in path)}"
    Path(f"src/main/deploy/choreo/{name}.traj").write_text(
        json.dumps(
            template(
                [pose.export() for pose in path],
                name,
            ),
        )
    )
    print(name)


def main() -> None:
    """Run the app."""
    poses = dict(
        load_poses(
            json.loads(Path("src/main/deploy/choreo/Choreo.chor").read_text())[
                "variables"
            ]["poses"]
        )
    )
    for i, pose in enumerate("ABCDEFGHIJK"):
        export([poses["Source2"], poses[pose]])
        export([poses[pose], poses["Source2"]])
    # for i, pose in enumerate("ABCDEFGHIJK"):

    #     path = [poses["Source2"]]
    #     j = i + 1
    #     j %= len(positions)
    #     while len(path) < len(positions):
    #         path.append(poses[positions[j]])
    #         path.append(poses["Source2"])
    #         j += 1
    #         j %= len(positions)
    #         export(path)
    # for i in range(1 << len(poses)):
    #     Path(f"src/main/choreo/path_{i}.traj").write_text(
    #         json.dumps(
    #             template(
    #                 [pose.export() for j, pose in enumerate(poses) if i & (1 << j)],
    #                 f"Path {i}",
    #             ),
    #         )
    #     )
    #     print(i)


if __name__ == "__main__":
    main()
