#!/usr/bin/python3
# Copyright 2022-2024 Fraunhofer Italia Research

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copyright 2022 Fraunhofer Italia Research. All Rights Reserved.
#
# Author: Marco Magri <marco.magri@fraunhofer.it>

from __future__ import annotations
from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, List

from waypoint_manager_msgs.msg import Target as TargetMsg
from waypoint_manager_msgs.msg import Motion as MotionMsg
from waypoint_manager_msgs.msg import Action as ActionMsg

from geometry_msgs.msg import Pose, Point, Quaternion

DEFAULT_VELOCITY_SCALING = 0.1
DEFAULT_ACCELERATION_SCALING = 0.1


class Entity(ABC):
    name: str
    description: str

    def __init__(self, name: str, description: str) -> None:
        super().__init__()
        self.name = name
        self.description = description

    @abstractmethod
    def from_dict(dict: Dict[str, Any]) -> Entity:
        pass

    @abstractmethod
    def to_dict(self) -> Dict[str, Any]:
        pass

    @abstractmethod
    def from_msg(msg) -> Entity:
        pass

    @abstractmethod
    def to_msg(self):
        pass


_MOTION_TYPES = {
    "CARTESIAN": 0,
    "JOINT": 1,
}

_SPACE_TYPES = {
    "CARTESIAN": 0,
    "JOINT": 1,
}

_SCALE_TYPES = {
    "RELATIVE": 0,
    "ABSOLUTE": 1,
}

_PIPELINE_TYPES = {
    "OMPL": 0,
    "pilz_industrial_motion_planner": 1,
}


def reverse_dict(dict):
    return {v: k for k, v in dict.items()}


class Target(Entity):
    class SpaceType(Enum):
        CARTESIAN = 0
        JOINT = 1

    class ScaleType(Enum):
        RELATIVE = 0
        ABSOLUTE = 1

    space_type: SpaceType = SpaceType.JOINT
    scale_type: ScaleType = ScaleType.ABSOLUTE
    frame_id: str = None
    tip_frame: str = None
    target: List[float] = None
    seed: List[float] = None
    max_velocity_scaling_factor: float = DEFAULT_VELOCITY_SCALING
    max_acceleration_scaling_factor: float = DEFAULT_ACCELERATION_SCALING
    blending_radius: float = 0.0

    def __init__(
        self,
        name: str,
        target: List[float],
        space_type: SpaceType,
        scale_type: ScaleType,
        frame_id: str = None,
        tip_frame: str = None,
        seed: List[float] = None,
        max_velocity_scaling_factor: float = DEFAULT_VELOCITY_SCALING,
        max_acceleration_scaling_factor: float = DEFAULT_ACCELERATION_SCALING,
        blending_radius: float = 0.0,
        description: str = "",
    ) -> None:
        # TODO: add checks that arguments are coherent
        super().__init__(name, description)
        self.space_type = space_type
        self.scale_type = scale_type
        self.frame_id = frame_id
        self.tip_frame = tip_frame
        self.target = target
        self.seed = seed
        self.max_velocity_scaling_factor = max_velocity_scaling_factor
        self.max_acceleration_scaling_factor = max_acceleration_scaling_factor
        self.blending_radius = blending_radius

    def from_dict(dict: Dict[str, Any]) -> Target:
        raise NotImplementedError

    def to_dict(self) -> Dict[str, Any]:
        dict = self.__dict__.copy()
        dict["space_type"] = reverse_dict(_SPACE_TYPES)[self.space_type.value]
        dict["scale_type"] = reverse_dict(_SCALE_TYPES)[self.scale_type.value]
        return dict

    def from_msg(msg: TargetMsg) -> Target:
        space_type = msg.space_type
        if space_type == Target.SpaceType.CARTESIAN:
            pose = msg.cartesian_target.pose
            target_values = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            seed = msg.cartesian_target.seed
            frame_id = msg.cartesian_target.frame_id
            tip_frame = msg.cartesian_target.tip_frame
        else:
            target_values = msg.joint_target.target
            seed = []
            frame_id = ""
            tip_frame = ""

        return Target(
            name=msg.id,
            description=msg.description,
            space_type=space_type,
            scale_type=msg.scale_type,
            frame_id=frame_id,
            tip_frame=tip_frame,
            target=list(target_values),
            seed=list(seed),
            max_velocity_scaling_factor=f"{msg.max_velocity_scaling_factor:.3f}",
            max_acceleration_scaling_factor=f"{msg.max_acceleration_scaling_factor:.3f}",
            blending_radius=f"{msg.blending_radius:.3f}",
        )

    def to_msg(self) -> TargetMsg:
        target_msg = TargetMsg()
        target_msg.id = self.name
        target_msg.description = self.description
        target_msg.space_type = self.space_type.value
        target_msg.scale_type = self.scale_type.value
        target_msg.max_velocity_scaling_factor = float(self.max_velocity_scaling_factor)
        target_msg.max_acceleration_scaling_factor = float(
            self.max_acceleration_scaling_factor
        )
        target_msg.blending_radius = float(self.blending_radius)
        if self.space_type == Target.SpaceType.CARTESIAN:
            target_msg.cartesian_target.pose = Pose(
                position=Point(x=self.target[0], y=self.target[1], z=self.target[2]),
                orientation=Quaternion(
                    x=self.target[3],
                    y=self.target[4],
                    z=self.target[5],
                    w=self.target[6],
                ),
            )
            target_msg.cartesian_target.frame_id = str(self.frame_id)
            target_msg.cartesian_target.seed = self.seed
            target_msg.cartesian_target.tip_frame = self.tip_frame
        else:
            target_msg.joint_target.target = self.target

        return target_msg


class Motion(Entity):
    class Type(Enum):
        CARTESIAN = 0
        JOINT = 1

    class PipelineID(Enum):
        OMPL = 0
        PILZ = 1

    pipeline_id: PipelineID
    type: Type
    targets: List[Target]

    def __init__(
        self,
        name: str,
        type: Type,
        targets: List[Target],
        pipeline_id: PipelineID = PipelineID.PILZ,
        description: str = "",
    ) -> None:
        super().__init__(name, description)
        self.pipeline_id = pipeline_id
        self.type = type
        self.targets = targets

    def from_dict(dict: Dict[str, Any]) -> Entity:
        raise NotImplementedError

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "pipeline_id": reverse_dict(_PIPELINE_TYPES)[self.pipeline_id.value],
            "motion_type": reverse_dict(_MOTION_TYPES)[self.type.value],
            "targets": [t.to_dict() for t in self.targets],
        }

    def from_msg(msg: MotionMsg) -> Motion:
        return Motion(
            name=msg.id,
            description=msg.description,
            pipeline_id=msg.pipeline_id,
            type=msg.motion_type,
            targets=[Target.from_msg(target_msg) for target_msg in msg.targets],
        )

    def to_msg(self) -> MotionMsg:
        motion_msg = MotionMsg()
        motion_msg.id = self.name
        motion_msg.pipeline_id = self.pipeline_id.value
        motion_msg.description = self.description
        motion_msg.motion_type = self.type.value
        motion_msg.targets = [target.to_msg() for target in self.targets]
        return motion_msg


class Action(Entity):
    group_name: str
    motions: List[Motion]

    def __init__(
        self,
        name: str,
        group_name: str,
        motions: List[Motion],
        description: str = "",
    ) -> None:
        super().__init__(name, description)
        self.group_name = group_name
        self.motions = motions

    def from_dict(dict: Dict[str, Any]) -> Entity:
        raise NotImplementedError

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "group_name": self.group_name,
            "description": self.description,
            "motions": [t.to_dict() for t in self.motions],
        }

    def from_msg(msg: ActionMsg) -> Motion:
        return Action(
            name=msg.id,
            description=msg.description,
            group_name=msg.group_name,
            motions=[Motion.from_msg(motion_msgs) for motion_msgs in msg.motions],
        )

    def to_msg(self) -> ActionMsg:
        motion_msg = ActionMsg()
        motion_msg.id = self.name
        motion_msg.description = self.description
        motion_msg.group_name = self.group_name
        motion_msg.motions = [motion.to_msg() for motion in self.motions]
        return motion_msg


# target = Target(
#     name="pippo/target",
#     scale_type=Target.ScaleType.ABSOLUTE,
#     space_type=Target.SpaceType.CARTESIAN,
#     target=[0.0] * 6,
# )
# motion = Motion(name="pippo/motion", type=Motion.Type.CARTESIAN)
# action = Action(name="pippo/action", group_name="pippo")
# motion.targets.append(target)
# action.motions.append(motion)

# import yaml
# print(yaml.safe_dump(action.to_dict(), sort_keys=False, default_flow_style=None))
