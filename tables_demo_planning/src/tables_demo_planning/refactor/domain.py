# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, 2023, DFKI GmbH
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors: Alexander Sung, DFKI

"""
The Mobipick domain, providing methods to connect its applications to its planning representations
"""


from typing import Callable, List, Type
from geometry_msgs.msg import Pose
from unified_planning.model import Fluent, InstantaneousAction, Object
from unified_planning.shortcuts import Equals, Not, Or
from up_esb import Bridge
from tables_demo_planning.refactor.components import ArmPose, Item, Location, Robot, Unique


class Domain(Bridge):
    def __init__(self) -> None:
        super().__init__()
        # Create types for planning based on class types.
        self.create_types([Robot, Pose, ArmPose, Item, Location])

    def _get_fluent(self, name: str, **kwargs: type) -> Fluent:
        """
        Return fluent with name if it exists, else create a new one using kwargs.
        Note: Use set_fluent_functions() to provide a callable which calculates the fluent's values.
        """
        return self._fluents[name] if name in self._fluents.keys() else self.create_fluent(name, **kwargs)

    @property
    def robot_at(self) -> Fluent:
        return self._get_fluent("get_robot_at", robot=Robot, pose=Pose)

    @property
    def robot_arm_at(self) -> Fluent:
        return self._get_fluent("get_robot_arm_at", robot=Robot, arm_pose=ArmPose)

    @property
    def robot_has(self) -> Fluent:
        return self._get_fluent("get_robot_has", robot=Robot, item=Item)

    @property
    def believe_item_at(self) -> Fluent:
        return self._get_fluent("get_believe_item_at", item=Item, location=Location)

    @property
    def searched_at(self) -> Fluent:
        return self._get_fluent("get_searched_at", location=Location)

    @property
    def pose_at(self) -> Fluent:
        return self._get_fluent("get_pose_at", pose=Pose, location=Location)

    @property
    def item_offered(self) -> Fluent:
        return self._get_fluent("get_item_offered", item=Item)

    def get(self, api_type: Type[Unique], name: str) -> Object:
        """Return UP Object with name if it exists, else create a new one using api_type."""
        return self.objects[name] if name in self.objects.keys() else self.create_object(name, api_type.get(name))

    def get_objects_with_prefix(self, prefix: str) -> List[Object]:
        """Return UP Objects whose names start with prefix."""
        return [obj for name, obj in self.objects.items() if name.startswith(prefix)]

    def get_objects_with_suffix(self, suffix: str) -> List[Object]:
        """Return UP Objects whose names end with suffix."""
        return [obj for name, obj in self.objects.items() if name.endswith(suffix)]

    # TODO get_arm_pose_objects() from ArmPose

    def get_tool_objects(self) -> List[Object]:
        """Return UP Objects representing tools in the Mobipick domain."""
        return [
            self.get(Item, name)
            for name, item in Item.instances.items()
            if name.startswith("multimeter") or name.startswith("relay") or name.startswith("screwdriver")
        ]

    def create_move_base_action(self, _callable: Callable[[Robot, Pose, Pose], object]) -> InstantaneousAction:
        move_base, (robot, x, y) = self.create_action_from_function(_callable)
        move_base.add_precondition(self.robot_at(robot, x))
        move_base.add_precondition(Not(self.robot_at(robot, y)))
        move_base.add_precondition(self.robot_has(robot, self.get(Item, "nothing")))
        move_base.add_precondition(self.robot_arm_at(robot, self.get(ArmPose, "arm_pose_home")))
        move_base.add_effect(self.robot_at(robot, x), False)
        move_base.add_effect(self.robot_at(robot, y), True)
        return move_base

    def create_move_base_with_item_action(
        self, _callable: Callable[[Robot, Item, Pose, Pose], object]
    ) -> InstantaneousAction:
        move_base_with_item, (robot, item, x, y) = self.create_action_from_function(_callable)
        move_base_with_item.add_precondition(self.robot_at(robot, x))
        move_base_with_item.add_precondition(Not(self.robot_at(robot, y)))
        move_base_with_item.add_precondition(self.robot_has(robot, item))
        move_base_with_item.add_precondition(Not(Equals(item, self.get(Item, "nothing"))))
        move_base_with_item.add_precondition(self.robot_arm_at(robot, self.get(ArmPose, "arm_pose_transport")))
        move_base_with_item.add_effect(self.robot_at(robot, x), False)
        move_base_with_item.add_effect(self.robot_at(robot, y), True)
        return move_base_with_item

    def create_move_arm_action(self, _callable: Callable[[Robot, ArmPose, ArmPose], object]) -> InstantaneousAction:
        move_arm, (robot, x, y) = self.create_action_from_function(_callable)
        move_arm.add_precondition(self.robot_arm_at(robot, x))
        move_arm.add_precondition(Not(self.robot_arm_at(robot, y)))
        move_arm.add_effect(self.robot_arm_at(robot, x), False)
        move_arm.add_effect(self.robot_arm_at(robot, y), True)
        return move_arm

    def create_pick_item_action(
        self, _callable: Callable[[Robot, Pose, Location, Item], object]
    ) -> InstantaneousAction:
        pick_item, (robot, pose, location, item) = self.create_action_from_function(_callable)
        pick_item.add_precondition(self.robot_at(robot, pose))
        pick_item.add_precondition(self.robot_has(robot, self.get(Item, "nothing")))
        pick_item.add_precondition(self.believe_item_at(item, location))
        pick_item.add_precondition(self.pose_at(pose, location))
        pick_item.add_precondition(
            Or(
                Equals(location, table)
                for table in [
                    *self.get_objects_with_prefix("table_"),
                    self.get(Location, "tool_search_location"),
                    self.get(Location, "klt_search_location"),
                ]
            ),
        )
        pick_item.add_precondition(Not(Equals(item, self.get(Item, "nothing"))))
        pick_item.add_effect(self.robot_has(robot, self.get(Item, "nothing")), False)
        pick_item.add_effect(self.robot_has(robot, item), True)
        for arm_pose in self.get_objects_with_prefix("arm_pose_"):
            pick_item.add_effect(
                self.robot_arm_at(robot, arm_pose), arm_pose == self.get(ArmPose, "arm_pose_transport")
            )
        pick_item.add_effect(self.believe_item_at(item, location), False)
        pick_item.add_effect(self.believe_item_at(item, self.get(Location, "on_robot")), True)
        return pick_item

    def create_place_item_action(
        self, _callable: Callable[[Robot, Pose, Location, Item], object]
    ) -> InstantaneousAction:
        place_item, (robot, pose, location, item) = self.create_action_from_function(_callable)
        place_item.add_precondition(self.robot_at(robot, pose))
        place_item.add_precondition(self.robot_has(robot, item))
        place_item.add_precondition(self.believe_item_at(item, self.get(Location, "on_robot")))
        place_item.add_precondition(self.pose_at(pose, location))
        place_item.add_precondition(Not(Equals(location, self.get(Location, "anywhere"))))
        place_item.add_effect(self.robot_has(robot, item), False)
        place_item.add_effect(self.robot_has(robot, self.get(Item, "nothing")), True)
        for arm_pose in self.get_objects_with_prefix("arm_pose_"):
            place_item.add_effect(self.robot_arm_at(robot, arm_pose), arm_pose == self.get(ArmPose, "arm_pose_home"))
        place_item.add_effect(self.believe_item_at(item, self.get(Location, "on_robot")), False)
        place_item.add_effect(self.believe_item_at(item, location), True)
        return place_item

    def create_store_item_action(
        self, _callable: Callable[[Robot, Pose, Location, Item], object]
    ) -> InstantaneousAction:
        store_item, (robot, pose, location, item) = self.create_action_from_function(_callable)
        store_item.add_precondition(self.robot_at(robot, pose))
        store_item.add_precondition(self.robot_has(robot, item))
        store_item.add_precondition(self.believe_item_at(item, self.get(Location, "on_robot")))
        store_item.add_precondition(self.believe_item_at(self.get(Item, "klt_1"), location))
        store_item.add_precondition(self.pose_at(pose, location))
        store_item.add_precondition(Not(Equals(location, self.get(Location, "anywhere"))))
        store_item.add_effect(self.robot_has(robot, item), False)
        store_item.add_effect(self.robot_has(robot, self.get(Item, "nothing")), True)
        for arm_pose in self.get_objects_with_prefix("arm_pose_"):
            store_item.add_effect(self.robot_arm_at(robot, arm_pose), arm_pose == self.get(ArmPose, "arm_pose_home"))
        store_item.add_effect(self.believe_item_at(item, self.get(Location, "on_robot")), False)
        store_item.add_effect(self.believe_item_at(item, self.get(Location, "in_klt")), True)

    def create_handover_item_action(self, _callable: Callable[[Robot, Pose, Item], object]) -> InstantaneousAction:
        handover_item, (robot, pose, item) = self.create_action_from_function(_callable)
        handover_item.add_precondition(self.robot_at(robot, pose))
        handover_item.add_precondition(self.robot_has(robot, item))
        handover_item.add_precondition(self.believe_item_at(item, self.get(Location, "on_robot")))
        handover_item.add_precondition(Not(self.item_offered(item)))
        handover_item.add_effect(self.robot_has(robot, item), False)
        handover_item.add_effect(self.robot_has(robot, self.get(Item, "nothing")), True)
        for arm_pose in self.get_objects_with_prefix("arm_pose_"):
            handover_item.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.get(ArmPose, "arm_pose_handover"))
        handover_item.add_effect(self.believe_item_at(item, self.get("on_robot")), False)
        handover_item.add_effect(self.believe_item_at(item, self.get("anywhere")), True)
        handover_item.add_effect(self.item_offered(item), True)
        return handover_item

    def create_search_at_action(self, _callable: Callable[[Robot, Pose, Location], object]) -> InstantaneousAction:
        search_at, (robot, pose, location) = self.create_action_from_function(_callable)
        search_at.add_precondition(self.robot_at(robot, pose))
        search_at.add_precondition(
            Or(
                self.robot_arm_at(robot, arm_pose)
                for arm_pose in (
                    self.get(ArmPose, "arm_pose_home"),
                    self.get(ArmPose, "arm_pose_observe100cm_right"),
                    self.get(ArmPose, "arm_pose_transport"),
                )
            )
        )
        search_at.add_precondition(Not(self.searched_at(location)))
        search_at.add_precondition(Or(Equals(location, table) for table in self.get_objects_with_prefix("table_")))
        search_at.add_precondition(self.pose_at(pose, location))
        search_at.add_effect(self.searched_at(location), True)
        for arm_pose in self.get_objects_with_prefix("arm_pose_"):
            search_at.add_effect(self.robot_arm_at(robot, arm_pose), arm_pose == self.get(ArmPose, "arm_pose_observe"))

    def create_search_tool_action(self, _callable: Callable[[Robot, Item], object]) -> InstantaneousAction:
        search_tool, (robot, item) = self.create_action_from_function(_callable)
        search_tool.add_precondition(Not(self.robot_at(robot, self.get(Pose, "tool_search_pose"))))
        search_tool.add_precondition(self.robot_has(robot, self.get(Item, "nothing")))
        search_tool.add_precondition(self.believe_item_at(item, self.get(Location, "anywhere")))
        search_tool.add_precondition(Or(Equals(item, tool) for tool in self.get_tool_objects()))
        for pose in self.get_objects_with_suffix("_pose"):
            search_tool.add_effect(self.robot_at(robot, pose), pose == self.get(Pose, "tool_search_pose"))
        search_tool.add_effect(self.believe_item_at(item, self.get(Location, "anywhere")), False)
        search_tool.add_effect(self.believe_item_at(item, self.get(Location, "tool_search_location")), True)

    def create_search_klt_action(self, _callable: Callable[[Robot], object]) -> InstantaneousAction:
        search_klt, (robot,) = self.create_action_from_function(_callable)
        search_klt.add_precondition(Not(self.robot_at(robot, self.get(Pose, "klt_search_pose"))))
        search_klt.add_precondition(self.believe_item_at(self.get(Item, "klt_1"), self.get(Location, "anywhere")))
        for pose in self.get_objects_with_suffix("_pose"):
            search_klt.add_effect(self.robot_at(robot, pose), pose == self.get(Pose, "klt_search_pose"))
        search_klt.add_effect(self.believe_item_at(self.get(Item, "klt_1"), self.get(Location, "anywhere")), False)
        search_klt.add_effect(
            self.believe_item_at(self.get(Item, "klt_1"), self.get(Location, "klt_search_location")), True
        )

    def create_conclude_tool_search_action(self, _callable: Callable[[Item], object]) -> InstantaneousAction:
        conclude_tool_search, (item,) = self.create_action_from_function(_callable)
        conclude_tool_search.add_precondition(self.believe_item_at(item, self.get(Location, "anywhere")))
        for table in self.get_objects_with_prefix("table_"):
            conclude_tool_search.add_precondition(self.searched_at(table))
        conclude_tool_search.add_precondition(Not(Equals(item, self.get(Item, "klt_1"))))
        conclude_tool_search.add_effect(self.believe_item_at(item, self.get(Location, "anywhere")), False)
        conclude_tool_search.add_effect(self.believe_item_at(item, self.get(Location, "tool_search_location")), True)

    def create_conclude_klt_search_action(self, _callable: Callable[[], object]) -> InstantaneousAction:
        conclude_klt_search, _ = self.create_action_from_function(_callable)
        conclude_klt_search.add_precondition(
            self.believe_item_at(self.get(Item, "klt_1"), self.get(Location, "anywhere"))
        )
        for table in self.get_objects_with_prefix("table_"):
            conclude_klt_search.add_precondition(self.searched_at(table))
        conclude_klt_search.add_effect(
            self.believe_item_at(self.get(Item, "klt_1"), self.get(Location, "anywhere")), False
        )
        conclude_klt_search.add_effect(
            self.believe_item_at(self.get(Item, "klt_1"), self.get(Location, "klt_search_location")), True
        )
