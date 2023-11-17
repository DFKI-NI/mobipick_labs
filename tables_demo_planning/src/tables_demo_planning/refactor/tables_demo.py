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
# Authors: Alexander Sung, Oscar Lima, Marc Vinci

"""
The Tables Demo domain and environment specify concrete instances of the scenario for planning
 and include methods valid for both simulation and reality. It does not handle execution yet.
"""


from typing import Callable, Dict, List, Optional, Set
from collections import defaultdict
from geometry_msgs.msg import Pose
from unified_planning.model import Object, Problem
from tables_demo_planning.refactor.domain import Domain
from tables_demo_planning.refactor.components import ArmPose, Item, Location, Robot


class TablesDemoDomain(Domain):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.api_robot = robot
        self.robot = self.get(Robot, robot.name)
        self.pose_locations: Dict[Object, Object] = {}
        self.arm_pose_unknown = self.get(ArmPose, "arm_pose_unknown")
        self.nothing = self.get(Item, "nothing")
        self.anywhere = self.get(Location, "anywhere")

    def initialize_pose_locations(self) -> None:
        """Initialize information which pose is at which location."""
        for name, obj in self.objects.items():
            if name.startswith("base_table_") and name.endswith("_pose"):
                self.pose_locations[obj] = self.get(Location, name[5:-5])
            elif name.endswith("_search_pose"):
                self.pose_locations[obj] = self.get(Location, name[:-4] + "location")
        assert self.pose_locations

    def get_pose_at(self, pose: Object, location: Object) -> bool:
        """Return fluent value whether pose is at location."""
        if not self.pose_locations:
            self.initialize_pose_locations()
        return location == (self.pose_locations[pose] if pose in self.pose_locations.keys() else self.anywhere)

    def set_goals(self, problem: Problem, target_location: Location) -> None:
        problem.clear_goals()
        problem.add_goal(self.robot_at(self.robot, self.objects["base_table_2_pose"]))
        problem.add_goal(self.robot_arm_at(self.robot, self.objects["arm_pose_home"]))


class EnvironmentRepresentation:
    def __init__(self, demo_items: List[Item]) -> None:
        self.demo_items = demo_items
        self.table_locations = [location for name, location in Location.instances.items() if name.startswith("table_")]
        self.robot_home_poses: Dict[Robot, Pose] = {}
        self.robot_poses: Dict[Robot, Pose] = {}
        self.robot_arm_poses: Dict[Robot, ArmPose] = defaultdict(lambda: ArmPose.get("arm_pose_unknown"))
        self.robot_items: Dict[Robot, Item] = defaultdict(lambda: Item.get("nothing"))
        self.believed_item_locations: Dict[Item, Location] = defaultdict(lambda: Location.get("anywhere"))
        # self.newly_perceived_item_locations: Dict[Item, Location] = {}
        self.item_search: Optional[Item] = None
        self.searched_locations: Set[Location] = set()
        self.search_location = Location.get("anywhere")
        self.offered_items: Set[Item] = set()

        # Provide a function which returns all perceived items with their locations.
        #  Note: Due to request, this is not an abstract method anymore.
        self.perceive: Callable[[Robot, Location], Dict[Item, Location]]

    def initialize_robot_states(
        self,
        robot: Robot,
        pose: Optional[Pose] = None,
        arm_pose: Optional[ArmPose] = None,
        item: Optional[Item] = None,
    ) -> None:
        if pose:
            self.robot_home_poses[robot] = self.robot_poses[robot] = pose
        if arm_pose:
            self.robot_arm_poses[robot] = arm_pose
        if item:
            self.robot_items[robot] = item

    def get_robot_at(self, robot: Robot, pose: Pose) -> bool:
        """Return fluent value whether robot is at pose."""
        return self.robot_poses[robot] == pose

    def get_robot_arm_at(self, robot: Robot, arm_pose: ArmPose) -> bool:
        """Return fluent value whether robot arm is at arm_pose."""
        return self.robot_arm_poses[robot] == arm_pose

    def get_robot_has(self, robot: Robot, item: Item) -> bool:
        """Return fluent value whether robot has item."""
        return self.robot_items[robot] == item

    def get_item_offered(self, item: Item) -> bool:
        """Return fluent value whether item has already been offered."""
        return item in self.offered_items

    def get_believe_item_at(self, item: Item, location: Location) -> bool:
        """Return fluent value whether item is believed to be at location."""
        return location == self.believed_item_locations[item]

    def get_searched_at(self, location: Location) -> bool:
        """Return fluent value whether robot has already searched at location."""
        return location in self.searched_locations

    def resolve_search_location(self, location: Location) -> Location:
        """Resolve a location symbol to the actual table where the search succeeded."""
        if location.name not in ("tool_search_location", "klt_search_location"):
            return location

        assert self.search_location in self.table_locations
        return self.search_location

    def print_believed_item_locations(self) -> None:
        """Print at which locations the items are believed to be."""
        print("The believed item locations are:")
        for item in self.demo_items:
            print(f"- {item.name}:", self.believed_item_locations[item].name)

    def move_base(self, robot: Robot, _: Pose, pose: Pose) -> bool:
        """Move robot base to pose."""
        self.robot_poses[robot] = pose
        return True

    def move_base_with_item(self, robot: Robot, item: Item, _: Pose, pose: Pose) -> bool:
        """Move robot base with item to pose."""
        # Note: Same action as move_base(), just with item and arm in transport pose.
        return self.move_base(robot, _, pose)

    def move_arm(self, robot: Robot, _: ArmPose, arm_pose: ArmPose) -> bool:
        """Move robot arm to arm_pose."""
        self.robot_arm_poses[robot] = arm_pose
        return True

    def pick_item(self, robot: Robot, pose: Pose, location: Location, item: Item) -> bool:
        """
        At pose, let robot look for item at location, pick it up, then move its arm to transport pose.
        """
        print(f"Successfully picked up {item.name}.")
        self.robot_items[robot] = item
        self.believed_item_locations[item] = Location.get("on_robot")
        self.robot_arm_poses[robot] = ArmPose.get("transport")
        return True

    def place_item(self, robot: Robot, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, let robot place item at location, then move its arm to home pose."""
        print(f"Successfully placed {item.name}.")
        self.robot_items[robot] = Item.get("nothing")
        self.believed_item_locations[item] = location
        self.robot_arm_poses[robot] = ArmPose.get("home")
        return True

    def store_item(self, robot: Robot, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, let robot store item into KLT at location, then move its arm to home pose."""
        print(f"Successfully inserted {item.name} into KLT.")
        self.robot_items[robot] = Item.get("nothing")
        self.believed_item_locations[item] = Location.get("in_klt")
        self.robot_arm_poses[robot] = ArmPose.get("home")
        return True

    def hand_over_item(self, robot, Robot, pose: Pose, item: Item) -> bool:
        """At pose, let robot hand over item held to a person."""
        print(f"Successfully handed {item.name} over.")
        self.item = Item.get("nothing")
        self.believed_item_locations[item] = Location.get("anywhere")
        self.arm_pose = ArmPose.get("handover")
        self.offered_items.add(item)
        return True

    def search_at(self, robot: Robot, pose: Pose, location: Location) -> bool:
        """At pose, let robot search for item_search at location."""
        item_locations = self.perceive(robot, location)
        if self.item_search is not None:
            self.search_location = location
            item = self.item_search
            assert item
            if item in item_locations.keys():
                print(f"Search for {item.name} SUCCESSFUL.")
                self.item_search = None
        return True

    def check_reset_search(self) -> None:
        """Reset search if all tables have been searched."""
        if self.searched_locations.issuperset(self.table_locations):
            self.searched_locations.clear()

    def search_tool(self, robot: Robot, item: Item) -> bool:
        """Let robot initiate search for item."""
        self.item_search = item
        self.check_reset_search()
        return True

    def search_klt(self, robot: Robot) -> bool:
        """Let robot initiate search for the KLT."""
        self.item_search = Item.get("klt_1")
        self.check_reset_search()
        return True

    def conclude_tool_search(self, item: Item) -> bool:
        """Conclue tool search as failed. Success is determined in search_at()."""
        print(f"Search for {item.name} FAILED!")
        return False

    def conclude_klt_search(self) -> bool:
        """Conclue KLT search as failed. Success is determined in search_at()."""
        print("Search for KLT FAILED!")
        return False
