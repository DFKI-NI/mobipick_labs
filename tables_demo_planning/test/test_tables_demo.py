#!/usr/bin/env python3

"""This test script tests planning, re-planning, and visualization without Gazebo simulation."""


from typing import Dict, List, Optional
import unified_planning
from geometry_msgs.msg import Pose
from unified_planning.model import Object
from unified_planning.plans import ActionInstance
from tables_demo_planning.mobipick_components import ArmPose, Item, Location
from tables_demo_planning.tables_demo import TablesDemoDomain, TablesDemoEnv, TablesDemoRobot


class TablesDemoSimRobot(TablesDemoRobot['TablesDemoSimEnv']):
    def pick_item(self, pose: Pose, location: Location, item: Item) -> bool:
        location = self.env.resolve_search_location(location)
        perceived_item_locations = self.perceive(location)
        if perceived_item_locations.get(item) != location:
            print(f"Cannot find {item.name} at {location.name}. Pick item FAILED!")
            return False

        if self.env.actual_item_locations[item] != location:
            print(f"Pick up {item.name} at {location.name} FAILED!")
            return False

        self.env.actual_item_locations[item] = Location.on_robot
        return TablesDemoRobot.pick_item(self, pose, location, item)

    def place_item(self, pose: Pose, location: Location, item: Item) -> bool:
        location = self.env.resolve_search_location(location)
        if item != self.env.robot.item:
            print(f"Item {item.name} is not on the robot. Place item FAILED!")
            return False

        if self.env.actual_item_locations[item] != Location.on_robot:
            print(f"Place down {item.name} at {location.name} FAILED!")
            return False

        self.env.actual_item_locations[item] = location
        return TablesDemoRobot.place_item(self, pose, location, item)

    def store_item(self, pose: Pose, location: Location, item: Item) -> bool:
        location = self.env.resolve_search_location(location)
        perceived_item_locations = self.perceive(location)
        if perceived_item_locations.get(Item.get("box_1")) != location:
            print(f"Cannot find box at {location.name}. Store item FAILED!")
            return False

        if (
            self.env.actual_item_locations[item] != Location.on_robot
            or self.env.actual_item_locations[Item.get("box_1")] != location
        ):
            print(f"Store {item.name} into box at {location.name} FAILED!")
            return False

        return TablesDemoRobot.store_item(self, pose, location, item)

    def perceive(self, location: Location) -> Dict[Item, Location]:
        # Determine newly perceived items and their locations.
        perceived_item_locations = {
            check_item: check_location
            for check_item, check_location in self.env.actual_item_locations.items()
            if check_location == location
        }
        self.env.newly_perceived_item_locations.clear()
        for perceived_item, perceived_location in perceived_item_locations.items():
            if (
                perceived_item not in self.env.believed_item_locations.keys()
                or self.env.believed_item_locations[perceived_item] != location
            ):
                self.env.newly_perceived_item_locations[perceived_item] = perceived_location
        print(f"Newly perceived items: {[item.name for item in self.env.newly_perceived_item_locations.keys()]}")
        # Remove all previously perceived items at location.
        for check_item, check_location in list(self.env.believed_item_locations.items()):
            if check_location == location:
                del self.env.believed_item_locations[check_item]
        # Add all currently perceived items at location.
        self.env.believed_item_locations.update(perceived_item_locations)
        self.env.searched_locations.add(location)
        self.env.print_believed_item_locations()
        return perceived_item_locations


class TablesDemoSimEnv(TablesDemoEnv[TablesDemoSimRobot]):
    def __init__(self, item_locations: Dict[Item, Location]) -> None:
        super().__init__(TablesDemoSimRobot(self))
        self.actual_item_locations = item_locations
        self.location_pose_names = {
            Location.anywhere: TablesDemoDomain.UNKNOWN_POSE_NAME,
            Location.table_1: TablesDemoDomain.BASE_TABLE_1_POSE_NAME,
            Location.table_2: TablesDemoDomain.BASE_TABLE_2_POSE_NAME,
            Location.table_3: TablesDemoDomain.BASE_TABLE_3_POSE_NAME,
            Location.tool_search_location: TablesDemoDomain.TOOL_SEARCH_POSE_NAME,
            Location.box_search_location: TablesDemoDomain.BOX_SEARCH_POSE_NAME,
        }


class TablesDemoSimDomain(TablesDemoDomain[TablesDemoSimEnv]):
    def __init__(self, item_locations: Dict[Item, Location]) -> None:
        super().__init__(TablesDemoSimEnv(item_locations))
        home_pose = self.api_poses[self.BASE_HOME_POSE_NAME]
        self.env.robot.initialize(home_pose, home_pose, ArmPose.unknown, Item.NOTHING)
        self.set_fluent_functions((self.get_robot_at,))
        self.set_api_actions(
            (
                TablesDemoSimRobot.move_base,
                TablesDemoSimRobot.move_base_with_item,
                TablesDemoSimRobot.move_arm,
                TablesDemoSimRobot.pick_item,
                TablesDemoSimRobot.place_item,
                TablesDemoSimRobot.store_item,
            )
        )

    def get_robot_at(self, pose: Object) -> bool:
        return pose.name in self.api_poses.keys() and self.api_poses[pose.name] == self.env.robot.pose

    def replan(self) -> Optional[List[ActionInstance]]:
        # Remember and delete locations for outdated items.
        actual_item_locations: Dict[str, Location] = {}
        believed_item_locations: Dict[str, Location] = {}
        for name, item in list(Item.items.items()):
            if item != Item.NOTHING:
                del self._objects[name]
                del self._api_objects[name]
                del Item.items[name]
                if item in self.env.actual_item_locations.keys():
                    actual_item_locations[name] = self.env.actual_item_locations[item]
                    del self.env.actual_item_locations[item]
                if item in self.env.believed_item_locations.keys():
                    believed_item_locations[name] = self.env.believed_item_locations[item]
                    del self.env.believed_item_locations[item]
        # Create new items.
        items = {
            item.name: item
            for item in [
                Item.get("power_drill_1"),
                Item.get("box_1"),
                Item.get("multimeter_1"),
                Item.get("relay_1"),
                Item.get("screwdriver_1"),
            ]
        }
        self.items = self.create_objects(items)
        self.power_drill = self.objects["power_drill_1"]
        self.box = self.objects["box_1"]
        self.multimeter = self.objects["multimeter_1"]
        self.relay = self.objects["relay_1"]
        self.screwdriver = self.objects["screwdriver_1"]
        # Update item locations from previously remembered locations.
        for name, location in actual_item_locations.items():
            if name in items.keys():
                self.env.actual_item_locations[items[name]] = location
        for name, location in believed_item_locations.items():
            if name in items.keys():
                self.env.believed_item_locations[items[name]] = location
        # Reset goals using the new items.
        target_item = Item.get("multimeter_1")
        TablesDemoDomain.DEMO_ITEMS = {item.name: item for item in (Item.get("box_1"), target_item)}
        target_location = Location.table_2
        self.set_goals(target_location)
        return super().replan()


def test_tables_demo() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("multimeter_1"): Location.table_3,
        Item.get("relay_1"): Location.table_3,
        Item.get("screwdriver_1"): Location.table_2,
        Item.get("box_1"): Location.table_1,
    }
    # Define goal.
    target_location = Location.table_2

    TablesDemoSimDomain(item_locations).run(target_location)


if __name__ == '__main__':
    test_tables_demo()
