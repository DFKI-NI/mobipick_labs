import rospy
import rosparam

from typing import Dict, Iterable, List, Set, Sequence, Union, Optional, Callable
from collections import defaultdict
from geometry_msgs.msg import Pose, Point
from unified_planning.plans import ActionInstance
from tables_demo_planning.components import ArmPose, Item, Location, Robot
from tables_demo_planning.tables_demo import EnvironmentRepresentation, TablesDemoDomain

from symbolic_fact_generation.on_fact_generator import OnGenerator
from symbolic_fact_generation.robot_facts_generator import RobotAtGenerator, HasArmPostureGenerator
from pose_selector.srv import PoseDelete, PoseDeleteRequest
from mobipick_api import Robot as mobipick_api_robot
from robot_api import TuplePose, is_instance
from tables_demo_planning.subplan_visualization import SubPlanVisualization


class TablesDemoAPI:
    RETRIES_BEFORE_ABORTION = 2

    def __init__(self, api_items: Iterable[Item]) -> None:
        self.mobipick = Robot("mobipick")
        self.mobipick_api = mobipick_api_robot("mobipick", True, True)
        self.domain = TablesDemoDomain(self.mobipick)

        rosparam_namespace = "/mobipick/tables_demo_planning"
        self.api_poses: Dict[str, Pose] = {}
        table_names: List[str] = []
        for param_path in rosparam.list_params(rosparam_namespace):
            assert isinstance(param_path, str)
            param = rosparam.get_param(param_path)
            param_name = param_path.rsplit('/', maxsplit=1)[-1]
            # Collect poses from rosparams.
            if is_instance(param, Sequence[Sequence[Union[float, int]]]) and tuple(map(len, param)) == (3, 4):
                self.api_poses[param_name] = TuplePose.to_pose(param)
            # Collect table names from rosparam names.
            if param_name.startswith("base_table_") and param_name.endswith("_pose"):
                table_names.append(param_name[5:-5])
        if len({TuplePose.from_pose(pose) for pose in self.api_poses.values()}) < len(self.api_poses):
            rospy.logwarn(
                f"Duplicate poses in rosparam namespace '{rosparam_namespace}'"
                " might let checks fail whether something is at a specific symbolic pose."
            )

        self.api_poses["unknown_pose"] = Pose(position=Point(x=-1.0))
        self.api_poses["tool_search_pose"] = Pose(position=Point(x=-3.0))
        self.api_poses["klt_search_pose"] = Pose(position=Point(x=-5.0))
        self.api_pose_names = {id(pose): name for name, pose in self.api_poses.items()}
        self.poses = self.domain.create_objects(self.api_poses)
        self.items = self.domain.create_objects({item.name: item for item in api_items})
        self.tables = [self.domain.get(Location, name) for name in table_names]
        self.arm_poses = [
            self.domain.get(ArmPose, arm_pose)
            for arm_pose in ["unknown", "home", "transport", "handover", "observe100cm_right"]
        ]
        self.env = EnvironmentRepresentation(api_items)
        self.env.perceive = lambda _, location: self.perceive(_, location)
        self.env.initialize_robot_states(self.domain.api_robot, self.api_poses["base_home_pose"])
        self.demo_items = list(api_items)

        self.domain.set_fluent_functions(
            [
                self.get_robot_at,
                self.get_robot_arm_at,
                self.env.get_robot_has,
                self.env.get_believe_item_at,
                self.env.get_believe_item_in,
                self.env.get_searched_at,
                self.env.get_item_offered,
                self.domain.get_pose_at,
            ]
        )
        self.domain.create_move_base_action(self.move_base)
        self.domain.create_move_base_with_item_action(self.move_base_with_item)
        self.domain.create_move_arm_action(self.move_arm)
        self.domain.create_pick_item_action(self.pick_item)
        self.domain.create_place_item_action(self.place_item)
        self.domain.create_store_item_action(self.store_item)
        self.domain.create_hand_over_item_action(self.hand_over_item)
        self.domain.create_search_at_action(self.env.search_at, ["home", "observe100cm_right", "transport"])
        self.domain.create_search_tool_action(self.env.search_tool)
        self.domain.create_search_klt_action(self.env.search_klt)
        self.domain.create_conclude_tool_search_action(self.env.conclude_tool_search)
        self.domain.create_conclude_klt_search_action(self.env.conclude_klt_search)
        self.problem = self.domain.initialize_tables_demo_problem()
        self.subproblem = self.domain.initialize_item_search_problem()

        self.pose_selector_delete = rospy.ServiceProxy("/pick_pose_selector_node/pose_selector_delete", PoseDelete)

        self.on_fact_generator = OnGenerator(
            fact_name='on',
            objects_of_interest=[str(item) for item in self.demo_items],
            container_objects=['klt'],
            query_srv_str="/pick_pose_selector_node/pose_selector_class_query",
            planning_scene_param="/mobipick/pick_object_node/planning_scene_boxes",
        )
        self.robot_at_fact_generator = RobotAtGenerator(
            fact_name='robot_at',
            global_frame='/map',
            robot_frame='/mobipick/base_link',
            undefined_pose_name="unknown_pose",
        )
        self.arm_pose_fact_generator = HasArmPostureGenerator(
            fact_name='robot_arm_pose',
            joint_states_topic='/mobipick/joint_states',
            arm_posture_param='',
            arm_tolerance=0.01,
            undefined_pose_name="unknown",
        )
        # Clear all demo items' poses already on the pose_selector from previous observations.
        for item in self.demo_items:
            class_id, instance_id = str(item).rsplit("_", 1)
            self.pose_selector_delete(PoseDeleteRequest(class_id=class_id, instance_id=int(instance_id)))

        # Initialize plan visualization
        self.visualization = SubPlanVisualization()

        # Readable labels for parameters
        self.parameter_labels: Dict[str, str] = {
            "base_home_pose": "home",
            "base_handover_pose": "handover",
            "base_pick_pose": "pick",
            "base_place_pose": "place",
            "base_table_1_pose": "table_1",
            "base_table_2_pose": "table_2",
            "base_table_3_pose": "table_3",
            "tool_search_pose": "where tool has been found",
            "klt_search_pose": "where KLT has been found",
        }

        # Readable string for each action to visualize
        self.method_labels: Dict[str, Callable[[Sequence[str]], str]] = {
            "move_base": lambda parameters: f"Move to {parameters[-1]}",
            "move_base_with_item": lambda parameters: f"Transport {parameters[1]} to {parameters[-1]}",
            "move_arm": lambda parameters: f"Move arm to its {parameters[-1]} pose",
            "pick_item": lambda parameters: f"Pick up {parameters[-1]}",
            "place_item": lambda parameters: f"Place {parameters[-1]} onto table",
            "store_item": lambda parameters: f"Place {parameters[-2]} into {parameters[-1]}",
            "hand_over_item": lambda parameters: f"Handover {parameters[-1]} to person",
            "search_at": lambda parameters: f"Search at {parameters[-1]}",
            "search_tool": lambda parameters: f"Search tables for {parameters[-1]}",
            "search_klt": lambda _: "Search tables for the KLT",
            "conclude_tool_search": lambda parameters: f"Conclude search for {parameters[-1]}",
            "conclude_klt_search": lambda parameters: f"Conclude search for {parameters[-1]}",
        }

    def label(self, action: ActionInstance) -> str:
        """Return a user-friendly label for visualizing action names and parameters."""
        parameters = [parameter.object() for parameter in action.actual_parameters]
        parameter_labels = [self.parameter_labels.get(parameter.name, str(parameter)) for parameter in parameters]
        return self.method_labels[action.action.name](parameter_labels)

    def replan(self) -> Optional[List[ActionInstance]]:
        self.env.print_believed_item_locations()
        self.domain.set_initial_values(self.problem)
        return self.domain.solve(self.problem)

    def run(self, target_location: Optional[Location] = None) -> None:
        """
        Run planning and execution loop.
        Note: Set the goals for self.problem before using planning.
        """
        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
        error_counts: Dict[str, int] = defaultdict(int)
        executed_action_names: Set[str] = set()  # Note: For visualization purposes only.
        # Solve overall problem.
        plan = self.replan()
        if plan is None or plan.actions is None:
            print("Execution ended because no plan could be found.")
            return

        while plan.actions:
            print("> Plan:")
            print('\n'.join(map(str, plan.actions)))
            self.visualization.set_actions(
                [
                    f"{number + len(executed_action_names)} {self.label(action)}"
                    for number, action in enumerate(plan.actions, start=1)
                ],
                preserve_actions=executed_action_names,
            )
            print("> Execution:")
            for action in plan.actions:
                executable_action, parameters = self.domain.get_executable_action(action)
                action_name = f"{len(executed_action_names) + 1} {self.label(action)}"
                print(action)
                # Explicitly do not pick up KLT from target_table since planning does not handle it yet.
                if target_location and action.action.name == "pick_item" and parameters[-1].name.startswith("klt_"):
                    assert isinstance(parameters[-2], Location)
                    location = self.env.resolve_search_location(parameters[-2])
                    if location == target_location:
                        print("Picking up KLT OBSOLETE.")
                        self.visualization.cancel(action_name)
                        print("Replanning")
                        plan = self.replan()
                        break

                self.visualization.execute(action_name)

                # Execute action.
                result = executable_action(
                    *(parameters[1:] if hasattr(self.mobipick, action.action.name) else parameters)
                )
                executed_action_names.add(action_name)

                # Handle item search as an inner execution loop.
                # Rationale: It has additional stop criteria, and might continue the outer loop.
                if self.env.item_search:
                    try:
                        # Check whether an obsolete item search invalidates the previous plan.
                        if self.env.believed_item_locations[self.env.item_search] != Location.get("anywhere"):
                            print(f"Search for {self.env.item_search.name} OBSOLETE.")
                            self.visualization.cancel(action_name)
                            plan = self.replan()
                            break

                        # Search for item by creating and executing a subplan.
                        self.domain.set_initial_values(self.subproblem)
                        self.domain.set_search_goals(self.subproblem, self.env.item_search)
                        subplan = self.domain.solve(self.subproblem)
                        assert subplan, f"No solution for: {self.subproblem}"
                        print("- Search plan:")
                        print('\n'.join(map(str, subplan.actions)))
                        self.visualization.set_actions(
                            [
                                f"{len(executed_action_names)}{chr(number)} {self.label(subaction)}"
                                for number, subaction in enumerate(subplan.actions, start=97)
                            ],
                            preserve_actions=executed_action_names,
                            predecessor=action_name,
                        )
                        print("- Search execution:")
                        subaction_execution_count = 0
                        for subaction in subplan.actions:
                            executable_subaction, subparameters = self.domain.get_executable_action(subaction)
                            subaction_name = (
                                f"{len(executed_action_names)}{chr(subaction_execution_count + 97)}"
                                f" {self.label(subaction)}"
                            )
                            print(subaction)
                            self.visualization.execute(subaction_name)
                            # Execute search action.
                            result = executable_subaction(
                                *(subparameters[1:] if hasattr(self.mobipick, subaction.action.name) else subparameters)
                            )
                            subaction_execution_count += 1

                            if result is not None:
                                if result:
                                    # Note: True result only means any subaction succeeded.
                                    # Check if the search actually succeeded.
                                    if (
                                        self.env.item_search is None
                                        and len(self.env.newly_perceived_item_locations) <= 1
                                    ):
                                        print("- Continue with plan.")
                                        self.visualization.succeed(subaction_name)
                                        break
                                    # Check if the search found another item.
                                    elif self.env.newly_perceived_item_locations:
                                        self.env.newly_perceived_item_locations.clear()
                                        print("- Found another item, search ABORTED.")
                                        self.visualization.cancel(subaction_name)
                                        # Set result to None to trigger replanning.
                                        result = None
                                        break
                                else:
                                    self.visualization.fail(subaction_name)
                                    break
                        # Note: The conclude action at the end of any search always fails.
                    finally:
                        # Always end the search at this point.
                        self.env.item_search = None

                if result is not None:
                    if result:
                        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                        self.visualization.succeed(action_name)
                    else:
                        self.visualization.fail(action_name)
                        error_counts[self.domain.label(action)] += 1
                        # Note: This will also fail if two different failures occur successively.
                        if retries_before_abortion <= 0 or any(count >= 3 for count in error_counts.values()):
                            print("Task could not be completed even after retrying.")
                            self.visualization.add_node("Mission impossible", "red")
                            return

                        retries_before_abortion -= 1
                        plan = self.replan()
                        break
                else:
                    self.visualization.cancel(action_name)
                    retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                    plan = self.replan()
                    break
            else:
                break
            if plan is None or plan.actions is None:
                print("Execution ended because no plan could be found.")
                return

        print("Demo complete.")
        self.visualization.add_node("Demo complete", "green")

    def get_robot_at(self, robot: Robot, pose: Pose) -> bool:
        base_pose_name = None
        robot_at_facts = self.robot_at_fact_generator.generate_facts()
        if robot_at_facts:
            base_pose_name = robot_at_facts[0].values[0]
        base_pose = self.api_poses.get(base_pose_name, self.api_poses["unknown_pose"])
        return pose == base_pose

    def get_robot_arm_at(self, robot: Robot, arm_pose: ArmPose) -> bool:
        """Return fluent value whether robot arm is at arm_pose."""
        arm_pose_facts = self.arm_pose_fact_generator.generate_facts()
        arm_pose_name = ArmPose.get("unknown").name
        if arm_pose_facts:
            if arm_pose_facts[0].values[0] in ArmPose.instances:
                arm_pose_name = arm_pose_facts[0].values[0]
        return ArmPose.get(arm_pose_name) == arm_pose

    def move_base(self, robot: Robot, _: Pose, pose: Pose) -> bool:
        if self.mobipick_api.base.move(pose) != 3:
            rospy.logerr(f"Move base to {pose} FAILED!")
            # Move to home pose whenever movement fails. Note: This is a drastic workaround.
            self.mobipick_api.base.move(self.api_poses["base_home_pose"])
            return False

        return self.env.move_base(self.mobipick, _, pose)

    def move_base_with_item(self, robot: Robot, item: Item, _: Pose, pose: Pose) -> bool:
        if self.mobipick_api.base.move(pose) != 3:
            rospy.logerr(f"Move base to {pose} FAILED!")
            # Move to home pose whenever movement fails. Note: This is a drastic workaround.
            self.mobipick_api.base.move(self.api_poses["base_home_pose"])
            return False

        return self.env.move_base(self.mobipick, _, pose)

    def move_arm(self, robot: Robot, _: ArmPose, arm_pose: ArmPose) -> bool:
        if not self.mobipick_api.arm.move(arm_pose.name):
            rospy.logerr(f"Move arm to '{arm_pose.name} FAILED!'")
            return False

        return self.env.move_arm(self.mobipick, _, arm_pose)

    def pick_item(self, robot: Robot, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, look for item at location, pick it up, then move arm to transport pose."""

        location = self.env.resolve_search_location(location)
        perceived_item_locations = self.perceive(robot, location)
        if item not in perceived_item_locations.keys():
            rospy.logwarn(f"Cannot find {item.name} at {location.name}. Pick up FAILED!")
            return False

        if perceived_item_locations[item] != location:
            rospy.logwarn(f"Found {item.name} but not on {location.name}. Pick up FAILED!")
            return False

        # ignore objects inside klts
        ignore_object_list = [item.name for item in self.env.believed_klt_contents.get(item, [])]

        pick_result = self.mobipick_api.arm.pick_object(item.name, location.name, ignore_object_list)

        if not pick_result:
            rospy.logwarn(f"Pick up {item.name} at {location.name} FAILED!")
            return False

        self.env.pick_item(self.mobipick, pose, location, item)
        self.mobipick_api.arm.move("transport")
        return True

    def place_item(self, robot: Robot, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, place item at location, then move arm to home pose."""

        observe_before_place = not bool(self.env.believed_klt_contents.get(item, []))

        place_result = self.mobipick_api.arm.place_object(location.name, observe_before_place)

        if not place_result:
            rospy.logwarn(f"Place {item.name} at {location.name} FAILED!")
            return False

        self.env.place_item(self.mobipick, pose, location, item)
        self.mobipick_api.arm.move("home")
        return True

    def store_item(self, robot: Robot, pose: Pose, location: Location, tool: Item, klt: Item) -> bool:
        """At pose, store item into box at location, the move arm to home pose."""

        insert_result = self.mobipick_api.arm.insert_object(klt.name, True)

        if not insert_result:
            rospy.logwarn(f"Insert {tool.name} into {klt.name} at {location.name} FAILED!")
            return False

        self.env.store_item(self.mobipick, pose, location, tool, klt)
        self.mobipick_api.arm.move("home")
        return True

    def hand_over_item(self, robot: Robot, item: Item) -> bool:
        """
        At pose, hand over item to person, observe force torque feedback
        until item is taken by person, then move arm to home pose.
        """
        self.mobipick_api.arm.move("handover")
        # observe force torque sensor using treshold 5.0 and a timeout of 25.0 seconds
        if not self.mobipick_api.arm.observe_force_torque(5.0, 25.0):
            return False

        self.mobipick_api.arm.execute("ReleaseGripper")
        self.env.hand_over_item(self.mobipick, item)
        return True

    def perceive(self, robot: Robot, location: Location) -> Dict[Item, Location]:
        """Move arm into observation pose and return all perceived items with their locations."""

        rospy.loginfo(f"Clear facts for {location.name}.")
        for believed_item, believed_location in self.env.believed_item_locations.items():
            if believed_location == location:
                class_id, instance_id = believed_item.name.rsplit("_", 1)
                self.pose_selector_delete(PoseDeleteRequest(class_id=class_id, instance_id=int(instance_id)))

        self.mobipick_api.arm_cam.perceive(observation_list=['observe100cm_right'])

        rospy.loginfo("Get facts from fact generator.")
        facts = self.on_fact_generator.generate_facts()
        perceived_item_locations: Dict[Item, Location] = {}
        perceived_items_in_klts: Dict[Item, Set[Item]] = {}
        # Perceive facts for items on table location.
        for fact in facts:
            if fact.name == "on":
                fact_item_name, fact_location_name = fact.values
                rospy.loginfo(f"{fact_item_name} on {fact_location_name} returned by pose_selector and fact_generator.")
                if fact_item_name in [item.name for item in self.demo_items] and fact_location_name == location.name:
                    rospy.loginfo(f"{fact_item_name} is perceived as on {fact_location_name}.")
                    fact_item = Item.get(fact_item_name)
                    perceived_item_locations[fact_item] = location
                    # Remove item from offered_items dict to be able to repeat the handover
                    # action with that item, if it is perceived again in the scene.
                    self.env.offered_items.discard(fact_item)
        # Also perceive facts for items in klts if there are any perceived on table location.
        for fact in facts:
            if fact.name == "in":
                fact_item_name, fact_location_name = fact.values
                if (
                    fact_item_name in [item.name for item in self.demo_items]
                    and fact_location_name in Item.instances
                    and perceived_item_locations.get(Item.get(fact_location_name)) == location
                ):
                    rospy.loginfo(f"{fact_item_name} is perceived as in {fact_location_name}.")
                    perceived_items_in_klts.setdefault(Item.get(fact_location_name), set()).add(
                        Item.get(fact_item_name)
                    )
                    # Remove item from offered_items dict to be able to repeat the handover
                    # action with that item, if it is perceived again in the scene.
                    self.env.offered_items.discard(Item.get(fact_item_name))
        # Determine newly perceived items and their locations.
        self.env.newly_perceived_item_locations.clear()
        for perceived_item, perceived_location in perceived_item_locations.items():
            if (
                perceived_item not in self.env.believed_item_locations.keys()
                or self.env.believed_item_locations[perceived_item] != location
            ):
                self.env.newly_perceived_item_locations[perceived_item] = perceived_location
        rospy.loginfo(f"Newly perceived items: {self.env.newly_perceived_item_locations.keys()}")
        # Remove all previously perceived items at location.
        for check_item, check_location in list(self.env.believed_item_locations.items()):
            if check_location == location:
                del self.env.believed_item_locations[check_item]
        # Add all currently perceived items at location.
        self.env.believed_item_locations.update(perceived_item_locations)
        self.env.believed_klt_contents.update(
            perceived_items_in_klts
        )  # overwrites old believed klt contents for perceived klts
        # Update believed item locations for items in klts
        for klt_contents in self.env.believed_klt_contents.values():
            for content in klt_contents:
                self.env.believed_item_locations[content] = Location.get("in_klt")
        self.env.searched_locations.add(location)
        self.env.print_believed_item_locations()
        return perceived_item_locations
