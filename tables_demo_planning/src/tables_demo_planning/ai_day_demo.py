from typing import Dict, List, Optional
from geometry_msgs.msg import Pose
from unified_planning.engines import OptimalityGuarantee
from unified_planning.model import MinimizeSequentialPlanLength, Problem
from unified_planning.plans import ActionInstance
from unified_planning.shortcuts import Not, OneshotPlanner
from up_esb import Bridge


# Generic item class
class Item:
    def __init__(self, name: str) -> None:
        self.name = name


# Simple environment representation
# TODO Replace with fact generator.
class Env:
    robot_items: Dict[str, Item] = {}


# Robot action to get item
# TODO Call actual robot command within.
def get(item: Item) -> None:
    Env.robot_items[item.name] = item
    print(f"Get {item.name}.")


# Fluent which returns whether robot has item
def robot_has(item: Item) -> bool:
    return item in Env.robot_items.values()


class DemoDayDomain(Bridge):
    def __init__(self) -> None:
        super().__init__()
        self.create_types([Pose, Item])
        self.robot_has = self.create_fluent_from_function(robot_has)
        self.get, (item,) = self.create_action_from_function(get)
        self.get.add_precondition(Not(self.robot_has(item)))
        self.get.add_effect(self.robot_has(item), True)

    def define_mobipick_problem(self, api_objects: Dict[str, object]) -> Problem:
        # Create objects from scratch.
        self._objects.clear()
        self._api_objects.clear()
        objects = self.create_objects(api_objects)
        # Create problem with metric using only these objects.
        problem = self.define_problem(objects=objects)
        problem.add_quality_metric(MinimizeSequentialPlanLength())
        return problem

    def solve(self, problem: Problem) -> Optional[List[ActionInstance]]:
        result = OneshotPlanner(
            problem_kind=problem.kind,
            optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY,
        ).solve(problem)
        return result.plan.actions if result.plan else None

    def calculate_plan_to_get_all_items(
        self,
        names: List[str],
    ) -> Optional[List[ActionInstance]]:
        problem = self.define_mobipick_problem({name: Item(name) for name in names})
        # Add example goal to get all items.
        for item in self.objects.values():
            problem.add_goal(self.robot_has(item))
        # Initialize fluents.
        self.set_initial_values(problem)
        return self.solve(problem)


if __name__ == "__main__":
    domain = DemoDayDomain()

    plan = domain.calculate_plan_to_get_all_items(["Item_1", "Item_2"])
    print(plan)

    plan = domain.calculate_plan_to_get_all_items(["Item_3", "Item_4"])
    print(plan)

    if plan:
        for action in plan:
            function, parameters = domain.get_executable_action(action)
            function(*parameters)
