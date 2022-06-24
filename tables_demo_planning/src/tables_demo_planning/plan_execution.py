from typing import Optional
from tables_demo_planning.demo_domain import Domain
from tables_demo_planning.plan_visualization import PlanVisualization

"""Generic execution loop for any demo in the mobipick tables domain."""


class Executor:
    def __init__(self, domain: Domain) -> None:
        self.domain = domain
        self.visualization: Optional[PlanVisualization] = None

    def run(self) -> None:
        active = True
        while active:
            # Create problem based on current state.
            self.problem = self.domain.initialize_problem()
            self.domain.set_values(self.problem)
            self.domain.set_goals(self.problem)

            # Plan
            actions = self.domain.solve(self.problem)
            if not actions:
                print("Execution ended because no plan could be found.")
                break

            print("> Plan:")
            up_actions = [up_action for up_action, _ in actions]
            print('\n'.join(map(str, up_actions)))
            if self.visualization:
                self.visualization.set_actions(up_actions)
            else:
                self.visualization = PlanVisualization(up_actions)
            # ... and execute.
            print("> Execution:")
            for up_action, (method, parameters) in actions:
                print(up_action)
                self.visualization.execute(up_action)
                result = method(*parameters)
                if result is None or result:
                    self.visualization.succeed(up_action)
                else:
                    print("-- Action failed! Need to replan.")
                    self.visualization.fail(up_action)
                    # Abort execution and loop to planning.
                    break
            else:
                active = False
                print("Task complete.")
