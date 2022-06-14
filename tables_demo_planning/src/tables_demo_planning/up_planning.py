from typing import Any, Dict, List, Optional, Tuple, Type
from collections import OrderedDict
from unified_planning.model import Fluent, InstantaneousAction, Object, Parameter, Problem
from unified_planning.plans.plan import ActionInstance
from unified_planning.shortcuts import BoolType, OneshotPlanner, UserType


class Action:
    SIGNATURE: Tuple[Type, ...]

    def __init__(self, *args: Any) -> None:
        self.args = args

    def __call__(self) -> bool:
        """Execute this action."""
        raise NotImplementedError(f"{self.__class__.__name__}.__call__() is not implemented!")

    def update_execution(self) -> None:
        """Update world state on action execution."""

    def update_completion(self) -> None:
        """Update world state on action completion."""


class Planning:
    def __init__(self) -> None:
        self.types: Dict[Type, UserType] = {}
        self.fluents: Dict[str, Fluent] = {}
        self.actions: Dict[Type, InstantaneousAction] = {}
        self.objects: Dict[str, Object] = {}
        self.api_objects: Dict[Object, object] = {}
        self.problem = Problem()

    def create_types(self, api_types: List[Type]) -> None:
        """Create UP user types based on api_types."""
        for api_type in api_types:
            assert api_type not in self.types.keys()
            self.types[api_type] = UserType(api_type.__name__)

    def get_type(self, api_type: Type) -> UserType:
        """Return the UP user type corresponding to api_type."""
        for check_type, user_type in self.types.items():
            if issubclass(api_type, check_type):
                return user_type
        raise ValueError(f"No corresponding UserType defined for {api_type}!")

    def get_object_type(self, api_object: object) -> UserType:
        """Return the UP user type corresponding to api_object's type."""
        for api_type, user_type in self.types.items():
            if isinstance(api_object, api_type):
                return user_type
        raise ValueError(f"No corresponding UserType defined for {api_object}!")

    def create_fluent(self, name: str, api_types: List[Type]) -> Fluent:
        """Create a UP BoolType() fluent using the UP types corresponding to the api_types given."""
        assert name not in self.fluents.keys()
        self.fluents[name] = fluent = Fluent(
            name,
            BoolType(),
            OrderedDict([(api_type.__name__.lower(), self.get_type(api_type)) for api_type in api_types]),
        )
        return fluent

    def create_action(self, api_action: Type) -> Tuple[InstantaneousAction, List[Parameter]]:
        """
        Create a UP InstantaneousAction by using the execute() method signature of api_action.
        Return the InstantaneousAction with its parameters for convenient definition of preconditions and effects.
        """
        assert api_action not in self.actions.keys()

        action = InstantaneousAction(
            api_action.__name__,
            OrderedDict(
                [
                    (f"{index}_{api_type.__name__}", self.get_type(api_type))
                    for index, api_type in enumerate(api_action.SIGNATURE)
                ]
            ),
        )
        self.actions[api_action] = action
        return action, action.parameters

    def get_action(self, action: ActionInstance) -> Action:
        """Return the Robot API action associated with the given action."""
        for api_action, check_action in self.actions.items():
            if action.action == check_action:  # Note: Must check for equality, cannot use hash of dict!
                api_parameters = [self.api_objects[parameter.object()] for parameter in action.actual_parameters]
                return api_action(*api_parameters)
        raise ValueError(f"No corresponding robot_api.Action defined for {action}!")

    def create_object(self, name: str, api_object: object) -> Object:
        """Create UP object based on api_object."""
        assert name not in self.objects.keys()
        self.objects[name] = obj = Object(name, self.get_object_type(api_object))
        self.api_objects[obj] = api_object
        return obj

    def create_objects(self, api_objects: Optional[Dict[str, object]] = None, **kwargs: object) -> List[Object]:
        """Create UP objects based on api_objects."""
        api_objs: Dict[str, object] = kwargs if api_objects is None else dict(api_objects, **kwargs)
        objs: List[Object] = []
        for name, api_object in api_objs.items():
            objs.append(self.create_object(name, api_object))
        return objs

    def init_problem(self) -> Problem:
        """Return a UP problem with all fluents, actions, and objects for definition of initial values and goals."""
        self.problem = Problem()
        for fluent in self.fluents.values():
            self.problem.add_fluent(fluent, default_initial_value=False)
        for action in self.actions.values():
            self.problem.add_action(action)
        self.problem.add_objects(self.objects.values())
        return self.problem

    def plan_actions(self) -> Optional[List[Tuple[ActionInstance, Action]]]:
        """Solve planning problem, then return list of UP and Robot API actions."""
        result = OneshotPlanner(problem_kind=self.problem.kind).solve(self.problem)
        return [(action, self.get_action(action)) for action in result.plan.actions] if result.plan else None
