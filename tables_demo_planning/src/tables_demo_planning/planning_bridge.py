from typing import Any, Dict, List, Optional, Tuple, Type
from abc import ABC, abstractmethod
from collections import OrderedDict
from unified_planning.model import Fluent, InstantaneousAction, Object, Parameter
from unified_planning.plans.plan import ActionInstance
from unified_planning.shortcuts import BoolType, UserType

"""Bridge library to map between representations in robotics and planning domains."""


class Action(ABC):
    SIGNATURE: Tuple[Type, ...]

    def __init__(self, *args: Any) -> None:
        self.args = args

    @abstractmethod
    def __call__(self) -> bool:
        """Execute this action."""


class Bridge:
    def __init__(self) -> None:
        self.types: Dict[Type, UserType] = {}
        self.fluents: Dict[str, Fluent] = {}
        self.actions: Dict[Type, InstantaneousAction] = {}
        self.api_actions: Dict[str, Type] = {}
        self.objects: Dict[str, Object] = {}
        self.api_objects: Dict[str, object] = {}

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
        Create a UP InstantaneousAction by using api_action's signature.
        Return the InstantaneousAction with its parameters for convenient definition
         of preconditions and effects.
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
        self.api_actions[api_action.__name__] = api_action
        return action, action.parameters

    def get_action(self, action: ActionInstance) -> Action:
        """Return the Robot API action associated with the given action."""
        if action.action.name not in self.api_actions.keys():
            raise ValueError(f"No corresponding Action defined for {action}!")

        api_parameters = [self.api_objects[parameter.object().name] for parameter in action.actual_parameters]
        return self.api_actions[action.action.name](*api_parameters)

    def create_object(self, name: str, api_object: object) -> Object:
        """Create UP object based on api_object."""
        assert name not in self.objects.keys()
        self.objects[name] = obj = Object(name, self.get_object_type(api_object))
        self.api_objects[name] = api_object
        return obj

    def create_objects(self, api_objects: Optional[Dict[str, object]] = None, **kwargs: object) -> List[Object]:
        """Create UP objects based on api_objects."""
        api_objs: Dict[str, object] = kwargs if api_objects is None else dict(api_objects, **kwargs)
        objs: List[Object] = []
        for name, api_object in api_objs.items():
            objs.append(self.create_object(name, api_object))
        return objs
