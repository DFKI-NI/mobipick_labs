from typing import Callable, Dict, List, Optional, Tuple, Type
from collections import OrderedDict
from unified_planning.model import Fluent, InstantaneousAction, Object, Parameter
from unified_planning.plans.plan import ActionInstance
from unified_planning.shortcuts import BoolType, UserType

"""Bridge library to map between representations in robotics and planning domains."""


class Bridge:
    def __init__(self) -> None:
        self.types: Dict[Type, UserType] = {}
        self.fluents: Dict[str, Fluent] = {}
        self.actions: Dict[str, InstantaneousAction] = {}
        self.api_actions: Dict[str, Callable[..., bool]] = {}
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

    def create_action(
        self, caller_type: Type, method: Callable[..., bool]
    ) -> Tuple[InstantaneousAction, List[Parameter]]:
        """
        Create a UP InstantaneousAction by using caller_type and method's signature.
        Return the InstantaneousAction with its parameters for convenient definition
         of preconditions and effects.
        """
        assert method.__name__ not in self.actions.keys()
        api_types = [caller_type]
        api_types.extend(method.__annotations__.values())
        api_types = api_types[:-1]  # without return type
        action = InstantaneousAction(
            method.__name__,
            OrderedDict(
                [(f"{index}_{api_type.__name__}", self.get_type(api_type)) for index, api_type in enumerate(api_types)]
            ),
        )
        self.actions[method.__name__] = action
        self.api_actions[method.__name__] = method
        return action, action.parameters

    def get_action(self, action: ActionInstance) -> Tuple[Callable[..., bool], List[object]]:
        """Return the Robot API action associated with the given action."""
        if action.action.name not in self.api_actions.keys():
            raise ValueError(f"No corresponding Action defined for {action}!")

        return self.api_actions[action.action.name], [
            self.api_objects[parameter.object().name] for parameter in action.actual_parameters
        ]

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
