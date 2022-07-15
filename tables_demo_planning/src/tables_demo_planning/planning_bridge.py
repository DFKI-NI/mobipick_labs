from typing import Callable, Dict, Iterable, List, Optional, Tuple, Type
from collections import OrderedDict
from enum import Enum
import sys
from unified_planning.model import Fluent, InstantaneousAction, Object, Parameter
from unified_planning.plans import ActionInstance
from unified_planning.shortcuts import BoolType, IntType, RealType, UserType

"""Generic bridge between application and planning domains"""


class Bridge:
    def __init__(self) -> None:
        # Note: Map from type instead of str to recognize subclasses.
        self._types: Dict[type, UserType] = {bool: BoolType(), int: IntType(), float: RealType()}
        self._fluents: Dict[str, Fluent] = {}
        self._fluent_functions: Dict[str, Callable[..., object]] = {}
        self._actions: Dict[str, InstantaneousAction] = {}
        self._api_actions: Dict[str, Callable[..., object]] = {}
        self._objects: Dict[str, Object] = {}
        self._api_objects: Dict[str, object] = {}

    @property
    def objects(self) -> Dict[str, Object]:
        return self._objects

    def create_types(self, api_types: Iterable[type]) -> None:
        """Create UP user types based on api_types."""
        for api_type in api_types:
            assert api_type not in self._types.keys()
            self._types[api_type] = UserType(api_type.__name__)

    def get_type(self, api_type: type) -> UserType:
        """Return UP user type corresponding to api_type or its superclasses."""
        for check_type, user_type in self._types.items():
            if issubclass(api_type, check_type):
                return user_type

        raise ValueError(f"No corresponding UserType defined for {api_type}!")

    def get_object_type(self, api_object: object) -> UserType:
        """Return UP user type corresponding to api_object's type."""
        for api_type, user_type in self._types.items():
            if isinstance(api_object, api_type):
                return user_type

        raise ValueError(f"No corresponding UserType defined for {api_object}!")

    def create_fluent(self, name: str, api_types: Iterable[Type], result_type: Optional[Type] = None) -> Fluent:
        """
        Create a UP fluent using the UP types corresponding to the api_types given.
        By default, use BoolType() for the result unless specified otherwise.
        """
        assert name not in self._fluents.keys()
        self._fluents[name] = fluent = Fluent(
            name,
            self.get_type(result_type) if result_type else BoolType(),
            OrderedDict([(api_type.__name__.lower(), self.get_type(api_type)) for api_type in api_types]),
        )
        return fluent

    def create_action(self, function: Callable[..., object]) -> Tuple[InstantaneousAction, List[Parameter]]:
        """
        Create UP InstantaneousAction based on the function's signature.
        Return the InstantaneousAction with its parameters for convenient definition of its
         preconditions and effects.
        """
        assert function.__name__ not in self._actions.keys()
        parameters: Dict[str, UserType] = OrderedDict()
        if '.' in function.__qualname__:
            # Add defining class of function to parameters.
            namespace = sys.modules[function.__module__]
            for name in function.__qualname__.split('.')[:-1]:
                # Note: Use "context" to resolve potential relay to Python source file.
                namespace = (
                    namespace.__dict__["context"][name]
                    if "context" in namespace.__dict__.keys()
                    else namespace.__dict__[name]
                )
            assert isinstance(namespace, type)
            parameters[function.__qualname__.rsplit('.', maxsplit=1)[0]] = self.get_type(namespace)
        # Add function's parameter types, without its return type.
        for parameter_name, api_type in list(function.__annotations__.items())[:-1]:
            parameters[parameter_name] = self.get_type(api_type)
        action = InstantaneousAction(function.__name__, parameters)
        self._actions[function.__name__] = action
        self._api_actions[function.__name__] = function
        return action, action.parameters

    def get_executable_action(self, action: ActionInstance) -> Tuple[Callable[..., object], List[object]]:
        """Return API function and parameters corresponding to the given action."""
        if action.action.name not in self._api_actions.keys():
            raise ValueError(f"No corresponding action defined for {action}!")

        return self._api_actions[action.action.name], [
            self._api_objects[parameter.object().name] for parameter in action.actual_parameters
        ]

    def create_object(self, name: str, api_object: object) -> Object:
        """Create UP object with name based on api_object."""
        assert name not in self._objects.keys()
        self._objects[name] = Object(name, self.get_object_type(api_object))
        self._api_objects[name] = api_object
        return self._objects[name]

    def create_objects(self, api_objects: Optional[Dict[str, object]] = None, **kwargs: object) -> List[Object]:
        """Create UP objects based on api_objects and kwargs."""
        return [
            self.create_object(name, api_object)
            for name, api_object in (dict(api_objects, **kwargs) if api_objects else kwargs).items()
        ]

    def create_enum_objects(self, enum: Type[Enum]) -> List[Object]:
        """Create UP objects based on enum."""
        return [self.create_object(member.name, member) for member in enum]
