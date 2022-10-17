# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, DFKI GmbH
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


from typing import Callable, Dict, Iterable, List, Optional, Tuple
import typing
from collections import OrderedDict
from enum import Enum
import itertools
import sys
from unified_planning.engines import OptimalityGuarantee
from unified_planning.model import Fluent, InstantaneousAction, Object, Parameter, Problem, Type
from unified_planning.plans import ActionInstance
from unified_planning.shortcuts import BoolType, IntType, OneshotPlanner, RealType, UserType

"""Generic bridge between application and planning domains"""


class Bridge:
    def __init__(self) -> None:
        # Note: Map from type instead of str to recognize subclasses.
        self.types: Dict[type, Type] = {bool: BoolType(), int: IntType(), float: RealType()}
        self.fluents: Dict[str, Fluent] = {}
        self.fluent_functions: Dict[str, Callable[[Iterable[Object]], Object]] = {}
        self.actions: Dict[str, InstantaneousAction] = {}
        self.api_actions: Dict[str, Callable[..., object]] = {}
        self.objects: Dict[str, Object] = {}
        self.api_objects: Dict[str, object] = {}

    def create_types(self, api_types: Iterable[type]) -> None:
        """Create UP user types based on api_types."""
        for api_type in api_types:
            assert api_type not in self.types.keys()
            self.types[api_type] = UserType(api_type.__name__)

    def get_type(self, api_type: type) -> Type:
        """Return UP user type corresponding to api_type or its superclasses."""
        for check_type, user_type in self.types.items():
            if issubclass(api_type, check_type):
                return user_type

        raise ValueError(f"No corresponding UserType defined for {api_type}!")

    def get_object_type(self, api_object: object) -> Type:
        """Return UP user type corresponding to api_object's type."""
        for api_type, user_type in self.types.items():
            if isinstance(api_object, api_type):
                return user_type

        raise ValueError(f"No corresponding UserType defined for {api_object}!")

    def create_fluent(self, function: Callable[..., object]) -> Fluent:
        """
        Create UP fluent based on function, which provides the fluent's values
         in the application domain for problem initialization.
        """
        name = function.__qualname__.split('.')[-1]
        assert name not in self.fluents.keys()
        api_types = list(function.__annotations__.items())
        _, result_api_type = api_types[-1]
        self.fluents[name] = Fluent(
            name,
            self.get_type(result_api_type),
            OrderedDict((parameter_name, self.get_type(api_type)) for parameter_name, api_type in api_types[:-1]),
        )
        self.fluent_functions[name] = lambda *args: self.get_object(
            function(*[self.api_objects[arg.name] for arg in args])
        )
        return self.fluents[name]

    def create_fluent_from_signature(
        self,
        function: Callable[[Iterable[Object]], Object],
        api_types: Iterable[type],
        result_api_type: Optional[type] = None,
    ) -> Fluent:
        """
        Create UP fluent using the UP types corresponding to the api_types given.
        By default, use BoolType() for the result unless specified otherwise through result_api_type.
        Use function which provides the fluent's values in the UP domain for problem initialization.
        """
        name = function.__qualname__.split('.')[-1]
        assert name not in self.fluents.keys()
        self.fluents[name] = Fluent(
            name,
            self.get_type(result_api_type) if result_api_type else BoolType(),
            OrderedDict([(api_type.__name__.lower(), self.get_type(api_type)) for api_type in api_types]),
        )
        self.fluent_functions[name] = function
        return self.fluents[name]

    def create_action(self, callable: Callable[..., object]) -> Tuple[InstantaneousAction, List[Parameter]]:
        """
        Create UP InstantaneousAction based on the function's signature.
        Return the InstantaneousAction with its parameters for convenient definition of its
         preconditions and effects.
        """
        action_name = callable.__name__ if hasattr(callable, '__name__') else callable.__class__.__name__
        assert action_name not in self.actions.keys()
        parameters: Dict[str, Type] = OrderedDict()
        if hasattr(callable, '__qualname__') and '.' in callable.__qualname__:
            # Add defining class of callable to parameters.
            namespace = sys.modules[callable.__module__]
            for name in callable.__qualname__.split('.')[:-1]:
                # Note: Use "context" to resolve potential relay to Python source file.
                namespace = (
                    namespace.__dict__["context"][name]
                    if "context" in namespace.__dict__.keys()
                    else namespace.__dict__[name]
                )
            assert isinstance(namespace, type)
            parameters[callable.__qualname__.rsplit('.', maxsplit=1)[0]] = self.get_type(namespace)
        # Add callable's parameter types, without its return type.
        annotations = (
            callable.__annotations__ if hasattr(callable, '__annotations__') else callable.__call__.__annotations__
        )  # type: ignore
        for parameter_name, api_type in list(annotations.items())[:-1]:
            parameters[parameter_name] = self.get_type(api_type)
        action = InstantaneousAction(action_name, parameters)
        self.actions[action_name] = action
        self.api_actions[action_name] = callable
        return action, action.parameters

    def get_executable_action(self, action: ActionInstance) -> Tuple[Callable[..., object], List[object]]:
        """Return API callable and parameters corresponding to the given action."""
        if action.action.name not in self.api_actions.keys():
            raise ValueError(f"No corresponding action defined for {action}!")

        return self.api_actions[action.action.name], [
            self.api_objects[parameter.object().name] for parameter in action.actual_parameters
        ]

    def create_object(self, name: str, api_object: object) -> Object:
        """Create UP object with name based on api_object."""
        assert name not in self.objects.keys()
        self.objects[name] = Object(name, self.get_object_type(api_object))
        self.api_objects[name] = api_object
        return self.objects[name]

    def create_objects(self, api_objects: Optional[Dict[str, object]] = None, **kwargs: object) -> List[Object]:
        """Create UP objects based on api_objects and kwargs."""
        return [
            self.create_object(name, api_object)
            for name, api_object in (dict(api_objects, **kwargs) if api_objects else kwargs).items()
        ]

    def create_enum_objects(self, enum: typing.Type[Enum]) -> List[Object]:
        """Create UP objects based on enum."""
        return [self.create_object(member.name, member) for member in enum]

    def get_object(self, api_object: object) -> Object:
        """Return UP object corresponding to api_object if it exists, else api_object itself."""
        name = getattr(api_object, "name") if hasattr(api_object, "name") else str(api_object)
        return self.objects[name] if name in self.objects.keys() else api_object

    def define_problem(
        self,
        fluents: Optional[Iterable[Fluent]] = None,
        actions: Optional[Iterable[InstantaneousAction]] = None,
        objects: Optional[Iterable[Object]] = None,
    ) -> Problem:
        """Define UP problem by its (potential subsets of) fluents, actions, and objects."""
        # Note: Reset goals and initial values to reuse this problem.
        problem = Problem()
        problem.add_fluents(self.fluents.values() if fluents is None else fluents)
        problem.add_actions(self.actions.values() if actions is None else actions)
        problem.add_objects(self.objects.values() if objects is None else objects)
        return problem

    def set_initial_values(self, problem: Problem) -> None:
        """Set all initial values using the functions corresponding to this problem's fluents."""
        type_objects: Dict[type, List[Object]] = {}
        # Collect objects in problem for all parameters of all fluents.
        for fluent in problem.fluents:
            for parameter in fluent.signature:
                # Avoid redundancy.
                if parameter.type not in type_objects.keys():
                    type_objects[parameter.type] = list(problem.objects(parameter.type))
        for fluent in problem.fluents:
            # Loop through all parameter value combinations.
            for parameters in itertools.product(*[type_objects[parameter.type] for parameter in fluent.signature]):
                # Use the fluent function to calculate the initial values.
                value = self.fluent_functions[fluent.name](*parameters)
                problem.set_initial_value(fluent(*parameters), value)

    def solve(self, problem: Problem) -> Optional[List[ActionInstance]]:
        """Solve planning problem and return list of UP actions."""
        result = OneshotPlanner(
            problem_kind=problem.kind, optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY
        ).solve(problem)
        return result.plan.actions if result.plan else None
