# Copyright (c) 2026, DFKI GmbH and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from typing import Dict, Generic, TypeVar


T = TypeVar("T")


class Unique(Generic[T]):
    instances: Dict[str, T]

    def __init__(self, name: str) -> None:
        if name in self.instances.keys():
            print(f"Warning: Replacing duplicate {self.__class__.__name__} '{name}'.")
        self.name = name
        self.instances[name] = self

    def __repr__(self) -> str:
        return str(self.name)

    @classmethod
    def get(cls, name: str) -> T:
        """Return instance with name if it exists, else create a new one."""
        return cls.instances[name] if name in cls.instances.keys() else cls(name)


class ArmPose(Unique["ArmPose"]):
    instances: Dict[str, "ArmPose"] = {}


class Item(Unique["Item"]):
    instances: Dict[str, "Item"] = {}


class Location(Unique["Location"]):
    instances: Dict[str, "Location"] = {}


class Robot(Unique["Robot"]):
    instances: Dict[str, "Robot"] = {}
