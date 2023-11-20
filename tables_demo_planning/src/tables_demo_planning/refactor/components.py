from typing import Dict, Generic, TypeVar


T = TypeVar("T")


class Unique(Generic[T]):
    instances: Dict[str, T]

    def __init__(self, name: str) -> None:
        assert name not in self.instances.keys(), f"{self.__class__.__name__} '{name}' already exists!"
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
