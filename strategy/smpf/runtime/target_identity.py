"""Stable target identity state for ordered SMPF task stages."""

from dataclasses import dataclass
from typing import Optional, Tuple


class CompletedTargetError(ValueError):
    """A long-horizon stage selected an object completed by an earlier stage."""

    def __init__(self, object_id):
        self.object_id = str(object_id)
        super().__init__("target %s was completed by an earlier stage" % self.object_id)


@dataclass(frozen=True)
class TargetIdentityState:
    current_object_id: Optional[str] = None
    completed_object_ids: Tuple[str, ...] = ()

    def select(self, object_id, reject_completed=False):
        object_id = str(object_id or "").strip()
        if not object_id:
            raise ValueError("target object_id cannot be empty")
        if reject_completed and object_id in self.completed_object_ids:
            raise CompletedTargetError(object_id)
        return TargetIdentityState(object_id, self.completed_object_ids)

    def complete_current(self):
        if self.current_object_id is None:
            raise ValueError("cannot complete a stage without a selected target")
        completed = self.completed_object_ids
        if self.current_object_id not in completed:
            completed = completed + (self.current_object_id,)
        return TargetIdentityState(None, completed)


def associate_target_observation(
    memory,
    sphere,
    timestamp,
    identity,
    reject_completed=False,
    dynamic=False,
    max_distance_m=None,
):
    """Associate an observation, rejecting a completed entity before memory mutation."""
    matched = memory.match(sphere, timestamp, max_distance_m=max_distance_m)
    if matched is not None:
        identity.select(matched.object_id, reject_completed=reject_completed)
    entry = memory.update(
        sphere,
        timestamp,
        dynamic=dynamic,
        max_distance_m=max_distance_m,
    )
    return entry, identity.select(entry.object_id, reject_completed=reject_completed)
