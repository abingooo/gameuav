import unittest

from strategy.smpf.runtime.contracts import ObjectSphere
from strategy.smpf.runtime.scene_memory import SemanticSceneMemory
from strategy.smpf.runtime.target_identity import (
    CompletedTargetError,
    TargetIdentityState,
    associate_target_observation,
)


class SmpfTargetIdentityTest(unittest.TestCase):
    def test_completed_target_is_rejected_by_next_stage(self):
        state = TargetIdentityState().select("obj-0001").complete_current()
        with self.assertRaises(CompletedTargetError):
            state.select("obj-0001", reject_completed=True)

    def test_distinct_target_is_accepted_and_retains_history(self):
        state = TargetIdentityState().select("obj-0001").complete_current()
        state = state.select("obj-0002", reject_completed=True)
        self.assertEqual(state.current_object_id, "obj-0002")
        self.assertEqual(state.completed_object_ids, ("obj-0001",))

    def test_association_rejects_old_entity_but_accepts_nearby_distinct_entity(self):
        memory = SemanticSceneMemory(association_distance_m=0.35)
        first = ObjectSphere("chair", (1.0, 0.0, 0.0), 0.8, frame_id="world")
        entry, state = associate_target_observation(
            memory,
            first,
            1.0,
            TargetIdentityState(),
        )
        state = state.complete_current()
        repeated = ObjectSphere("chair", (1.1, 0.0, 0.0), 0.8, frame_id="world")
        with self.assertRaises(CompletedTargetError):
            associate_target_observation(
                memory,
                repeated,
                2.0,
                state,
                reject_completed=True,
            )
        distinct = ObjectSphere("chair", (1.6, 0.0, 0.0), 0.8, frame_id="world")
        second_entry, state = associate_target_observation(
            memory,
            distinct,
            3.0,
            state,
            reject_completed=True,
        )
        self.assertNotEqual(second_entry.object_id, entry.object_id)
        self.assertEqual(state.current_object_id, second_entry.object_id)


if __name__ == "__main__":
    unittest.main()
