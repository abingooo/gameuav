"""Conservative world-frame semantic memory for repeated SMPF observations."""

from dataclasses import dataclass, replace
import json
import math
import re
from typing import Dict, List

import numpy as np

from .contracts import ObjectSphere


@dataclass
class MemoryEntry:
    object_id: str
    label: str
    center: tuple
    radius: float
    confidence: float
    observations: int
    first_seen: float
    last_seen: float
    source: str

    def as_sphere(self):
        return ObjectSphere(
            label=self.label,
            center=self.center,
            radius=self.radius,
            confidence=self.confidence,
            frame_id="world",
            source=self.source,
        )


class SemanticSceneMemory:
    """Associate repeated observations and retain conservative world-space volumes."""

    def __init__(self, association_distance_m=1.0, ttl_sec=120.0):
        self.association_distance_m = float(association_distance_m)
        self.ttl_sec = float(ttl_sec)
        self._entries: Dict[str, MemoryEntry] = {}
        self._next_id = 1

    @staticmethod
    def _normalized_label(label):
        normalized = " ".join(str(label or "").strip().lower().split())
        tokens = re.findall(r"[a-z0-9]+", normalized)
        if not tokens:
            return normalized
        relation_words = {
            "at",
            "behind",
            "beside",
            "by",
            "in",
            "near",
            "next",
            "of",
            "on",
            "under",
            "with",
            "who",
        }
        for index, token in enumerate(tokens):
            if token in relation_words:
                tokens = tokens[:index]
                break
        if not tokens:
            return normalized
        head = tokens[-1]
        irregular = {"people": "person", "persons": "person", "men": "man", "women": "woman"}
        if head in irregular:
            return irregular[head]
        if head.endswith("ies") and len(head) > 4:
            return head[:-3] + "y"
        if head.endswith("s") and not head.endswith("ss") and len(head) > 3:
            return head[:-1]
        return head

    def _candidate(self, sphere, max_distance_m=None):
        label = self._normalized_label(sphere.label)
        center = np.asarray(sphere.center, dtype=float)
        gate = self.association_distance_m if max_distance_m is None else float(max_distance_m)
        if not math.isfinite(gate) or gate < 0.0:
            raise ValueError("association distance must be finite and non-negative")
        candidates = []
        for entry in self._entries.values():
            if self._normalized_label(entry.label) != label:
                continue
            distance = float(np.linalg.norm(center - np.asarray(entry.center, dtype=float)))
            if distance <= gate:
                candidates.append((distance, entry))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def match(self, sphere: ObjectSphere, timestamp=None, max_distance_m=None):
        """Return a copy of the closest compatible entry within the center gate."""
        if sphere.frame_id != "world":
            raise ValueError("scene memory accepts only world-frame spheres")
        if timestamp is not None:
            self.prune(timestamp)
        candidate = self._candidate(sphere, max_distance_m=max_distance_m)
        return None if candidate is None else replace(candidate)

    def update(self, sphere: ObjectSphere, timestamp, dynamic=False, max_distance_m=None):
        if sphere.frame_id != "world":
            raise ValueError("scene memory accepts only world-frame spheres")
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        existing = self._candidate(sphere, max_distance_m=max_distance_m)
        if existing is None:
            object_id = "obj-%04d" % self._next_id
            self._next_id += 1
            entry = MemoryEntry(
                object_id=object_id,
                label=sphere.label,
                center=tuple(sphere.center),
                radius=float(sphere.radius),
                confidence=float(sphere.confidence),
                observations=1,
                first_seen=timestamp,
                last_seen=timestamp,
                source=sphere.source,
            )
            self._entries[object_id] = entry
            return entry

        old_center = np.asarray(existing.center, dtype=float)
        observed_center = np.asarray(sphere.center, dtype=float)
        if dynamic:
            fused_center = observed_center
            fused_radius = float(sphere.radius)
        else:
            old_weight = max(1.0, float(existing.observations) * max(0.1, existing.confidence))
            new_weight = max(0.1, float(sphere.confidence))
            fused_center = (old_weight * old_center + new_weight * observed_center) / (old_weight + new_weight)
            fused_radius = max(
                float(np.linalg.norm(old_center - fused_center)) + existing.radius,
                float(np.linalg.norm(observed_center - fused_center)) + sphere.radius,
            )
        existing.center = tuple(float(value) for value in fused_center)
        existing.radius = float(fused_radius)
        existing.confidence = max(existing.confidence, float(sphere.confidence))
        existing.observations += 1
        existing.last_seen = timestamp
        existing.source = sphere.source or existing.source
        return existing

    def prune(self, timestamp):
        timestamp = float(timestamp)
        expired = [
            object_id
            for object_id, entry in self._entries.items()
            if self.ttl_sec >= 0.0 and timestamp - entry.last_seen > self.ttl_sec
        ]
        for object_id in expired:
            del self._entries[object_id]
        return expired

    def snapshot(self, timestamp=None) -> List[ObjectSphere]:
        return [entry.as_sphere() for entry in self.snapshot_entries(timestamp)]

    def snapshot_entries(self, timestamp=None) -> List[MemoryEntry]:
        """Return stable-ID copies ordered by object ID."""
        if timestamp is not None:
            self.prune(timestamp)
        return [replace(self._entries[key]) for key in sorted(self._entries)]

    def to_json(self, timestamp=None):
        entries = self.snapshot_entries(timestamp)
        payload = [
            {
                "object_id": entry.object_id,
                "label": entry.label,
                "center": list(entry.center),
                "safety_radius": entry.radius,
                "confidence": entry.confidence,
                "frame_id": "world",
                "source": entry.source,
            }
            for entry in entries
        ]
        return json.dumps(payload, ensure_ascii=True, sort_keys=True)
