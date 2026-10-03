"""Compact beacon message schema (JSON-encoded stand-in for the RF payload).

Fields follow the challenge spec: identifier, WHAT, WHERE, distance, WHEN, version.
Real hardware would pack this into a few bytes for LoRa/RF broadcast; JSON here
keeps the simulation's data flow transparent and easy to inspect on the bus.
"""
from dataclasses import dataclass, asdict
import json
import time

FRESH_S = 5 * 60
AGING_S = 15 * 60


@dataclass
class Beacon:
    id: str
    what: str          # hazard | victim | junction | exit ...
    x: float            # local writer-frame coordinates (metres from entrance)
    y: float
    distance: float      # metres from previous beacon / entrance
    when: float           # unix timestamp, this is what makes it age
    version: int = 1

    def age_s(self, now: float = None) -> float:
        now = now if now is not None else time.time()
        return max(0.0, now - self.when)

    def freshness(self, now: float = None) -> str:
        age = self.age_s(now)
        if age < FRESH_S:
            return "fresh"
        if age < AGING_S:
            return "aging"
        return "stale"

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @staticmethod
    def from_dict(d: dict) -> "Beacon":
        return Beacon(**d)
