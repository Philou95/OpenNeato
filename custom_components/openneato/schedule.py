"""Shared shape of the robot's weekly cleaning schedule.

Both the `time` entities (when a slot runs) and the `switch` entities (whether
it runs) are generated from this, so the two can never drift apart on the day
order or on a field name.

The firmware numbers its days **Monday=0 .. Sunday=6**, which is not the C
library's Sunday=0 — `Scheduler::toSchedDay()` converts between them. Anything
here that assumed the C order would clean on the wrong day, quietly, so the
mapping is written out rather than computed.

Slot 1 has no infix in its field names and slot 2 carries `Slot1`, an
off-by-one in the firmware's own naming that is easy to trip over:

    slot 0 -> sched3Hour, sched3Min, sched3On
    slot 1 -> sched3Slot1Hour, sched3Slot1Min, sched3Slot1On
"""

from __future__ import annotations

SCHED_SLOTS = 2  # SCHEDULE_SLOTS_PER_DAY in the firmware's config.h

# (index, key used in entity ids, display name)
SCHED_DAYS: tuple[tuple[int, str, str], ...] = (
    (0, "monday", "Monday"),
    (1, "tuesday", "Tuesday"),
    (2, "wednesday", "Wednesday"),
    (3, "thursday", "Thursday"),
    (4, "friday", "Friday"),
    (5, "saturday", "Saturday"),
    (6, "sunday", "Sunday"),
)


def slot_fields(day_index: int, slot: int) -> dict[str, str]:
    """Settings field names for one day/slot."""
    infix = "" if slot == 0 else f"Slot{slot}"
    return {
        "hour": f"sched{day_index}{infix}Hour",
        "minute": f"sched{day_index}{infix}Min",
        "on": f"sched{day_index}{infix}On",
    }
