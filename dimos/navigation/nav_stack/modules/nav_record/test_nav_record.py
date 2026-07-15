# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Public message types shared by navigation modules."""

from typing import get_args, get_type_hints

from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.modules.nav_record.nav_record import NavRecord


def _stream_message_type(module: type, port: str) -> type:
    return get_args(get_type_hints(module)[port])[0]


def test_nav_record_uses_canonical_bool_stream_type() -> None:
    assert _stream_message_type(NavRecord, "stop_movement") is Bool
    assert _stream_message_type(NavRecord, "goal_reached") is Bool
    assert _stream_message_type(MovementManager, "stop_movement") is Bool
    assert _stream_message_type(DanHolonomicTC, "stop_movement") is Bool
    assert _stream_message_type(DanHolonomicTC, "goal_reached") is Bool
