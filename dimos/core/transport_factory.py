# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Backend-agnostic transport construction."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.transport import (
    LCMTransport,
    ZenohTransport,
    pLCMTransport,
    pZenohTransport,
)
from dimos.protocol.rpc.pubsubrpc import LCMRPC, ZenohRPC
from dimos.protocol.tf.tf import LCMTF, ZenohTF

if TYPE_CHECKING:
    from dimos.core.transport import PubSubTransport
    from dimos.protocol.rpc.spec import RPCSpec
    from dimos.protocol.tf.tf import TFSpec


def transport_topic(name: str, g: GlobalConfig = global_config) -> str:
    """Map a logical channel name to the active backend's topic string.

    LCM channels are leading-slash paths (`/foo`).

    Zenoh key expressions can't start with `/` and are namespaced under `dimos`.
    """
    if g.transport == "zenoh":
        return "dimos/" + name.lstrip("/")
    return name if name.startswith("/") else "/" + name


def make_transport(
    name: str, msg_type: type | None = None, *, g: GlobalConfig = global_config
) -> PubSubTransport[Any]:
    """Construct the active-backend pub/sub transport for a logical channel.

    A pickled (self-describing) transport is used when no `msg_type` is given or
    the type has no `lcm_encode`. Otherwise a typed transport is used.

    The factory covers pub/sub backends only; per-backend channel tuning lives
    in `GlobalConfig` overlay fields (`zenoh_qos` maps key-expr patterns to
    Zenoh publisher QoS) rather than per-call-site parameters. A future
    non-pubsub backend (e.g. TCP) would add its own overlay fields the same way.
    """

    use_pickled = msg_type is None or getattr(msg_type, "lcm_encode", None) is None
    topic = transport_topic(name, g)
    if use_pickled:
        return pZenohTransport(topic) if g.transport == "zenoh" else pLCMTransport(topic)
    assert msg_type is not None  # not use_pickled implies a typed msg_type
    return (
        ZenohTransport(topic, msg_type) if g.transport == "zenoh" else LCMTransport(topic, msg_type)
    )


def apply_transport_arg(argv: list[str], *, g: GlobalConfig = global_config) -> None:
    """Apply a `--transport <lcm|zenoh>` / `--transport=...` override from argv.

    Lets standalone CLIs (`humancli`, `agentspy`, `dtop`) flip the backend
    explicitly. Without it they follow `DIMOS_TRANSPORT` / `.env` via the
    global config, which is the single switch shared with the `dimos` process.
    """
    for i, arg in enumerate(argv):
        value: str | None = None
        if arg.startswith("--transport="):
            value = arg.split("=", 1)[1]
        elif arg == "--transport" and i + 1 < len(argv):
            value = argv[i + 1]
        if value is not None:
            g.update(transport=value)


def rpc_backend(g: GlobalConfig = global_config) -> type[RPCSpec]:
    """Return the RPC class (`LCMRPC` or `ZenohRPC`) for the active backend."""
    return ZenohRPC if g.transport == "zenoh" else LCMRPC


def tf_backend(g: GlobalConfig = global_config) -> type[TFSpec]:
    """Return the TF class (`LCMTF` or `ZenohTF`) for the active backend."""
    return ZenohTF if g.transport == "zenoh" else LCMTF
