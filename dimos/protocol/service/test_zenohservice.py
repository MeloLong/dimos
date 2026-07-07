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

from __future__ import annotations

import pickle

import pytest

from dimos.protocol.pubsub.impl.zenohqos import ZenohQoS
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohService, ZenohSessionPool


@pytest.fixture()
def session_pool():
    """Provide a fresh, isolated session pool and close it after the test."""
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


def test_different_modes_produce_different_keys() -> None:
    peer = ZenohConfig(mode="peer")
    client = ZenohConfig(mode="client")
    assert peer.session_key != client.session_key


def test_qos_does_not_change_session_key() -> None:
    # Sessions are shared across QoS configs; QoS applies per publisher.
    with_qos = ZenohConfig(qos=(ZenohQoS(key="dimos/x", congestion_control="block"),))
    assert with_qos.session_key == ZenohConfig().session_key


def test_start_creates_session(session_pool) -> None:
    svc = ZenohService(session_pool=session_pool)
    svc.start()
    assert svc.session is not None


def test_two_services_share_session(session_pool) -> None:
    svc1 = ZenohService(session_pool=session_pool)
    svc2 = ZenohService(session_pool=session_pool)
    svc1.start()
    svc2.start()
    assert svc1.session is svc2.session


def test_stop_does_not_close_shared_session(session_pool) -> None:
    svc1 = ZenohService(session_pool=session_pool)
    svc2 = ZenohService(session_pool=session_pool)
    svc1.start()
    svc2.start()
    svc1.stop()
    # svc2's session should still be valid
    assert svc2.session is not None


def test_session_before_start_raises(session_pool) -> None:
    svc = ZenohService(session_pool=session_pool)
    with pytest.raises(RuntimeError, match="not initialized"):
        _ = svc.session


def test_start_is_idempotent(session_pool) -> None:
    svc = ZenohService(session_pool=session_pool)
    svc.start()
    session1 = svc.session
    svc.start()
    session2 = svc.session
    assert session1 is session2


def test_started_service_is_picklable(session_pool) -> None:
    # Modules cross the worker pipe via pickle; a live session must not block it.
    svc = ZenohService(session_pool=session_pool)
    svc.start()
    restored = pickle.loads(pickle.dumps(svc))
    # Live handles are dropped; the restored service reopens on start().
    assert restored._session is None
    with pytest.raises(RuntimeError, match="not initialized"):
        _ = restored.session
    restored.start()
    assert restored.session is not None


def test_getstate_does_not_mutate_live_instance(session_pool) -> None:
    # __getstate__ must copy, not strip handles off the live object (regression:
    # a shared __getstate__ that popped from the live __dict__ broke shutdown).
    svc = ZenohService(session_pool=session_pool)
    svc.start()
    pickle.dumps(svc)
    assert svc._session is not None
    assert svc._session_pool is session_pool


def test_started_zenoh_rpc_survives_pickle(session_pool) -> None:
    # The composed RPC stack (PubSubRPCMixin over ZenohPubSubBase) is what rides
    # the worker pipe. Pickling it must neither hit the live session nor strip
    # _call_thread_pool_lock off the live instance, then it must still stop().
    from dimos.protocol.rpc.pubsubrpc import ZenohRPC

    rpc = ZenohRPC(session_pool=session_pool)
    rpc.start()
    rpc._get_call_thread_pool()  # populate the otherwise-lazy thread pool
    pickle.dumps(rpc)
    assert rpc._session is not None
    assert hasattr(rpc, "_call_thread_pool_lock")
    rpc.stop()  # would raise AttributeError if the lock had been stripped
