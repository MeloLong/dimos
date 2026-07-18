from pathlib import Path

from dimos.navigation.nav_stack.modules.nav_record.nav_record import NavRecordConfig


def test_nav_record_config_accepts_path_db_path() -> None:
    db_path = Path("recordings/m20-nav.db")

    assert NavRecordConfig(db_path=db_path).db_path == db_path
