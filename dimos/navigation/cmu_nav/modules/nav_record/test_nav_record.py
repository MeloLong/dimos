from pathlib import Path

from dimos.navigation.cmu_nav.modules.nav_record.nav_record import NavRecordConfig


def test_nav_record_config_accepts_path_db_path() -> None:
    db_path = Path("recordings/m20-nav.db")

    config = NavRecordConfig(db_path=db_path)

    assert config.db_path == db_path.resolve()
    assert config.root_frame == "world"
