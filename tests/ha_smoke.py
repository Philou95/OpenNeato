"""Run in a separate process with real Home Assistant, without offline stubs."""
import asyncio
import importlib
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from homeassistant.core import HomeAssistant

integration = importlib.import_module("custom_components.openneato")
for path in (Path(__file__).resolve().parents[1] / "custom_components/openneato").glob("*.py"):
    importlib.import_module(f"custom_components.openneato.{path.stem}")


async def main():
    with TemporaryDirectory() as directory:
        hass = HomeAssistant(directory)
        # Platform loading is an HA service boundary. The core object and all
        # imported integration APIs remain real; no device entities are started.
        hass.config_entries = MagicMock()
        hass.config_entries.async_shutdown = AsyncMock()
        api = SimpleNamespace(
            get_firmware_version=AsyncMock(return_value={"version": "test"}),
            get_robot_version=AsyncMock(return_value={"serialNumber": "offline", "modelName": "D7"}),
        )
        coordinator = SimpleNamespace(async_config_entry_first_refresh=AsyncMock())
        entry = SimpleNamespace(
            data={"host": "offline.invalid"}, options={integration.CONF_MAP_ENABLED: False},
            entry_id="offline", async_on_unload=MagicMock(), add_update_listener=MagicMock(),
        )
        with (
            patch.object(integration, "async_get_clientsession", return_value=None),
            patch.object(integration, "OpenNeatoApiClient", return_value=api),
            patch.object(integration, "OpenNeatoCoordinator", return_value=coordinator),
            patch.object(integration, "_async_register_frontend", new=AsyncMock()),
            patch.object(hass.config_entries, "async_forward_entry_setups", new=AsyncMock()) as forward,
            patch.object(hass.config_entries, "async_unload_platforms", new=AsyncMock(return_value=True)),
        ):
            assert await integration.async_setup_entry(hass, entry)
            assert hass.data["openneato"]["offline"]["api"] is api
            forward.assert_awaited_once()
            assert await integration.async_unload_entry(hass, entry)
            assert "offline" not in hass.data["openneato"]
        await hass.async_stop()
    print("All integration modules imported; setup/unload passed with real HA APIs")


if __name__ == "__main__":
    asyncio.run(main())
