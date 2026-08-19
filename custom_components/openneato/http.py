"""HTTP views for the replay card.

The card needs the calibrated floorplan image, but the config entry stores a
server-side filesystem path the browser can't reach. This view serves that
one file, authenticated like any other HA endpoint.
"""

from __future__ import annotations

import logging

from aiohttp import web

from homeassistant.components.http import HomeAssistantView
from homeassistant.core import HomeAssistant

from .const import DOMAIN

_LOGGER = logging.getLogger(__name__)


class OpenNeatoMapView(HomeAssistantView):
    """Serve the floor plan built from the robot's LIDAR.

    Rendered on demand from the accumulated grid rather than kept on disk, so
    it always reflects every cleaning merged so far.
    """

    url = "/api/openneato/map/{entry_id}"
    name = "api:openneato:map"
    requires_auth = True

    async def get(self, request: web.Request, entry_id: str) -> web.Response:
        """Return the generated plan as a PNG."""
        hass: HomeAssistant = request.app["hass"]
        stored = hass.data.get(DOMAIN, {}).get(entry_id)
        mapper = stored.get("mapper") if isinstance(stored, dict) else None
        if mapper is None:
            return web.Response(status=404, text="LIDAR mapping is not enabled")

        result = await mapper.async_render()
        if result is None:
            return web.Response(status=404, text="No map has been built yet")

        png, _calibration = result
        return web.Response(
            body=png,
            content_type="image/png",
            # The map only changes when a cleaning finishes, and the card
            # cache-busts on the session count.
            headers={"Cache-Control": "private, max-age=300"},
        )

