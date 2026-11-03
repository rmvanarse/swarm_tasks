"""Generic `color` command handler (stub of the removed upstream helper):
sets the LED color of a UAV that has an async `set_led_color()` method."""

from __future__ import annotations

from typing import Any, Callable

__all__ = ("create_color_command_handler",)


def create_color_command_handler() -> Callable[..., Any]:
    async def _color_command_handler(driver, uav, color: str | None = None, *args):
        from flockwave.server.utils import color_to_rgb8_triplet  # noqa: F401

        if color is None or str(color).lower() in ("off", "none"):
            await uav.set_led_color(None)
        else:
            await uav.set_led_color(color)
        return "Color command sent"

    return _color_command_handler
