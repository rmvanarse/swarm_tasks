"""Stub types used by the mavlink extension (see ext/show/__init__.py)."""

from typing import Any, Awaitable, Callable, TypedDict


class ShowExtensionAPI(TypedDict):
    get_clock: Callable[[], Any]
    get_configuration: Callable[[], Any]
    get_light_configuration: Callable[[], Any]
    encode_show: Callable[[Any], Awaitable[bytes]]
