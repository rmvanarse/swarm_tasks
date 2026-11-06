"""Placeholder for the drone-show extension, which this fork removed.

The mavlink extension still imports show types and declares a dependency on
the "show" extension, so it cannot load without something here. This stub
exports the small API surface mavlink calls and always reports "no show
configured". Nothing in this fork uploads or runs light shows.
"""

from .types import ShowExtensionAPI

__all__ = ("dependencies", "description", "exports", "run")

dependencies = ()
description = "Stub for the removed drone-show extension"


def get_clock():
    return None


def get_configuration():
    return None


def get_light_configuration():
    return None


async def encode_show(show):
    raise NotImplementedError("drone shows are not supported in this fork")


exports: ShowExtensionAPI = {
    "get_clock": get_clock,
    "get_configuration": get_configuration,
    "get_light_configuration": get_light_configuration,
    "encode_show": encode_show,
}


async def run(app, configuration, logger):
    from trio import sleep_forever

    await sleep_forever()
