#!/usr/bin/env python3
import argparse
import asyncio
import logging
from typing import Any, List

from commonwealth.utils.apis import GenericErrorHandlingRoute, PrettyJSONResponse
from commonwealth.utils.logs import InterceptHandler, get_new_log_path
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi_versioning import VersionedFastAPI, version
from loguru import logger
from uvicorn import Config, Server

from pingmanager import PingManager
from pingprober import PingProber
from portwatcher import PortWatcher
from typedefs import PingDeviceDescriptorModel

SERVICE_NAME = "ping"

app = FastAPI(
    title="Ping Manager API",
    description="Ping Manager is responsible for managing Ping devices connected to BlueOS.",
    default_response_class=PrettyJSONResponse,
    debug=True,
)
app.router.route_class = GenericErrorHandlingRoute
logger.info("Starting Ping Service.")

# TODO: move to singleton
ping_manager = PingManager()


@app.get("/sensors", response_model=List[PingDeviceDescriptorModel], summary="Current sensors detected.")
@version(1, 0)
def get_sensors() -> Any:
    devices = ping_manager.devices()
    logger.debug(f"Sensors available: {devices}")
    return [PingDeviceDescriptorModel.from_descriptor(device) for device in ping_manager.devices()]


app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)


@app.get("/")
async def root() -> Any:
    html_content = """
    <html>
        <head>
            <title>Ping Service</title>
        </head>
    </html>
    """
    return HTMLResponse(content=html_content, status_code=200)


async def sensor_manager() -> None:
    ping_prober = PingProber()
    port_watcher = PortWatcher(probe_callback=ping_prober.probe)
    port_watcher.set_port_post_callback(ping_manager.stop_driver_at_port)

    ping_prober.on_ping_found(ping_manager.launch_driver_instance)

    await port_watcher.start_watching()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ping Service for Bluerobotics BlueOS")
    args = parser.parse_args()

    logging.basicConfig(handlers=[InterceptHandler()], level=0)
    logger.add(get_new_log_path(SERVICE_NAME))

    loop = asyncio.new_event_loop()

    # Running uvicorn with log disabled so loguru can handle it
    config = Config(app=app, loop=loop, host="0.0.0.0", port=9110, log_config=None)
    server = Server(config)

    loop.create_task(sensor_manager())
    loop.run_until_complete(server.serve())
