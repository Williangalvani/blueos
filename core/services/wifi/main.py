#!/usr/bin/env python3

import argparse
import asyncio
import logging
import os
import sys
from pathlib import Path
from typing import Any, List

from commonwealth.utils.apis import (
    GenericErrorHandlingRoute,
    PrettyJSONResponse,
    StackedHTTPException,
)
from commonwealth.utils.logs import InterceptHandler, init_logger
from fastapi import FastAPI, HTTPException, status
from fastapi.staticfiles import StaticFiles
from fastapi_versioning import VersionedFastAPI, version
from loguru import logger
from tabulate import tabulate
from uvicorn import Config, Server

from exceptions import BusyError
from typedefs import (
    HotspotStatus,
    SavedWifiNetwork,
    ScannedWifiNetwork,
    WifiCredentials,
)
from wifi_handlers.AbstractWifiHandler import AbstractWifiManager
from wifi_handlers.networkmanager.networkmanager import NetworkManagerWifi
from wifi_handlers.wpa_supplicant.WifiManager import WifiManager

FRONTEND_FOLDER = Path.joinpath(Path(__file__).parent.absolute(), "frontend")
SERVICE_NAME = "wifi-manager"

logging.basicConfig(handlers=[InterceptHandler()], level=0)
init_logger(SERVICE_NAME)

logger.info("Starting Wifi Manager.")
wpa_manager = WifiManager()
network_manager = NetworkManagerWifi()
wifi_manager: AbstractWifiManager = wpa_manager


app = FastAPI(
    title="WiFi Manager API",
    description="WiFi Manager is responsible for managing WiFi connections on BlueOS.",
    default_response_class=PrettyJSONResponse,
)
app.router.route_class = GenericErrorHandlingRoute


@app.get("/status", summary="Retrieve status of wifi manager.")
@version(1, 0)
async def network_status() -> Any:
    logger.info("Calling wifi_manager.status()")
    wifi_status = await wifi_manager.status()
    for line in tabulate(list(wifi_status.items())).splitlines():
        logger.info(line)
    return wifi_status


@app.get("/scan", response_model=List[ScannedWifiNetwork], summary="Retrieve available wifi networks.")
@version(1, 0)
async def scan() -> Any:
    logger.info("Calling wifi_manager.get_wifi_available()")
    try:
        available_networks = await wifi_manager.get_wifi_available()
        return available_networks
    except BusyError as error:
        raise StackedHTTPException(status_code=status.HTTP_425_TOO_EARLY, error=error) from error


@app.get("/saved", response_model=List[SavedWifiNetwork], summary="Retrieve saved wifi networks.")
@version(1, 0)
async def saved() -> Any:
    logger.info("Calling wifi_manager.get_saved_wifi_network()")
    saved_networks = await wifi_manager.get_saved_wifi_network()
    return saved_networks


@app.post("/connect", summary="Connect to wifi network.")
@version(1, 0)
async def connect(credentials: WifiCredentials, hidden: bool = False) -> Any:
    logger.info(f"Calling wifi_manager.try_connect_to_network(credentials={credentials}, hidden={hidden})")
    await wifi_manager.try_connect_to_network(credentials, hidden)


@app.post("/remove", summary="Remove saved wifi network.")
@version(1, 0)
async def remove(ssid: str) -> Any:
    logger.info(f"Processing remove request for SSID: {ssid}")
    try:
        logger.info("Calling wifi_manager.get_saved_wifi_network()")
        await wifi_manager.remove_network(ssid)
    except StopIteration as error:
        logger.info(f"Network '{ssid}' is unknown.")
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=f"Network '{ssid}' not saved.") from error
    logger.info(f"Successfully removed '{ssid}'.")


@app.get("/disconnect", summary="Disconnect from wifi network.")
@version(1, 0)
async def disconnect() -> Any:
    logger.info("Calling wifi_manager.disconnect()")
    await wifi_manager.disconnect()
    logger.info("Successfully disconnected from network.")


@app.get("/hotspot", summary="Get hotspot state.")
@version(1, 0)
def hotspot_state() -> Any:
    logger.info("Calling wifi_manager.hotspot_is_running()")
    return wifi_manager.hotspot_is_running()


@app.get("/hotspot_extended_status", summary="Get extended hotspot status.")
@version(1, 0)
async def hotspot_extended_state() -> HotspotStatus:
    logger.info("Calling wifi_manager.supports_hotspot() and wifi_manager.hotspot_is_running()")
    return HotspotStatus(
        supported=await wifi_manager.supports_hotspot(), enabled=await wifi_manager.hotspot_is_running()
    )


@app.post("/hotspot", summary="Enable/disable hotspot.")
@version(1, 0)
async def toggle_hotspot(enable: bool) -> Any:
    if enable:
        logger.info("Calling wifi_manager.enable_hotspot()")
        return await wifi_manager.enable_hotspot()
    logger.info("Calling wifi_manager.disable_hotspot()")
    return await wifi_manager.disable_hotspot()


@app.post("/smart_hotspot", summary="Enable/disable smart-hotspot.")
@version(1, 0)
def toggle_smart_hotspot(enable: bool) -> Any:
    if enable:
        logger.info("Calling wifi_manager.enable_smart_hotspot()")
        wifi_manager.enable_smart_hotspot()
        return
    logger.info("Calling wifi_manager.disable_smart_hotspot()")
    wifi_manager.disable_smart_hotspot()


@app.get("/smart_hotspot", summary="Check if smart-hotspot is enabled.")
@version(1, 0)
def check_smart_hotspot() -> Any:
    logger.info("Calling wifi_manager.is_smart_hotspot_enabled()")
    return wifi_manager.is_smart_hotspot_enabled()


@app.post("/hotspot_credentials", summary="Update hotspot credentials.")
@version(1, 0)
async def set_hotspot_credentials(credentials: WifiCredentials) -> Any:
    logger.info(f"Calling wifi_manager.set_hotspot_credentials(credentials={credentials})")
    await wifi_manager.set_hotspot_credentials(credentials)


@app.get("/hotspot_credentials", summary="Get hotspot credentials.")
@version(1, 0)
def get_hotspot_credentials() -> Any:
    logger.info("Calling wifi_manager.hotspot_credentials()")
    return wifi_manager.hotspot_credentials()


app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)
app.mount("/", StaticFiles(directory=str(FRONTEND_FOLDER), html=True))


if __name__ == "__main__":
    if os.geteuid() != 0:
        logger.error("You need root privileges to run this script.\nPlease try again using **sudo**. Exiting.")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Abstraction CLI for WifiManager configuration.")
    candidates = [wpa_manager, network_manager]

    for implementation in candidates:
        implementation.add_arguments(parser)
    # we need to configure all arguments before parsing them, hence two loops
    for implementation in candidates:
        implementation.configure(parser.parse_args())

    loop = asyncio.new_event_loop()

    # Running uvicorn with log disabled so loguru can handle it
    config = Config(app=app, loop=loop, host="0.0.0.0", port=9000, log_config=None)
    server = Server(config)
    for implementation in candidates:
        if implementation.can_work():
            logger.info(f"Using {implementation} as wifi manager.")
            implementation.start(loop)
            wifi_manager = implementation

    loop.run_until_complete(server.serve())
