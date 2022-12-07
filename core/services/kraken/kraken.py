import asyncio
import json
from typing import Any, AsyncGenerator, Dict, List, Optional, cast

import aiodocker
import aiohttp
from aiodocker.docker import DockerContainer
from commonwealth.settings.manager import Manager
from commonwealth.utils.apis import StackedHTTPException
from fastapi import status
from loguru import logger

from exceptions import ContainerDoesNotExist
from settings import Extension, SettingsV1

REPO_URL = "https://bluerobotics.github.io/BlueOS-Extensions-Repository/manifest.json"
SERVICE_NAME = "Kraken"


class Kraken:
    def __init__(self) -> None:
        self.load_settings()
        self.running_containers: List[DockerContainer] = []
        self.should_run = True
        self._client: Optional[aiodocker.Docker] = None

    @property
    def client(self) -> aiodocker.Docker:
        if self._client is None:
            self._client = aiodocker.Docker()
        return self._client

    async def run(self) -> None:
        while self.should_run:
            await asyncio.sleep(5)
            running_containers: List[DockerContainer] = await self.client.containers.list(  # type: ignore
                filter='{"status": ["running"]}'
            )
            self.running_containers = running_containers

            for extension in self.settings.extensions:
                await self.check(extension)

    async def start_extension(self, extension: Extension) -> None:
        config = extension.settings()
        config["Image"] = extension.fullname()
        logger.info(f"Starting extension '{extension.fullname()}'")
        try:
            await self.client.images.pull(extension.fullname())
        except aiodocker.exceptions.DockerError:  # raised if we are offline
            logger.info("unable to pull a new image, attempting to continue with a local one")
        container = await self.client.containers.create_or_replace(name=extension.container_name(), config=config)  # type: ignore
        await container.start()

    async def check(self, extension: Extension) -> None:
        if not extension.enabled:
            return
        if not extension.is_valid():
            logger.warning(f"{extension.identifier} is invalid, removing it")
            try:
                await self.uninstall_extension(extension.identifier)
            except ContainerDoesNotExist:
                logger.warning(f"container for extension {extension.identifier} was not up?")
            return

        extension_name = extension.container_name()
        # Names is a list of of lists like ["[['/blueos-core'], ..."]
        # We assume which container has only one tag, and remove '/' using the [1:] slicing
        if not extension.enabled:
            return
        if not any(container["Names"][0][1:] == extension_name for container in self.running_containers):
            await self.start_extension(extension)

    def load_settings(self) -> None:
        self.manager = Manager(SERVICE_NAME, SettingsV1)
        self.settings = self.manager.settings

    async def fetch_manifest(self) -> Any:
        async with aiohttp.ClientSession() as session:
            async with session.get(REPO_URL) as resp:
                if resp.status != 200:
                    print(f"Error status {resp.status}")
                    raise Exception(f"Could not fetch manifest file: reponse status : {resp.status}")
                return await resp.json(content_type=None)

    async def get_configured_extensions(self) -> List[Extension]:
        return cast(List[Extension], self.settings.extensions)

    async def install_extension(self, extension: Any) -> AsyncGenerator[bytes, None]:
        # Remove older entry if it exists
        self.settings.extensions = [
            old_extension
            for old_extension in self.settings.extensions
            if old_extension.identifier != extension.identifier
        ]
        try:
            await self.remove(extension.identifier, False)
        except Exception as e:
            # this will fail if the container is not installed, we don't mind it
            logger.info(e)
        new_extension = Extension(
            identifier=extension.identifier,
            name=extension.name,
            docker=extension.docker,
            tag=extension.tag,
            permissions=extension.permissions,
            enabled=extension.enabled,
            user_permissions=extension.user_permissions,
        )
        self.settings.extensions.append(new_extension)
        self.manager.save()
        try:
            async for line in self.client.images.pull(
                f"{extension.docker}:{extension.tag}", repo=extension.docker, tag=extension.tag, stream=True
            ):
                yield json.dumps(line).encode("utf-8")
        except Exception as error:
            raise StackedHTTPException(status_code=status.HTTP_404_NOT_FOUND, error=error) from error

    def container_from_identifier(self, extension_identifier: str) -> str:
        extension = [
            extension for extension in self.settings.extensions if extension.identifier == extension_identifier
        ]
        if not extension:
            raise Exception(f"could not find container for {extension_identifier}, is It running?")
        return str(extension[0].container_name())

    async def kill(self, container_name: str) -> None:
        logger.info(f"Killing {container_name}")
        container = await self.client.containers.list(filters={"name": {container_name: True}})  # type: ignore
        if container:
            await container[0].kill()

    async def remove(self, extension_identifier: str, delete: bool = True) -> None:
        logger.info(f"Removing extension {extension_identifier}")
        container_name = self.container_from_identifier(extension_identifier)
        container = await self.client.containers.list(filters={"name": {container_name: True}})  # type: ignore
        if not container:
            raise ContainerDoesNotExist(f"Unable remove {container_name}. container not found")
        image = container[0]["Image"]
        await self.kill(container_name)
        await container[0].delete()
        if delete:
            logger.info(f"Removing {image}")
            await self.client.images.delete(image, force=False, noprune=False)

    async def uninstall_extension(self, extension_identifier: str) -> None:
        try:
            await self.remove(extension_identifier)
        except Exception as e:
            logger.warning(f"Unable to remove container {e}")
        self.settings.extensions = [
            extension for extension in self.settings.extensions if extension.identifier != extension_identifier
        ]
        self.manager.save()

    async def disable_extension(self, extension_identifier: str) -> None:
        current_extension_list = [
            extension for extension in self.settings.extensions if extension.identifier == extension_identifier
        ]
        if not current_extension_list:
            raise Exception(f"Extension not found: {extension_identifier}")

        current_extension = current_extension_list[0]
        logger.info(f"disabling: {extension_identifier}")

        # replace extension with a copy of it, but disabled
        current_extension.enabled = False
        self.settings.extensions = [
            extension for extension in self.settings.extensions if extension.identifier != extension_identifier
        ]
        self.settings.extensions.append(current_extension)
        logger.info(f"{current_extension}")
        await self.remove(extension_identifier, False)
        self.manager.save()

    async def enable_extension(self, extension_identifier: str) -> None:
        current_extension_list = [
            extension for extension in self.settings.extensions if extension.identifier == extension_identifier
        ]
        if not current_extension_list:
            raise Exception(f"Extension not found: {extension_identifier}")

        current_extension = current_extension_list[0]
        logger.info(f"enabling: {extension_identifier}")

        # replace extension with a copy of it, but disabled
        current_extension.enabled = True
        self.settings.extensions = [
            extension for extension in self.settings.extensions if extension.identifier != extension_identifier
        ]
        self.settings.extensions.append(current_extension)
        self.manager.save()

    async def restart_extension(self, extension_identifier: str) -> None:
        await self.remove(extension_identifier, False)

    async def list_containers(self) -> List[DockerContainer]:
        containers: List[DockerContainer] = await self.client.containers.list(filter='{"status": ["running"]}')  # type: ignore
        return containers

    async def load_logs(self, container_name: str) -> List[str]:
        containers = await self.client.containers.list(filters={"name": {container_name: True}})  # type: ignore
        if not containers:
            raise Exception(f"Container not found: {container_name}")
        return cast(List[str], await containers[0].log(stdout=True, stderr=True))

    async def load_stats(self) -> Dict[str, Any]:
        containers = await self.client.containers.list()  # type: ignore
        container_stats = [(await container.stats(stream=False))[0] for container in containers]
        result = {}
        for stats in container_stats:
            # Based over: https://github.com/docker/cli/blob/v20.10.20/cli/command/container/stats_helpers.go
            cpu_percent = 0

            previous_cpu = stats["precpu_stats"]["cpu_usage"]["total_usage"]
            previous_system_cpu = stats["precpu_stats"]["system_cpu_usage"]

            cpu_total = stats["cpu_stats"]["cpu_usage"]["total_usage"]
            cpu_delta = cpu_total - previous_cpu

            cpu_system = stats["cpu_stats"]["system_cpu_usage"]
            system_delta = cpu_system - previous_system_cpu

            if system_delta > 0.0:
                cpu_percent = (cpu_delta / system_delta) * 100.0

            try:
                memory_usage = 100 * stats["memory_stats"]["usage"] / stats["memory_stats"]["limit"]
            except KeyError:
                memory_usage = "N/A"

            name = stats["name"].replace("/", "")

            result[name] = {
                "cpu": cpu_percent,
                "memory": memory_usage,
            }
        return result

    async def stop(self) -> None:
        self.should_run = False
