import json
import logging
import pathlib
from dataclasses import asdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Union

import aiodocker
import appdirs
from aiohttp import web

from utils.dockerhub import TagFetcher, TagMetadata

DOCKER_CONFIG_PATH = pathlib.Path(appdirs.user_config_dir("bootstrap"), "startup.json")

current_folder = pathlib.Path(__file__).parent.parent.absolute()
# Folder for static files (mostly css/js)
FRONTEND_FOLDER = pathlib.Path.joinpath(current_folder, "frontend")
STATIC_FOLDER = pathlib.Path.joinpath(FRONTEND_FOLDER, "static")

logging.basicConfig(level=logging.INFO)


class VersionChooser:
    def __init__(self, client: aiodocker.Docker):
        self.client = client

    @staticmethod
    def index() -> web.FileResponse:
        """Serve index.html"""
        return web.FileResponse(str(FRONTEND_FOLDER) + "/index.html", headers={"cache-control": "no-cache"})

    @staticmethod
    def get_current_image_and_tag() -> Optional[Tuple[str, str]]:
        with open(DOCKER_CONFIG_PATH, encoding="utf-8") as startup_file:
            try:
                core = json.load(startup_file)["core"]
                tag = core["tag"]
                image = core["image"]
                return image, tag
            except KeyError as error:
                logging.warning(f"Invalid version file: {error}")
            except Exception as e:
                logging.warning(f"Unable to load settings file: {e}")
        return None

    async def get_version(self) -> web.Response:
        """Fetches current version from config file

        Returns:
            web.Response: json with image name, tag, last modification date,
            sha and architecture of the image
        """
        version = self.get_current_image_and_tag()
        if version is None:
            return web.Response(status=500, text="Unable to load current version from settings. Check the log")
        image_name, tag = version
        full_name = f"{image_name}:{tag}"
        image = await self.client.images.get(full_name)
        output = {
            "repository": image_name,
            "tag": tag,
            "last_modified": image["Created"],
            "sha": image["Id"],
            "architecture": image["Architecture"],
        }
        return web.json_response(output)

    async def load(self, data: bytes) -> web.Response:
        """Load a docker image file.

        Args:
            data (bytes): Tar file from `docker save` output

        Returns:
            web.Response:
                200 - OK
                400 - Error while processing data
                500 - Internal server error while processing docker import image
                501 - Failed to handle docker result
        """
        response = {}
        try:
            # import_image only returns a single line
            response_list = await self.client.images.import_image(data)
            response = response_list[0]
        except Exception as error:
            logging.critical("Error: %s: %s", type(error), error)
            return web.Response(status=500, text=f"Error: {type(error)}: {error}")

        if "errorDetail" in response:
            return web.Response(status=500, text=response["errorDetail"]["message"])
        if "stream" in response:
            return web.json_response(response)

        return web.Response(status=501, text=f"Response: {response}")

    @staticmethod
    def is_valid_version(_repository: str, _tag: str) -> bool:
        # TODO implement basic validation
        return True

    async def pull_version(self, request: web.Request, repository: str, tag: str) -> web.StreamResponse:
        """Applies a new version.

        Pulls the image from dockerhub, streaming the output as a StreamResponse

        Args:
            request (web.Request): http request from aiohttp
            repository (str): name of the image, such as bluerobotics/companion-core
            tag (str): image tag

        Returns:
            web.StreamResponse: Streams the 'docker pull' output
        """
        response = web.StreamResponse()
        response.headers["Content-Type"] = "application/x-www-form-urlencoded"
        # This step actually starts the chunked response
        await response.prepare(request)

        # Stream every line of the output back to the client
        async for line in self.client.images.pull(f"{repository}:{tag}", repo=repository, tag=tag, stream=True):
            await response.write(json.dumps(line).encode("utf-8"))
        await response.write_eof()
        # TODO: restore pruning
        return response

    async def set_version(self, image: str, tag: str) -> web.StreamResponse:
        """Sets the current version.

        Sets the version in startup.json()

        Args:
            image (str): the repository of the image
            tag (str): the desired tag

        Returns:
            web.Response:
                200 - OK
                400 - Invalid image/tag
                500 - Invalid settings file/Other internal error
        """
        if not self.is_valid_version(image, tag):
            return web.Response(status=400, text="Invalid version")

        with open(DOCKER_CONFIG_PATH, "r+", encoding="utf-8") as startup_file:
            try:
                data = json.load(startup_file)
                data["core"]["image"] = image
                data["core"]["tag"] = tag

                # overwrite file contents
                startup_file.seek(0)
                startup_file.write(json.dumps(data, indent=2))
                startup_file.truncate()

                logging.info("Stopping core...")
                core = await self.client.containers.get("companion-core")  # type: ignore
                if core:
                    await core.kill()
                core = await self.client.containers.get("blueos-core")  # type: ignore
                if core:
                    await core.kill()
                return web.Response(status=200, text=f"Changed to version {image}:{tag}, restarting...")

            except KeyError:
                return web.Response(status=500, text="Invalid version file")

            except Exception as error:
                logging.critical("Error: %s: %s", type(error), error)
                return web.Response(status=500, text=f"Error: {type(error)}: {error}")

    async def delete_version(self, image: str, tag: str) -> web.StreamResponse:
        """Deletes the selected version.

        Args:
            image (str): the repository of the image
            tag (str): the desired tag

        Returns:
            web.Response:
                200 - OK
                400 - Invalid image/tag
                403 - image cannot be deleted
                500 - Internal error (unable to read config file/docker refused to delete image)
        """
        full_name = f"{image}:{tag}"
        # refuse if it is the current image
        if (image, tag) == self.get_current_image_and_tag():
            return web.Response(status=500, text=f"Image {full_name} is in use and cannot be deleted.")
        # check if image exists
        try:
            await self.client.images.get(full_name)
        except Exception as error:
            logging.warning(f"Image not found: {full_name} ({error})")
            return web.Response(status=404, text=f"image '{full_name}' not found ({error})")

        # actually attempt to delete it
        logging.info(f"Deleting image {image}:{tag}...")
        try:
            await self.client.images.delete(full_name, force=False, noprune=False)
            logging.info("Image deleted successfully")
            return web.Response(status=200)
        except Exception as e:
            logging.warning(f"Error deleting image: {e}")
            return web.Response(status=500, text=f"Unable do delete image: {e}")

    async def get_available_versions(self, repository: str) -> web.Response:
        """Returns versions available locally and in the remote

        Args:
            repository (str): repository name (such as bluerobotics/companion-core)
            tag (str): tag (such as "master" or "latest")

        Returns:
            web.Response: json described in the openapi file
        """
        output: Dict[str, Optional[Union[str, List[TagMetadata]]]] = {"local": [], "remote": [], "error": None}
        for image in await self.client.images.list():
            if not image["RepoTags"]:
                continue
            if not any("/companion-core:" in tag for tag in image["RepoTags"]):
                continue
            for image_tag in image["RepoTags"]:
                image_repository, tag = image_tag.split(":")
                assert isinstance(output["local"], list)
                output["local"].append(
                    {
                        "repository": image_repository,
                        "tag": tag,
                        "last_modified": datetime.fromtimestamp(image["Created"]).strftime("%Y-%m-%dT%H:%M:%S.%fZ"),
                        "sha": image["Id"],
                    }
                )
        try:
            assert isinstance(output["local"], list)
            output["error"], online_tags = await TagFetcher().fetch_remote_tags(
                repository, [image["tag"] for image in output["local"]]
            )
        except Exception as error:
            logging.critical(f"error fetching online tags: {error}")
            online_tags = []
            output["error"] = f"error fetching online tags: {error}"
        assert isinstance(output["remote"], list)
        output["remote"].extend([asdict(tag) for tag in online_tags])

        return web.json_response(output)

    async def restart(self) -> web.Response:
        """Returns versions available locally and in the remote
        Returns:
            web.Response: always 200
        """
        logging.info("Stopping core...")
        core = await self.client.containers.get("companion-core")  # type: ignore
        await core.kill()
        return web.Response(status=200, text="Restarting...")
