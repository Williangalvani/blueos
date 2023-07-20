import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Generator, List
from unittest.mock import MagicMock, patch
import time
import pytest
from docker.errors import NotFound
from pyfakefs.fake_filesystem_unittest import TestCase

from bootstrap.bootstrap import Bootstrapper

SAMPLE_JSON = """{
    "core": {
        "tag": "master",
        "image": "bluerobotics/blueos-core",
        "enabled": true,
        "webui": false,
        "network": "host",
        "binds": {
            "/dev/": {
                "bind": "/dev/",
                "mode": "rw"
            },
            "/var/run/wpa_supplicant/wlan0": {
                "bind": "/var/run/wpa_supplicant/wlan0",
                "mode": "rw"
            },
            "/tmp/wpa_playground": {
                "bind": "/tmp/wpa_playground",
                "mode": "rw"
            }
        },
        "privileged": true
    },
    "ttyd": {
        "tag": "master",
        "image": "bluerobotics/companion-ttyd",
        "enabled": true,
        "webui": false,
        "network": "host",
        "binds": {},
        "privileged": true
    }
}"""


class FakeContainer:
    """Mocks a single Container from Docker-py"""

    def __init__(self, name: str, raise_if_stopped: bool = False) -> None:
        self.name: str = name
        self.client: Any
        self.raise_if_stopped = raise_if_stopped
        self.created_time = time.time()
        print(f"created container {self.name} at {self.created_time}")

    def age(self) -> float:
        return time.time() - self.created_time

    def set_client(self, client: Any) -> None:
        self.client = client

    def remove(self) -> None:
        if self.raise_if_stopped:
            raise RuntimeError("Container cannot be stopped")
        print(f"removing container {self.name}")
        self.client.containers.remove(self.name)

    def stop(self) -> None:
        pass

    def __repr__(self) -> str:
        return self.name


class FakeContainers:
    """Mocks "Containers" class from docker-py"""

    def __init__(self, containers: List[FakeContainer]):
        self.containers: Dict[str, FakeContainer] = {container.name: container for container in containers}

    def get(self, container: str) -> FakeContainer:
        result = self.containers.get(container, None)
        if result is None:
            raise NotFound("Container not found")
        return result

    def remove(self, container: str) -> None:
        del self.containers[container]

    # suppress warnings as we need to keep the same signature
    # pylint: disable=unused-argument
    def run(self, image: str, name: str = "", **kargs: Dict[str, Any]) -> None:
        self.containers[name] = FakeContainer(name)

    def list(self) -> List[FakeContainer]:
        return list(self.containers.values())


@dataclass
class FakeImage:
    image: str
    tags: List[str]


class FakeImages:
    @staticmethod
    def pull(_image: str) -> None:
        return

    @staticmethod
    def list(image_name: str) -> List[FakeImage]:
        return [FakeImage(image=image_name, tags=["master"])]


class FakeClient:
    """Mocks a docker-py client for testing purposes"""

    def __init__(self) -> None:
        self.containers = FakeContainers([])
        self.images = FakeImages()

    def set_active_dockers(self, containers: List[FakeContainer]) -> None:
        for container in containers:
            container.set_client(self)
        self.containers = FakeContainers(containers)


class FakeLowLevelAPI:

    progress_text: List[str] = [
        "Pretending we are pulling image {0}",
        "Almost there..",
        "Done!",
    ]

    def pull(self, image: str, stream: bool = True, decode: bool = True) -> Generator[Dict[str, Any], None, None]:
        if not stream or not decode:
            raise NotImplementedError
        for line in self.progress_text:
            yield {"status": line.format(image), "progressDetail": {}, "id": "e72ac664f4f0"}


# The "type: ignore" comment in the next line addresses the fact that TestCase is Any,
# which mypy disallows subclassing
class BootstrapperTests(TestCase):  # type: ignore
    def setUp(self) -> None:
        self.setUpPyfakefs()
        self.setupRequestsMock()

    def setupRequestsMock(self) -> None:
        # Create a mock response object
        self.mock_response = MagicMock()
        # Patch the requests.get function to return the mock response
        self.requests_mock = patch("requests.get", return_value=self.mock_response)
        self.mock_get = self.requests_mock.start()

    @pytest.mark.timeout(10)
    def test_start_core(self) -> None:
        self.fs.create_file(Bootstrapper.DOCKER_CONFIG_FILE_PATH, contents=SAMPLE_JSON)
        fake_client = FakeClient()
        bootstrapper = Bootstrapper(fake_client, FakeLowLevelAPI())
        bootstrapper.start("core")

    @pytest.mark.timeout(10)
    def test_start_core_and_ttyd(self) -> None:
        self.fs.create_file(Bootstrapper.DOCKER_CONFIG_FILE_PATH, contents=SAMPLE_JSON)
        fake_client = FakeClient()
        bootstrapper = Bootstrapper(fake_client, FakeLowLevelAPI())
        bootstrapper.start("core")
        self.mock_response.json.return_value = {"repository": ["core"]}
        # we don't check if ttyd is running with requests, so no mock is required
        bootstrapper.start("ttyd")
        assert bootstrapper.is_running("core")
        assert bootstrapper.is_running("ttyd")

    @pytest.mark.timeout(10)
    def test_start_core_no_config(self) -> None:
        self.fs.create_file(Bootstrapper.DEFAULT_FILE_PATH, contents=SAMPLE_JSON)
        self.fs.create_dir(Path(Bootstrapper.DOCKER_CONFIG_PATH))
        fake_client = FakeClient()
        bootstrapper = Bootstrapper(fake_client, FakeLowLevelAPI())
        bootstrapper.start("core")
        self.mock_response.json.return_value = {"repository": ["core"]}
        assert bootstrapper.is_running("core")

    @pytest.mark.timeout(10)
    def test_is_running(self) -> None:
        fake_client = FakeClient()
        bootstrapper = Bootstrapper(fake_client, FakeLowLevelAPI())

        fake_client.set_active_dockers([])
        assert bootstrapper.is_running("core") is False

        fake_core = FakeContainer(Bootstrapper.CORE_CONTAINER_NAME)
        fake_client.set_active_dockers([fake_core])
        assert bootstrapper.is_running("core") is True

    @pytest.mark.timeout(10)
    def test_is_version_chooser_online(self) -> None:
        fake_client = FakeClient()
        bootstrapper = Bootstrapper(fake_client, FakeLowLevelAPI())

        self.mock_response.json.return_value = {"repository": []}
        assert bootstrapper.is_version_chooser_online() is False

        self.mock_response.json.return_value = {"repository": ["core"]}
        assert bootstrapper.is_version_chooser_online() is True

    @pytest.mark.timeout(10)
    def test_remove_core(self) -> None:
        fake_client = FakeClient()
        bootstrapper = Bootstrapper(fake_client, FakeLowLevelAPI())
        fake_core = FakeContainer(Bootstrapper.CORE_CONTAINER_NAME)
        fake_client.set_active_dockers([fake_core])
        self.mock_response.json.return_value = {"repository": ["core"]}
        assert bootstrapper.is_running("core") is True
        bootstrapper.remove("core")
        assert bootstrapper.is_running("core") is False

    @pytest.mark.timeout(10)
    def test_bootstrap_start(self) -> None:
        self.fs.create_file(Bootstrapper.DOCKER_CONFIG_FILE_PATH, contents=SAMPLE_JSON)
        bootstrapper = Bootstrapper(FakeClient(), FakeLowLevelAPI())
        assert bootstrapper.is_running("core") is False
        bootstrapper.run()
        self.mock_response.json.return_value = {"repository": ["core"]}
        assert bootstrapper.is_running("core") is True

    @pytest.mark.timeout(10)
    def test_bootstrap_start_bad_json(self) -> None:
        self.fs.create_file(Bootstrapper.DEFAULT_FILE_PATH, contents=SAMPLE_JSON)
        self.fs.create_file(Bootstrapper.DOCKER_CONFIG_FILE_PATH, contents=json.dumps({"potato": "bread"}))
        bootstrapper = Bootstrapper(FakeClient(), FakeLowLevelAPI())
        bootstrapper.run()
        self.mock_response.json.return_value = {"repository": ["core"]}
        assert bootstrapper.is_running("core")

    @pytest.mark.timeout(10)
    def test_bootstrap_start_invalid_json(self) -> None:
        self.fs.create_file(Bootstrapper.DEFAULT_FILE_PATH, contents=SAMPLE_JSON)
        self.fs.create_file(Bootstrapper.DOCKER_CONFIG_FILE_PATH, contents="biscoito")
        bootstrapper = Bootstrapper(FakeClient(), FakeLowLevelAPI())
        bootstrapper.run()
        self.mock_response.json.return_value = {"repository": ["core"]}
        assert bootstrapper.is_running("core")
