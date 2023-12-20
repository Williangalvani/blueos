#!/usr/bin/env python3

import setuptools

setuptools.setup(
    name="Bridget",
    version="0.1.0",
    description="Manager for 'bridges' links.",
    license="MIT",
    py_modules=[],
    install_requires=[
        "bridges == 0.1.0",
        "commonwealth == 0.1.0",
        "fastapi == 0.63.0",
        "fastapi-versioning == 0.9.1",
        "loguru == 0.5.3",
        "uvicorn == 0.13.4",
    ],
)
