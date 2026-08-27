import os

from glob import glob
from setuptools import find_packages, setup


package_name = "room_context_manager"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/room_context_manager"],
        ),
        (
            "share/room_context_manager",
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                "room_context_manager",
                "config",
            ),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="AprilTag-based room context manager for TIAGo",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "room_context_manager = "
            "room_context_manager.room_context_manager:main",
        ],
    },
)
