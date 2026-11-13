from setuptools import setup, find_packages

package = "shiftgate_nav"

setup(
    name=package,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package]),
        ("share/" + package, ["package.xml"]),
        ("share/" + package + "/launch", ["launch/mission.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jitendra Phand",
    maintainer_email="jitendra@example.com",
    description="ShiftGate mission node",
    license="MIT",
    entry_points={"console_scripts": ["mission = shiftgate_nav.mission_node:main"]},
)
