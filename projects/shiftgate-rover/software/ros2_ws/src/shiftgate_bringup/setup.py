from setuptools import setup
setup(
    name="shiftgate_bringup",
    version="1.0.0",
    packages=["shiftgate_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/shiftgate_bringup"]),
        ("share/shiftgate_bringup", ["package.xml"]),
        ("share/shiftgate_bringup/launch", ["launch/mission.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jitendra Phand",
    license="MIT",
)
