from setuptools import find_packages, setup

package_name = "mos_ops_control"

setup(
    name=package_name,
    version="0.4.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Foxglove toggle controls for guarded Lumos teleoperation.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["ops_control = mos_ops_control.ops_control:main"]},
)
