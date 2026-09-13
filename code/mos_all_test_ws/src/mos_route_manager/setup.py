from glob import glob

from setuptools import find_packages, setup


package_name = "mos_route_manager"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JMT",
    maintainer_email="user@example.com",
    description="MOS patrol, fullness, and cart replacement route manager.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mos_route_manager = mos_route_manager.route_manager_node:main",
        ],
    },
)
