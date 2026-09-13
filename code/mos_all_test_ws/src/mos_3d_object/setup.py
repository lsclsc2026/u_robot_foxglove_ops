from glob import glob
from setuptools import find_packages, setup


package_name = "mos_3d_object"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/weights", glob("weights/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JMT",
    maintainer_email="user@example.com",
    description="MOS RGB-D YOLO 3D object coordinate detection node.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mos_3d_object_node = mos_3d_object.vision_node:main",
        ],
    },
)
