from glob import glob
from setuptools import find_packages, setup


package_name = "mos_cart_fullness"

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
    # Keep GPU framework installation under Lumos/Jetson environment control.
    install_requires=["setuptools"],
    extras_require={"inference": ["ultralytics>=8.0"]},
    zip_safe=True,
    maintainer="JMT",
    maintainer_email="user@example.com",
    description="MOS wrist-camera 2D cart fullness decision node.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mos_cart_fullness = mos_cart_fullness.cart_fullness_node:main",
        ],
    },
)
