from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_uses_pure_ament_python():
    package_xml = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

    build_tools = {
        element.text.strip()
        for element in package_xml.findall("buildtool_depend")
        if element.text
    }
    build_type = package_xml.findtext("./export/build_type", default="").strip()

    assert build_tools == {"ament_python"}
    assert build_type == "ament_python"
    assert not (PACKAGE_ROOT / "CMakeLists.txt").exists()
    assert not (PACKAGE_ROOT / "scripts" / "mos_cart_fullness").exists()


def test_package_declares_runtime_dependencies():
    package_xml = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependency_tags = ("depend", "exec_depend")
    dependencies = {
        element.text.strip()
        for tag in dependency_tags
        for element in package_xml.findall(tag)
        if element.text
    }

    assert {
        "rclpy",
        "std_msgs",
        "std_srvs",
        "ament_index_python",
        "launch",
        "launch_ros",
        "python3-opencv",
    } <= dependencies


def test_setup_installs_ros_resources_and_console_entry_point():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "share/ament_index/resource_index/packages" in setup_text
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("launch/*.py")' in setup_text
    assert 'glob("weights/*.pt")' in setup_text
    assert (
        "mos_cart_fullness = mos_cart_fullness.cart_fullness_node:main"
        in setup_text
    )


def test_setup_does_not_replace_jetson_gpu_dependencies_by_default():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert 'install_requires=["setuptools"]' in setup_text
    assert 'extras_require={"inference": ["ultralytics>=8.0"]}' in setup_text


def test_setup_cfg_installs_executable_in_ros_package_lib_directory():
    setup_cfg = (PACKAGE_ROOT / "setup.cfg").read_text(encoding="utf-8")

    assert "script_dir=$base/lib/mos_cart_fullness" in setup_cfg
    assert "install_scripts=$base/lib/mos_cart_fullness" in setup_cfg
