import ast
import xml.etree.ElementTree as ET
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def read_manifest(package: str) -> ET.Element:
    return ET.parse(REPOSITORY_ROOT / package / "package.xml").getroot()


def test_python_packages_have_real_maintainer_metadata() -> None:
    for package in ("ros2_lingua", "ros2_lingua_core", "ros2_lingua_mock"):
        manifest = read_manifest(package)
        maintainer = manifest.find("maintainer")
        assert maintainer is not None
        assert maintainer.text == "purahan"
        assert maintainer.attrib["email"] == "gpurahan@gmail.com"


def test_core_declares_python_build_dependencies() -> None:
    manifest = read_manifest("ros2_lingua_core")
    assert manifest.findtext("buildtool_depend") == "ament_python"
    assert manifest.findtext("exec_depend") == "python3-setuptools"


def test_core_exposes_optional_llm_dependencies() -> None:
    setup_path = REPOSITORY_ROOT / "ros2_lingua_core" / "setup.py"
    module = ast.parse(setup_path.read_text(encoding="utf-8"))
    setup_call = next(
        node
        for node in ast.walk(module)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "setup"
    )
    extras_node = next(
        keyword.value
        for keyword in setup_call.keywords
        if keyword.arg == "extras_require"
    )
    extras = ast.literal_eval(extras_node)

    assert extras["openai"] == ["openai"]
    assert extras["anthropic"] == ["anthropic"]
    assert extras["ollama"] == ["ollama"]
    assert extras["all"] == ["openai", "anthropic", "ollama"]


def test_cpp_uses_system_nlohmann_json() -> None:
    manifest = read_manifest("ros2_lingua_cpp")
    dependencies = {element.text for element in manifest.findall("depend")}
    assert "nlohmann-json-dev" in dependencies

    cmake = (REPOSITORY_ROOT / "ros2_lingua_cpp" / "CMakeLists.txt").read_text()
    assert "find_package(nlohmann_json REQUIRED)" in cmake
    assert "nlohmann_json::nlohmann_json" in cmake

    cpp_root = REPOSITORY_ROOT / "ros2_lingua_cpp"
    assert not (cpp_root / "include" / "ros2_lingua_cpp" / "json.hpp").exists()
    for source in (
        cpp_root / "include" / "ros2_lingua_cpp" / "capability.hpp",
        cpp_root / "include" / "ros2_lingua_cpp" / "lingua_mixin.hpp",
        cpp_root / "test" / "test_capability.cpp",
    ):
        source_text = source.read_text(encoding="utf-8")
        assert "ros2_lingua_cpp/json.hpp" not in source_text
        assert "#include <nlohmann/json.hpp>" in source_text


def test_mock_package_declares_dashboard_runtime_dependency() -> None:
    manifest = read_manifest("ros2_lingua_mock")
    assert "rosbridge_server" in {
        element.text for element in manifest.findall("exec_depend")
    }
