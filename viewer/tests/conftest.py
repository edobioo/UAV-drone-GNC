from pathlib import Path
import sys
import importlib.util
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture
def demo_dir(tmp_path):
    script = Path(__file__).resolve().parents[1] / 'examples/generate_demo.py'
    spec = importlib.util.spec_from_file_location('demo_fixture', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.generate(tmp_path)
    return tmp_path
