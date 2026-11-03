"""Lightweight checks that the package root exports the documented API.

These do not import ros2_lingua (that needs rclpy); they only assert the
``__init__.py`` surface the README documents.
"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path


EXPECTED_EXPORTS = {
    "LinguaMixin",
    "LinguaActionServer",
    "DispatchConfig",
    "ActionMapping",
    "ServiceMapping",
}


class TestPackageInit(unittest.TestCase):
    def test_init_exports_documented_names(self) -> None:
        init_path = Path(__file__).resolve().parents[1] / "ros2_lingua" / "__init__.py"
        self.assertTrue(init_path.is_file(), f"missing {init_path}")

        tree = ast.parse(init_path.read_text(encoding="utf-8"), filename=str(init_path))
        all_names: set[str] | None = None
        imported: set[str] = set()

        for node in tree.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == "__all__":
                        if isinstance(node.value, (ast.List, ast.Tuple)):
                            all_names = {
                                elt.value
                                for elt in node.value.elts
                                if isinstance(elt, ast.Constant) and isinstance(elt.value, str)
                            }
            elif isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    imported.add(alias.asname or alias.name)

        self.assertIsNotNone(all_names, "__init__.py must define __all__")
        assert all_names is not None
        self.assertEqual(all_names, EXPECTED_EXPORTS)
        self.assertTrue(EXPECTED_EXPORTS.issubset(imported), imported)


if __name__ == "__main__":
    unittest.main()
