"""Small test-only expander for the Xacro subset this project generates.

This is an internal consistency check. It is NOT the real xacro tool, and passing
here does not satisfy the external Xacro acceptance gate.
"""

import ast
import math
import operator
import re
import xml.etree.ElementTree as ET
from pathlib import Path

NS = "{http://www.ros.org/wiki/xacro}"
_OPS = {
    ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
    ast.Div: operator.truediv, ast.Pow: operator.pow, ast.USub: operator.neg, ast.UAdd: operator.pos,
}


def evaluate(text, scope):
    def walk(node):
        if isinstance(node, ast.Expression):
            return walk(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return node.value
        if isinstance(node, ast.Name):
            if node.id == "pi":
                return math.pi
            value = scope[node.id]
            try:
                return float(value)
            except (TypeError, ValueError):
                return value
        if isinstance(node, ast.BinOp):
            return _OPS[type(node.op)](walk(node.left), walk(node.right))
        if isinstance(node, ast.UnaryOp):
            return _OPS[type(node.op)](walk(node.operand))
        raise ValueError(f"Unsupported expression: {text}")

    return walk(ast.parse(text, mode="eval"))


def substitute(text, scope):
    return re.sub(r"\$\{([^}]*)\}", lambda m: repr(evaluate(m.group(1), scope)) if not isinstance(evaluate(m.group(1), scope), str) else evaluate(m.group(1), scope), text)


def expand_file(path, overrides=None):
    path = Path(path)
    macros = {}
    output = ET.Element("robot", {"name": ET.parse(path).getroot().get("name", "")})

    def load(file_path, scope):
        for child in ET.parse(file_path).getroot():
            handle(child, scope, output, file_path.parent)

    def handle(node, scope, parent, folder):
        if node.tag == f"{NS}include":
            load(folder / node.get("filename"), scope)
        elif node.tag == f"{NS}property":
            scope[node.get("name")] = substitute(node.get("value"), scope)
        elif node.tag == f"{NS}macro":
            params = dict(item.split(":=", 1) for item in node.get("params").split())
            macros[node.get("name")] = (params, list(node))
        elif node.tag.startswith(NS):
            params, body = macros[node.tag[len(NS):]]
            local = dict(scope)
            local.update(params)
            local.update({k: substitute(v, scope) for k, v in node.attrib.items()})
            local.update(overrides or {})
            for item in body:
                handle(item, local, parent, folder)
        else:
            copy = ET.SubElement(parent, node.tag, {k: substitute(v, scope) for k, v in node.attrib.items()})
            for item in node:
                handle(item, scope, copy, folder)

    load(path, {})
    return output
