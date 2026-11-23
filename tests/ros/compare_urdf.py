"""Compare two URDF files by parsed values. Usage: compare_urdf.py a.urdf b.urdf"""

import sys
import xml.etree.ElementTree as ET

TOLERANCE = 1e-7


def nums(text):
    return [float(v) for v in text.split()]


def origin(node):
    item = node.find("origin")
    if item is None:
        return [0.0] * 6
    return nums(item.get("xyz", "0 0 0")) + nums(item.get("rpy", "0 0 0"))


def view(path):
    root = ET.parse(path).getroot()
    out = {}
    for link in root.findall("link"):
        name = f"link {link.get('name')}"
        out[name] = []
        inertial = link.find("inertial")
        if inertial is not None:
            tensor = inertial.find("inertia")
            out[f"{name} inertial"] = origin(inertial) + [float(inertial.find("mass").get("value"))] + [
                float(tensor.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
            ]
        for kind in ("visual", "collision"):
            for item in link.findall(kind):
                shape = item.find("geometry")[0]
                values = [] if shape.tag == "mesh" else [v for k in sorted(shape.attrib) for v in nums(shape.get(k))]
                out[f"{name} {kind} {item.get('name')} {shape.tag}"] = origin(item) + values
    for joint in root.findall("joint"):
        name = f"joint {joint.get('name')} {joint.get('type')} {joint.find('parent').get('link')}>{joint.find('child').get('link')}"
        values = origin(joint)
        if joint.find("axis") is not None:
            values += nums(joint.find("axis").get("xyz"))
        limit = joint.find("limit")
        if limit is not None:
            values += [float(limit.get(k)) for k in sorted(limit.attrib)]
        out[name] = values
    return out


def main():
    a, b = view(sys.argv[1]), view(sys.argv[2])
    problems = [f"only in one file: {k}" for k in sorted(set(a) ^ set(b))]
    for key in sorted(set(a) & set(b)):
        if len(a[key]) != len(b[key]) or any(abs(x - y) > TOLERANCE for x, y in zip(a[key], b[key])):
            problems.append(f"differs: {key}: {a[key]} != {b[key]}")
    for problem in problems[:20]:
        print(problem)
    print(f"compared {len(a)} elements, {len(problems)} differences")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
