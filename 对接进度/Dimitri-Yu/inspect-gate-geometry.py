#!/usr/bin/env python3
"""Measure the pinned Crazyrun mesh openings; never emit a flight course.

This diagnostic deliberately supports only the existing unscaled, unposed SDF
and upright rectangular gates. It rejects unsupported placements rather than
silently returning coordinates in the wrong frame.
"""
import argparse
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def line_hits(origin, direction, triangles):
    """Signed intersections of an infinite line and triangles (Moller-Trumbore)."""
    hits = []
    for a, b, c in triangles:
        e1, e2 = sub(b, a), sub(c, a)
        h = cross(direction, e2)
        determinant = dot(e1, h)
        if abs(determinant) < 1e-10:
            continue
        s = sub(origin, a)
        u = dot(s, h)/determinant
        q = cross(s, e1)
        v = dot(direction, q)/determinant
        if u >= -1e-7 and v >= -1e-7 and u+v <= 1+1e-7:
            hits.append(dot(e2, q)/determinant)
    unique = []
    for t in sorted(hits):
        if not unique or t-unique[-1] > 1e-5:
            unique.append(t)
    return unique


def central_gap(hits):
    """Require an empty central interval bounded by two solid frame sections."""
    if len(hits) != 4 or not hits[1] < 0 < hits[2]:
        raise ValueError(f'Expected four frame intersections around an opening: {hits}')
    return hits[1], hits[2]


def read_obj(path):
    vertices, groups = [], {}
    name = 'default'
    for line in path.read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == 'o':
            name = p[1]
        elif p[0] == 'v':
            vertices.append(tuple(map(float, p[1:4])))
        elif p[0] == 'f':
            ids = [int(t.split('/')[0]) for t in p[1:]]
            ids = [i-1 if i > 0 else len(vertices)+i for i in ids]
            triangles = groups.setdefault(name, [])
            for i in range(1, len(ids)-1):
                triangles.append(tuple(vertices[j] for j in (ids[0], ids[i], ids[i+1])))
    return groups


def read_stl(path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84+50*count:
        raise ValueError('Expected binary STL with a consistent triangle count')
    return [tuple(tuple(struct.unpack_from('<3f', raw, 84+50*i+12+12*j))
                  for j in range(3)) for i in range(count)]


def check_identity_sdf(path):
    root = ET.parse(path).getroot()
    # Light poses do not transform the included obstacle model.
    scopes = root.findall('./world/include') if root.find('world') is not None else [root]
    for scope in scopes:
        for pose in scope.iter('pose'):
            if pose.attrib or any(abs(float(v)) > 1e-9 for v in pose.text.split()):
                raise ValueError(f'Nonidentity or frame-relative pose in {path.name}')
        for scale in scope.iter('scale'):
            if any(abs(float(v)-1) > 1e-9 for v in scale.text.split()):
                raise ValueError(f'Nonidentity scale in {path.name}')


def measure(name, triangles, collision):
    vertices = set(v for tri in triangles for v in tri)
    zmax = max(v[2] for v in vertices)
    # The current model has a rectangular 0.6 m outer frame. Use its upper
    # section, excluding the asymmetric feet, to estimate the horizontal axis.
    top = [v for v in vertices if v[2] > zmax-.61]
    mx, my = (sum(v[d] for v in top)/len(top) for d in (0, 1))
    xx = sum((v[0]-mx)**2 for v in top)
    yy = sum((v[1]-my)**2 for v in top)
    xy = sum((v[0]-mx)*(v[1]-my) for v in top)
    angle = .5*math.atan2(2*xy, xx-yy)
    axis = (math.cos(angle), math.sin(angle), 0.)
    normal = (-axis[1], axis[0], 0.)
    # Extrema, rather than vertex averages, locate the frame centre even when
    # the exported mesh has asymmetric vertex counts around the feet.
    u = [dot(v, axis) for v in top]
    n = [dot(v, normal) for v in top]
    uc, nc = (min(u)+max(u))/2, (min(n)+max(n))/2
    probe = (uc*axis[0]+nc*normal[0], uc*axis[1]+nc*normal[1], zmax-.3)
    horizontal = line_hits(probe, axis, triangles)
    vertical = line_hits(probe, (0., 0., 1.), triangles)
    left, right = central_gap(horizontal)
    bottom, upper = central_gap(vertical)
    width, height = right-left, upper-bottom
    if not (0 < width < max(u)-min(u) and height > 0):
        raise ValueError(f'Unsupported frame geometry for {name}: {width}, {height}')
    # Check five rays through both the visual OBJ and independent collision STL.
    # This checks rectangular clear spans, not just one centreline or a bbox.
    max_delta, collision_issues, checked = 0., [], 0
    for ratio in (-.4, -.2, 0., .2, .4):
        for direction, shift in ((axis, (0., 0., ratio*height)),
                                 ((0., 0., 1.), tuple(ratio*width*x for x in axis))):
            origin = tuple(a+b for a, b in zip(probe, shift))
            a = line_hits(origin, direction, triangles)
            # Ignore other gates on the same infinite line; only this local box.
            b = [t for t in line_hits(origin, direction, collision) if -.32 < t < .32]
            lo, hi = central_gap(a)
            expected = width if direction == axis else height
            if abs(hi-lo-expected) > 1e-4:
                raise ValueError(f'Visual opening is not rectangular: {name}')
            try:
                central_gap(b)
            except ValueError:
                collision_issues.append({'offset_ratio': ratio, 'axis': 'horizontal' if direction == axis else 'vertical', 'collision_hits_m': b})
                continue
            delta = max(abs(x-y) for x, y in zip(a, b))
            checked += 1
            max_delta = max(max_delta, delta)
            if delta > 1e-4:
                collision_issues.append({'coordinate_delta_m': delta})
    center = [probe[d]+axis[d]*(left+right)/2 for d in range(3)]
    center[2] += (bottom+upper)/2
    yaw = math.atan2(normal[1], normal[0]) % math.pi
    return {'mesh_object': name, 'world_center_m': [round(v, 6) for v in center],
            'opening_width_m': round(width, 6), 'opening_height_m': round(height, 6),
            'normal_axis_yaw_rad_mod_pi': round(yaw, 6),
            'normal_axis_yaw_deg_mod_180': round(math.degrees(yaw), 3),
            'visual_collision_agree': not collision_issues,
            'visual_collision_max_intersection_delta_m': max_delta if checked else None,
            'cross_section_pairs_checked': checked,
            'collision_issues': collision_issues}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resources', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base = args.resources
    world = base/'worlds/crazyflie_map.world'
    model = base/'models/obstacles_crazyrun/model.sdf'
    check_identity_sdf(world)
    check_identity_sdf(model)
    mesh = model.parent/'meshes'
    obj, stl = mesh/'obstacles_crazyrun.obj', mesh/'obstacles_crazyrun.stl'
    groups = read_obj(obj)
    if len(groups) != 7:
        raise ValueError(f'Expected seven objects in this pinned mesh, got {len(groups)}')
    collision = read_stl(stl)
    gates = [measure(name, triangles, collision) for name, triangles in groups.items()]
    result = {'date': '2026-10-30', 'coordinate_frame': 'Gazebo world',
              'flight_ready': False, 'gate_order': None, 'crossing_directions': None,
              'method': 'OBJ triangle line intersections at five offsets on each axis, independently checked against STL; identity SDF transforms verified',
              'limitations': ['Specific to the pinned upright mesh', 'No flight or vision validation',
                             'Unsigned normal axes do not establish traversal direction',
                             'World coordinates require conversion to calibrated estimator coordinates'],
              'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (world, model, obj, stl)}, 'gates': gates}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    for g in gates:
        print(g['mesh_object'], g['world_center_m'], g['opening_width_m'],
              g['opening_height_m'], g['normal_axis_yaw_deg_mod_180'])


if __name__ == '__main__':
    main()
