bl_info = {
    "name": "Manifold Solidify",
    "author": "Henrik Dick",
    "version": (2, 0),
    "blender": (2, 80, 0),
    "location": "View3D (Edit Mode) > Face > Manifold Solidify",
    "description": "Solidifies a mesh independent of normals. Useful for houses, Mobius strips and other cool stuff",
    "warning": "",
    "wiki_url": "",
    "category": "Mesh",
}

import bpy
import bmesh
from mathutils import *
from math import *
import itertools
from typing import NamedTuple

def manifold_solidify(thickness, even_thickness, high_quality_thickness, fix_intersections, boundary_fix, rim):
    # validate parameters
    assert boundary_fix in ('NONE', 'INNER', 'OUTER'), "Invalid boundary fix '{}'".format(boundary_fix)
    assert rim in ('NONE', 'ONLY_CLEAN', 'ONLY_FULL', 'CLEAN', 'FULL'), "Invalid rim option '{}'".format(boundary_fix)
    
    # data types
    # c type: class
    class EdgeData(NamedTuple):
        verts: list
        edge: bmesh.types.BMEdge
        faces: list
        link_edge_groups: list
        new_verts: dict
        def __hash__(self):
            return id(self)
        def other_group(self, edge_group):
            if self.link_edge_groups[0] == edge_group:
                return self.link_edge_groups[1]
            elif self.link_edge_groups[1] == edge_group:
                return self.link_edge_groups[0]
            else:
                return None
    # c type: class
    class VertData():
        __slots__ = ["_vert", "_normal", "_normals", "_positions", "_tested", "copy_data"]
        def __init__(self, vert:bmesh.types.BMVert, normal:Vector):
            self._vert = vert
            self._normal = normal
            self._normals = []
            self._positions = []
            self._tested = set()
            self.copy_data = None
        def __getattr__(self, item):
            return self.get_data().__getattribute__('_' + item)
        def get_data(self):
            return self.copy_data == None and self or self.copy_data.get_data()
        def merge(self, other):
            data = self.get_data()
            other_data = other.get_data()
            other_data._normals.extend(data._normals)
            other_data._positions.extend(data._positions)
            other_data._tested.update(data._tested)
            self.copy_data = other_data
    # c type: class
    class EdgeGroup(list):
        def __init__(self, iterable=None, closed=False):
            self.topology_groups = set()
            self.closed = closed
            if iterable:
                list.__init__(self, iterable)
            else:
                list.__init__(self)
        def __hash__(self):
            return id(self)
    # c type: class
    class HashableList(list):
        def __hash__(self):
            return id(self)
    # all face data is stored with Face(face, reversed)
    # c type: class
    class FaceData():
        face: bmesh.types.BMFace
        reversed: bool
        # list of EdgeData elements in order of face.loops or the reverse depending on reversed
        link_edges: list
        def __init__(self, face, reversed):
            self.face = face
            self.reversed = reversed
            self.link_edges = [None] * len(self.face.loops)
        def __hash__(self):
            return id(self)
    
    # function to project vector p on to plane n (normalized) going through O
    def project(p, n):
        return p - n * Vector.dot(n, p)
    # expects n and ref_n to be projected like project(n, edge_dir)
    def angle_around_edge(n, ref_n, edge_dir):
        d = Vector.dot(n, ref_n)
        angle_diff = acos(max(-1, min(1, d)))
        if Vector.dot(Vector.cross(n, ref_n), edge_dir) >= 0:
            angle_diff = 2 * pi - angle_diff
        return angle_diff
    
    # displacement from surface
    disp = thickness / 2
    
    # get context
    active = bpy.context.active_object
    mesh = active.data
    # get bmesh from edit mode
    bm = bmesh.from_edit_mesh(mesh)
    
    # start modifing
    # cache the data that will be modified
    original_verts = [v for v in bm.verts if v.select]
    original_edges = [e for e in bm.edges if e.select]
    original_faces = [f for f in bm.faces if f.select]
    
    loops_data = bm.loops.layers.uv.values() + bm.loops.layers.color.values()
    verts_data = bm.verts.layers.deform.values() + bm.verts.layers.paint_mask.values() + bm.verts.layers.bevel_weight.values()
    edges_data = bm.edges.layers.crease.values() + bm.edges.layers.bevel_weight.values() + bm.edges.layers.freestyle.values()
    faces_data = bm.faces.layers.int.values() + bm.faces.layers.face_map.values()# + [bm.faces.layers.freestyle.active]
    
    edge_crease = bm.edges.layers.crease.active
    
    # create face_data
    def create_face_data(original_faces):
        # map from original face to face data tuple
        # when implementing in c use a list with face.index as indices
        face_sides = {}
        for face in original_faces:
            face_sides[face] = (FaceData(face, False), FaceData(face, True))
        return face_sides
    
    # create edge data
    def create_edge_data(original_edges, face_sides):
        # a map that contains the data for each original edge
        # the data consist of a [child edges data (original edge, normal, [faces], contraint{BASE=face1_normal, AND=face2_normal, OR=face2_normal})]
        original_edge_data_map = {}
        for edge in original_edges:
            # edge vertices
            edge_v0 = edge.verts[0]
            edge_v1 = edge.verts[1]
            # get edge direction vector and center
            edge_vec = edge_v1.co - edge_v0.co
            edge_dir = edge_vec.normalized()
            assert edge_dir.length > 0, "No doubles are allowed, please merge by distance"
            # get adjacent faces
            adj_faces = [f for f in edge.link_faces if f.select]
            if len(adj_faces) > 0:
                # sort adj_faces by the rotation around the edge
                # map saving the reverse flag
                face_reverse = {}
                normal = None
                if len(adj_faces) > 1:
                    face_angle = {}
                    # variable for reference normal to measure the angle to
                    ref_normal = None
                    for face in adj_faces:
                        # get the correct normal
                        reverse = next((-1 for loop in face.loops if loop.edge == edge and loop.vert == edge_v1), 1)
                        normal = face.normal * reverse
                        normal_projected = len(face.verts) > 3 and project(normal, edge_dir) or normal
                        normal_projected.normalize()
                        face_reverse[face] = reverse < 0
                        # determine the angle
                        if not ref_normal:
                            ref_normal = normal_projected
                            face_angle[face] = 0
                        else:
                            face_angle[face] = angle_around_edge(normal_projected, ref_normal, edge_dir)
                    # sort by map value (order in CW rotation order)
                    adj_faces = sorted(adj_faces, key = lambda x : face_angle[x])
                else:
                    normal = adj_faces[0].normal
                    face_reverse[adj_faces[0]] = False
                
                # data storage
                edge_data_list = []
                # make a new edge for every adjacent face
                for i in range(0, len(adj_faces)):
                    face = adj_faces[i]
                    next_face = adj_faces[(i + 1) % len(adj_faces)]
                    # prepare the data and append it to the list
                    for j in range(0, face == next_face and 2 or 1):
                        n = j == 0 and normal or -normal
                        faces = None
                        if face == next_face:
                            faces = [face_sides[face][face_reverse[face] == (j == 0) and 1 or 0]]
                        else:
                            faces = [face_sides[face][face_reverse[face] and 1 or 0], face_sides[next_face][not face_reverse[next_face] and 1 or 0]]
                        edge_data = EdgeData([edge_v0, edge_v1], edge, faces, [None, None], {})
                        edge_data_list.append(edge_data)
                        for f in faces:
                            # TODO check if f.face.edges is in loops order (seems like it but is it ensured by bmesh?)
                            f.link_edges[list(f.face.edges).index(edge)] = edge_data
                # store data for later usage
                original_edge_data_map[edge] = edge_data_list
        return original_edge_data_map
    
    # create groups of edges that form vertices
    def create_groups(original_verts, original_edge_data_map):
        original_vert_groups_map = {}
        for vert in original_verts:
            # get adjacent edges
            adj_edges = [e for e in vert.link_edges if e.select and e in original_edge_data_map]
            if len(adj_edges) <= 1:
                continue
            adj_new_edge_data = [original_edge_data_map[e] for e in adj_edges]
            # setup grouping
            unassigned_edge_data = list(itertools.chain(*adj_new_edge_data))
            edge_groups = [[]]
            # group
            while len(unassigned_edge_data) > 0:
                # find an edge to fit the last open group
                found_edges = []
                # if the group is empty anything can fit in
                if len(edge_groups[-1]) == 0:
                    found_edges.append(next(iter(unassigned_edge_data)))
                else:
                    edge_group_faces = set(itertools.chain(*[e.faces for e in edge_groups[-1]]))
                    for edge in unassigned_edge_data:
                        # check if edges share a face
                        if not edge_group_faces.isdisjoint(edge.faces):
                            found_edges.append(edge)
                if len(found_edges) > 0:
                    # add to last group
                    edge_groups[-1].extend(found_edges)
                    for found_edge in found_edges:
                        unassigned_edge_data.remove(found_edge)
                else:
                    # open new group
                    edge_groups.append([])
            
            # sort group elements to make loops
            sorted_edge_groups = []
            contains_open_groups = False
            contains_long_groups = False
            for g in edge_groups:
                # determine the head (face) of the sort (no empty groups per definition)
                head = None
                head_face = None
                head = next((h for h in g if len(h.faces) > 1), g[0])
                start_face_direction = len(head.faces) > 1 and head.verts[0] == vert and 1 or 0
                head_face = head.faces[start_face_direction]
                sorted_g = [head]
                g = set(g)
                g.remove(head)
                reversed = False
                while head:
                    head = None
                    if head_face:
                        for e in head_face.link_edges:
                            if e in g:
                                head = e
                                if e.faces[0] == head_face:
                                    if len(e.faces) > 1:
                                        head_face = e.faces[1]
                                    else:
                                        head_face = None
                                else:
                                    head_face = e.faces[0]
                                break
                        if head:
                            sorted_g.append(head)
                            g.remove(head)
                    if not head and len(g) > 0:
                        assert not reversed, 'potential infinite iteration and coding bug: sorting group tried to reverse a second time'
                        head = sorted_g[0]
                        head_face = head.faces[1 - start_face_direction]
                        sorted_g.reverse()
                        reversed = True
                # make all groups sorted CW
                if not reversed:
                    sorted_g.reverse()
                # if open set flag
                # since no group is split yet there is no need to check both ends for the open/closed check
                open = len(sorted_g[0].faces) == 1
                if open:
                    contains_open_groups = True
                if not contains_long_groups and len(sorted_g) > 3:
                    contains_long_groups = True
                # add the sorted group
                sorted_edge_groups.append(EdgeGroup(sorted_g, not open))
            # replace original groups
            edge_groups = sorted_edge_groups
            
            if contains_open_groups:
                # sort groups
                sorted_groups = []
                open_groups_count = 0
                reject_streak_start = None
                while len(edge_groups) > 0:
                    g = edge_groups[-1]
                    edge_groups.remove(g)
                    # since no group is split yet there is no need to check both ends for the open/closed check
                    if len(g[0].faces) > 1 or len(g) < 3:
                        # add closed loops immediately because they are already sorted
                        sorted_groups.append(g)
                    else:
                        if open_groups_count == 0:
                            sorted_groups.insert(0, g)
                            open_groups_count += 1
                        else:
                            # find insert location
                            found_insert = None
                            start_edge, end_edge = g[0].edge, g[-1].edge
                            for i in range(0, open_groups_count):
                                existing_g = sorted_groups[i]
                                ex_start_edge, ex_end_edge = existing_g[0].edge, existing_g[-1].edge
                                if ex_end_edge == start_edge:
                                    found_insert = i + 1
                                elif ex_start_edge == end_edge:
                                    found_insert = i
                            # either insert or put back for later insert
                            if found_insert != None:
                                reject_streak_start = None
                                sorted_groups.insert(found_insert, g)
                                open_groups_count += 1
                            else:
                                # reject
                                if not reject_streak_start:
                                    reject_streak_start = g
                                    edge_groups.insert(0, g)
                                elif reject_streak_start == g:
                                    sorted_groups.append(g)
                                    open_groups_count += 1
                                else:
                                    edge_groups.insert(0, g)
                                        
                
                # replace the original groups
                edge_groups = sorted_groups
            
            if contains_long_groups:
                # try to split big loops if neccesary
                split_groups = []
                while len(edge_groups) > 0:
                    g = edge_groups[-1]
                    edge_groups.remove(g)
                    if len(g) < 4:
                        # let it pass untouched
                        split_groups.append(g)
                    else:
                        unique = list(g)
                        has_doubles = False
                        for i in range(0, len(g)):
                            e_i = g[i].edge
                            for j in range(i + 1, len(g)):
                                if e_i == g[j].edge:
                                    unique[i] = None
                                    unique[j] = None
                                    has_doubles = True
                        if not has_doubles:
                            split_groups.append(g)
                            continue
                        current_split_groups_size = len(split_groups)
                        unique_start = None
                        first_unique_end = None
                        last_split = None
                        first_split = None
                        real_i = 0
                        while real_i < len(g) or g.closed and (real_i <= (first_unique_end or 0) + len(g) or first_split != last_split):
                            i = real_i % len(g)
                            if unique[i]:
                                if first_unique_end != None and unique_start == None:
                                    unique_start = real_i
                            elif first_unique_end == None:
                                first_unique_end = i
                            elif unique_start != None:
                                split = ceil((unique_start + real_i) / 2) % len(g)
                                if last_split != None:
                                    if last_split > split:
                                        split_groups.append(EdgeGroup(g[last_split:] + g[:split], False))
                                    else:
                                        split_groups.append(EdgeGroup(g[last_split:split], False))
                                last_split = split
                                if first_split == None:
                                    first_split = split
                                unique_start = None
                            real_i += 1
                        
                        if first_split == None:
                            split_groups.append(g)
                        elif not g.closed:
                            split_groups.insert(current_split_groups_size, EdgeGroup(g[:first_split], False))
                            split_groups.append(EdgeGroup(g[last_split:], False))
                # replace original groups
                edge_groups = split_groups
            
            original_vert_groups_map[vert] = edge_groups
            
            # create links in edges (link_edge_groups)
            for g in edge_groups:
                for e in g:
                    e.link_edge_groups[e.verts.index(vert)] = g
            
        # return all groups for every vertex
        return original_vert_groups_map
    # find regions and topology
    def create_regions(original_vert_groups_map):
        # create basic regions
        regions = []
        unassigned_edge_groups = set(itertools.chain(*original_vert_groups_map.values()))
        regions.append([])
        while len(unassigned_edge_groups) > 0:
            # find an edge to fit the last open group
            found_edges = []
            if len(regions[-1]) == 0:
                found_edges.append(next(iter(unassigned_edge_groups), None))
            else:
                region_edges = set(itertools.chain(*[g for g in regions[-1]]))
                for e_g in set(itertools.chain(*[[e.other_group(g) for e in g if e.other_group(g) in unassigned_edge_groups] for g in regions[-1]])):
                    # if the group is empty anything can fit in
                    # check if edge groups share a edge
                    if not region_edges.isdisjoint(e_g):
                        found_edges.append(e_g)
            if len(found_edges) > 0:
                # add to last group
                regions[-1].extend(found_edges)
                for e in found_edges:
                    unassigned_edge_groups.remove(e)
            else:
                # open new group
                regions.append(found_edges) # recylce found_edges list instance
        # detect region topology
        topology_groups = []
        for edge_groups in regions:
            topology = []
            # create a topology group for inside edges
            inside = HashableList(edge_groups)
            # find open edge_groups and group them
            for e_g in edge_groups:
                # if edge group open
                if not e_g.closed:
                    # find intersections with topology groups
                    open_edges = set([e for e in [e_g[0], e_g[-1]] if len(e.faces) == 1])
                    intersections = [t_g for t_g in topology if any((not open_edges.isdisjoint(t_e_g) for t_e_g in t_g))]
                    topology_group = len(intersections) > 0 and intersections[0] or HashableList()
                    for i in range(1, len(intersections)):
                        intersections[0].extend(intersections[i])
                        topology.remove(intersections[i])
                    # remove from inside
                    inside.remove(e_g)
                    # add to topology
                    topology_group.append(e_g)
                    if len(intersections) == 0:
                        topology.append(topology_group)
            # add inside group to topology
            topology.insert(0, inside)
            # add the new topology group to edge_group.topology_groups
            for topology_group in topology:
                for edge_group in topology_group:
                    edge_group.topology_groups.add(topology_group)
            topology_groups.extend(topology)
        # return region-topology map
        return topology_groups
    # create the vertices (, edges and faces) around the old vertices, that form the new mesh
    def make_crossings(original_vert_groups_map):
        merged_vert_data = []
        new_verts = []
        vert_open_verts_map = {}
        for vert, edge_groups in original_vert_groups_map.items():
            open_verts = []
            vert_open_verts_map[vert] = open_verts
            for g in edge_groups:
                # get all the constraint_planes
                normals = []
                first_edge = None
                for i in range(0, len(g), 2):
                    e = g[i]
                    # append face normals
                    for f in e.faces:
                        if not first_edge or f not in first_edge.faces:
                            normals.append(f.face.normal * (f.reversed and -1 or 1))
                    if not first_edge:
                        first_edge = e
                # find normal
                normal = Vector((0, 0, 0))
                move_normal = None
                if not high_quality_thickness:
                    # not using constraints as of now
                    # find all face normals and add them
                    total_angle = 0
                    first_edge = None
                    for i in range(0, len(g), 2):
                        e = g[i]
                        for f in e.faces:
                            if not first_edge or f not in first_edge.faces:
                                angle = 1
                                if even_thickness:
                                    loop = next((l for l in f.face.loops if l.vert == vert))
                                    e0 = (loop.edge.other_vert(loop.vert).co - loop.vert.co).normalized()
                                    e1 = (loop.link_loop_prev.edge.other_vert(loop.vert).co - loop.vert.co).normalized()
                                    angle = acos(max(-1, min(1, Vector.dot(e0, e1))))
                                normal += f.face.normal * (angle * (f.reversed and -1 or 1))
                                total_angle += angle
                        if not first_edge:
                            first_edge = e
                    normal /= total_angle
                    if even_thickness:
                        d = Vector.dot(normal, normal)
                        if d > 0.001: normal /= d
                    else:
                        normal.normalize()
                    normal *= disp
                    # get free moving direction
                    move_normal = len(g) > 2 and Vector((0, 0, 0)) or None
                    for i in range(1, len(g) - 1):
                        e = g[i].edge.other_vert(vert).co - vert.co
                        move_normal += e.normalized()
                else:
                    normals_query = list(normals)
                    # group them into up to 3 distinct groups
                    normal_groups = []
                    while len(normals_query) > 0:
                        if len(normal_groups) == 0:
                            if len(normals_query) <= 2:
                                normal_groups.extend([v.copy() for v in normals_query])
                                normals_query.clear()
                            else:
                                # find most different two normals
                                min_projection = 2
                                min_normal0 = None
                                min_normal1 = None
                                for i in range(0, len(normals_query)):
                                    n0 = normals_query[i]
                                    for j in range(i + 1, len(normals_query)):
                                        n1 = normals_query[j]
                                        p = Vector.dot(n0, n1)
                                        if p < min_projection:
                                            min_projection = p
                                            min_normal0 = n0
                                            min_normal1 = n1
                                normal_groups.append(min_normal0.copy())
                                normal_groups.append(min_normal1.copy())
                                normals_query.remove(min_normal0)
                                normals_query.remove(min_normal1)
                                min_projection = 1
                                min_normal2 = None
                                for n in normals_query:
                                    max_p = -1
                                    for n_g in normal_groups:
                                        max_p = max(max_p, Vector.dot(n_g.normalized(), n))
                                    if max_p < min_projection:
                                        min_projection = max_p
                                        min_normal2 = n
                                if min_projection < 0.7:
                                    normal_groups.append(min_normal2.copy())
                                    normals_query.remove(min_normal2)
                        else:
                            # find closest two normals
                            closest_projection = -1
                            closest_normal = None
                            closest_group_normal = None
                            for n in normals_query:
                                for n_g in normal_groups:
                                    p = Vector.dot(n_g.normalized(), n)
                                    if p > closest_projection:
                                        closest_projection = p
                                        closest_normal = n
                                        closest_group_normal = n_g
                            # put them in a group or open a new one
                            closest_group_normal += closest_normal
                            normals_query.remove(closest_normal)
                    # normalized normal groups
                    for n_g in normal_groups:
                        n_g.normalize()
                    # using the saved constraints
                    if len(normal_groups) == 1:
                        normal += normal_groups[0]
                        move_normal = None
                    elif len(normal_groups) == 2:
                        normal += (normal_groups[0] + normal_groups[1]) * 0.5
                        d = Vector.dot(normal, normal)
                        if d > 0: normal /= d
                        move_normal = Vector.cross(normal_groups[0], normal_groups[1])
                    elif len(normal_groups) == 3:
                        normal += (normal_groups[0] + normal_groups[1]) * 0.5
                        d = Vector.dot(normal, normal)
                        if d > 0: normal /= d
                        free_n = Vector.cross(normal_groups[0], normal_groups[1])
                        d = Vector.dot(normal_groups[2], free_n)
                        if d != 0:
                            normal -= free_n * Vector.dot(normal_groups[2], normal - normal_groups[2]) / d
                        move_normal = None
                    # scale normal to displacment dimensions
                    normal *= disp
                # fix boundarys
                if move_normal and boundary_fix != 'NONE' and len(g) > 2 and len(g[0].faces) == 1 and len(g[-1].faces) == 1:
                    # get constrain direction
                    constraint_normal = None
                    e0 = g[0].edge.other_vert(vert).co - vert.co
                    e1 = g[-1].edge.other_vert(vert).co - vert.co
                    if boundary_fix == 'OUTER':
                        constraint_normal = Vector.cross(e0, e1)
                    else:
                        f0 = g[0].faces[0].face.normal * (g[0].faces[0].reversed and -1 or 1)
                        n0 = Vector.cross(e0, f0).normalized()
                        f1 = g[-1].faces[0].face.normal * (g[-1].faces[0].reversed and 1 or -1)
                        n1 = Vector.cross(e1, f1).normalized()
                        constraint_normal = n0 + n1
                    # do the projection
                    d = Vector.dot(constraint_normal, move_normal)
                    if d != 0:
                        normal -= move_normal * Vector.dot(constraint_normal, normal) / d
                vert_data = None
                pos = vert.co + normal
                pos.freeze()
                # fix intersections
                if fix_intersections:
                    current_vert_topology_groups = g.topology_groups
                    def test_intersections(vert_p, vert_co, other_verts, intersections=None):
                        vert_data = None
                        if intersections == None:
                            intersections = set()
                        vert_tested = []
                        for other_vert_data, other_edge_group in other_verts:
                            other_positions = other_vert_data.positions
                            # if intersection is detected
                            if any((Vector.dot(v.co - vert_co, p - vert_p) < 0 for p, v in other_positions)):
                                # replace vert_data and add to intersections
                                vert_data = other_vert_data.get_data()
                                intersections.add(vert_data)
                                if vert_data not in merged_vert_data:
                                    merged_vert_data.append(vert_data)
                            else:
                                # if no intersection detected add the vert to the tested list
                                vert_tested.append(other_vert_data)
                        # if multiple intersections are detected, fuse them all together
                        if len(intersections) >= 2:
                            for ol in intersections:
                                if ol != vert_data:
                                    # delete intersections[i].vert
                                    if set([v[1] for v in ol.positions]).isdisjoint([v[1] for v in vert_data.positions]):
                                        new_verts.remove(ol.vert)
                                        bm.verts.remove(ol.vert)
                                        # remove old vert data reference
                                        if ol in merged_vert_data:
                                            merged_vert_data.remove(ol)
                                        # merge it
                                        ol.merge(vert_data)
                        return vert_data, len(intersections), vert_tested
                    other_verts = [e.new_verts[e.edge.other_vert(vert)] for e in g if e.edge.other_vert(vert) in e.new_verts and not current_vert_topology_groups.isdisjoint(e.new_verts[e.edge.other_vert(vert)][1].topology_groups)]
                    vert_data, intersection_count, vert_tested = test_intersections(pos, vert.co, other_verts)
                    # create vert_data with new vert
                    if not vert_data:
                        vert_data = VertData(bm.verts.new(pos), normal)
                        new_verts.append(vert_data.vert)
                    # add vert_data to tested lists
                    for tested_vert_data in vert_tested:
                        vert_data.tested.add(tested_vert_data)
                    if intersection_count >= 1:
                        for tested_vert_data in list(vert_data.tested):
                            tested_vert_data = tested_vert_data.get_data()
                            for other_vert_p, other_vert in list(tested_vert_data.positions):
                                new_other_vert_data, other_intersection_count, other_tested_lists = test_intersections(other_vert_p, other_vert.co, [(vert_data, g)], set([tested_vert_data]))
                                if other_intersection_count > 1:
                                    vert_data = new_other_vert_data
                else:
                    # create vert
                    vert_data = VertData(bm.verts.new(pos), normal)
                    new_verts.append(vert_data.vert)
                # add normals/positions to vert_data
                for n in normals:
                    vert_data.normals.append((pos, n))
                vert_data.positions.append((pos, vert))
                # copy data
                for data_layer in verts_data:
                    vert_data.vert[data_layer] = vert[data_layer]
                # add to open_verts if the group is an open loop
                if not g.closed:
                    g_max_inner_crease = 0
                    if edge_crease:
                        for e in g:
                            if e.edge[edge_crease] > g_max_inner_crease and len([f for f in e.edge.link_faces if f.select]) > 1:
                                g_max_inner_crease = e.edge[edge_crease]
                    open_verts.append((vert_data, g_max_inner_crease))
                # store vert link in edges
                for e in g:
                    e.new_verts[vert] = (vert_data, g)
        # correct merged verts positions
        if fix_intersections:
            for vert_data in merged_vert_data:
                normals = vert_data.normals
                positions = vert_data.positions
                v_co = Vector((0, 0, 0))
                for p, co in positions:
                    v_co += p
                v_co /= len(positions)
                if high_quality_thickness:
                    # TODO find a better algorithm than this iterative approach
                    last_max_r = 10000
                    reset_co = v_co.copy()
                    for i in range(0, 100):
                        max_r = 0
                        for pos, n in normals:
                            r = Vector.dot(n, pos - v_co)
                            if r > 0:
                                max_r = max(max_r, r)
                                v_co += n * r
                        # stop iterating if solution is found
                        if max_r < 0.01:
                            break
                        # stop and reset position if values are exploding
                        if max_r > last_max_r * 0.99:
                            v_co = reset_co
                            break
                        last_max_r = max_r
                vert_data.vert.co = v_co
        return new_verts, vert_open_verts_map
        
    def make_boundary_faces(original_edges, vert_open_verts_map, original_vert_groups_map, face_sides):
        for vert, open_verts_data in vert_open_verts_map.items():
            try:
                open_verts = [v[0].vert for v in open_verts_data]
                max_inner_crease = [v[1] for v in open_verts_data]
                if len(open_verts) > 2:
                    new_face = bm.faces.new(open_verts)
                    if rim == 'FULL' or rim == 'ONLY_FULL':
                        g = original_vert_groups_map[vert][0]
                        old_loop = next((l for l in g[0].faces[0].face.loops if l.vert == vert), None)
                        for new_loop in new_face.loops:
                            for data_layer in loops_data:
                                new_loop[data_layer] = old_loop[data_layer]
                        new_face.material_index = g[0].faces[0].face.material_index
                    if edge_crease:
                        for i in range(0, len(open_verts)):
                            new_edge = new_face.loops[i].edge
                            crease = min(max_inner_crease[i], max_inner_crease[(i + 1) % len(open_verts)])
                            new_edge[edge_crease] = crease
                elif len(open_verts) == 2 and edge_crease:
                    new_edge = bm.edges.new(open_verts)
                    crease = max(max_inner_crease[0], max_inner_crease[1])
                    new_edge[edge_crease] = crease
            except ValueError:
                pass
        for edge in original_edges:
            # if it is a boundary edge
            adj_faces = [f for f in edge.link_faces if f.select]
            if len(adj_faces) == 1:
                bm_face = adj_faces[0]
                loop_index = next((i for i in range(0, len(bm_face.loops)) if bm_face.loops[i].edge == edge))
                loop = bm_face.loops[loop_index]
                adj_face_sides = face_sides[bm_face]
                verts = []
                for i in range(0, 4):
                    reverse = i == 1 or i == 2
                    e = adj_face_sides[reverse and 1 or 0].link_edges[loop_index]
                    v = loop.vert
                    if i > 1:
                        v = loop.edge.other_vert(v)
                    new_v = e.new_verts[v][0].vert
                    if new_v not in verts:
                        verts.append(new_v)
                if len(verts) > 2:
                    try:
                        new_face = bm.faces.new(verts)
                        if rim == 'FULL' or rim == 'ONLY_FULL':
                            for i in range(0, len(verts)):
                                new_loop = new_face.loops[i]
                                old_loop = i > 1 and loop.link_loop_next or loop
                                new_loop.copy_from(old_loop)
                            new_face.material_index = bm_face.material_index
                    except ValueError:
                        pass
    
    def make_faces(face_sides):
        for bm_face, faces in face_sides.items():
            for face in faces:
                verts = []
                loops = []
                edge = None
                i = 0
                for loop in bm_face.loops:
                    v0 = loop.vert
                    v1 = loop.link_loop_next.vert
                    # find the edge for this face
                    edge = face.link_edges[i]
                    if edge.edge != loop.edge: print("error")
                    flip = edge.verts[0] != v0
                    
                    new_v0 = edge.new_verts[v0][0].vert
                    new_v1 = edge.new_verts[v1][0].vert
                    if len(verts) == 0 or verts[-1] != new_v0:
                        if new_v0 not in verts:
                            verts.append(new_v0)
                            loops.append(loop)
                    if len(verts) <= 1 or verts[0] != new_v1:
                        if new_v1 not in verts:
                            verts.append(new_v1)
                            loops.append(loop.link_loop_next)
                    i += 1
                del i
                if face.reversed:
                    verts.reverse()
                    loops.reverse()
                if len(verts) > 2:
                    try:
                        new_face = bm.faces.new(verts)
                        for i in range(0, len(verts)):
                            new_loop = new_face.loops[i]
                            old_loop = loops[i] # verts.index(new_loop.vert)
                            if face.reversed:
                                new_loop.edge.copy_from(old_loop.link_loop_prev.edge)
                            else:
                                new_loop.edge.copy_from(old_loop.edge)
                            new_loop.copy_from(old_loop)
                        new_face.copy_from(bm_face)
                    except ValueError:
                        pass
    
    def remove_loose_verts(new_verts):
        bmesh.ops.delete(bm, geom=[v for v in new_verts if len(v.link_faces) == 0], context='VERTS')
    
    def remove_original_mesh():
        bmesh.ops.delete(bm, geom=original_faces, context='FACES')
    
    # prepare data
    face_sides = create_face_data(original_faces)
    original_edge_data_map = create_edge_data(original_edges, face_sides)
    original_vert_groups_map = create_groups(original_verts, original_edge_data_map)
    topology_groups = fix_intersections and create_regions(original_vert_groups_map) or None
    # generate new mesh
    new_verts, vert_open_verts_map = make_crossings(original_vert_groups_map)
    if rim != 'NONE':
        make_boundary_faces(original_edges, vert_open_verts_map, original_vert_groups_map, face_sides)
    if rim != 'ONLY_CLEAN' and rim != 'ONLY_FULL':
        make_faces(face_sides)
    # cleanup
    remove_loose_verts(new_verts)
    # remove original mesh
    remove_original_mesh()
    bm.normal_update()

    # finish editing
    bmesh.update_edit_mesh(mesh)

from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty
)

class ManifoldSolidify(bpy.types.Operator):
    """Solidify independent of Normals"""
    bl_idname = "mesh.manifold_solidify"
    bl_label = "Manifold Solidify"
    bl_options = {'REGISTER', 'UNDO'}
    
    thickness: FloatProperty(
        name="Thickness",
        description="Wall Thickness",
        min=0.0,
        unit="LENGTH",
        precision=4,
        default=0.1
    )
    method_items = (
            ('SIMPLE', "Simple", "Simple positioning of the new vertices"),
            ('EVEN', "Even", "Simple positioning of the new vertices but with angle correction"),
            ('CONSTRAINTS', "Constraints", "Space driven positioning of the new vertices to make the thickness perfectly even")
    )
    method: EnumProperty(
            name="Method",
            description="Which way the thickness is controlled",
            items=method_items,
            default='CONSTRAINTS'
    )
    fix_intersections: BoolProperty(
        name="Fix Intersections",
        description="try to fix any intersecting parts",
        default=False
    )
    boundary_fix_items = (
            ('NONE', "None", "No boundary correction"),
            ('INNER', "Inner", "Boundary faces facing inwards"),
            ('OUTER', "Outer", "Boundary faces facing outwards")
    )
    boundary_fix: EnumProperty(
        name="Fix Boundarys",
        description="A simple option to trim exploding corners",
        items=boundary_fix_items,
        default='INNER'
    )
    rim_items = (
            ('NONE', "No Rim", "No rim faces"),
            ('ONLY_CLEAN', "Rim Only (Clean)", "only rim faces, no data transfer"),
            ('ONLY_FULL', "Rim Only (Full)", "only rim faces, full data transfer"),
            ('CLEAN', "Clean", "no data transfer"),
            ('FULL', "Full", "full data transfer")
    )
    rim: EnumProperty(
        name="Rim",
        description="How to fill the boundary faces with appropriate data",
        items=rim_items,
        default='FULL'
    )
    
    def execute(self, context):
        manifold_solidify(thickness=self.thickness,
                even_thickness=self.method=="EVEN",
                high_quality_thickness=self.method=="CONSTRAINTS",
                fix_intersections=self.fix_intersections,
                boundary_fix=self.boundary_fix,
                rim=self.rim)
        
        return {'FINISHED'}


def menu_func(self, context):
    self.layout.operator(ManifoldSolidify.bl_idname)


def register():
    bpy.utils.register_class(ManifoldSolidify)
    bpy.types.VIEW3D_MT_edit_mesh_faces.append(menu_func)


def unregister():
    bpy.utils.unregister_class(ManifoldSolidify)
    bpy.types.VIEW3D_MT_edit_mesh_faces.remove(menu_func)


if __name__ == "__main__":
    register()