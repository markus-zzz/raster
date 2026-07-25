"""
Blender 4.0 exporter: writes mesh vertex/face data to a C header using the
GpuVertex_t / GpuFace_t structs from gamebub-tiny/raster/fw/gputypes.h.

Fixed-point format: fx is int32_t in Q20.12 (1.0 == 1 << 12 == 4096).

Usage
-----
1. As an add-on: Edit > Preferences > Add-ons > Install..., pick this file,
   enable it, then File > Export > GPU Mesh C Header (.h).
2. From the Text Editor / command line: edit CONFIG below and press "Run Script"
   (or `blender file.blend --background --python export_gpumesh.py`).

The generated header contains, for each exported mesh object:
    static const GpuVertex_t <name>_vtx[] = { ... };
    static const GpuFace_t   <name>_face[]    = { ... };
"""

import bpy
import bmesh
import re
import os
from mathutils import Vector

# Q20.12 fixed point
FXSH = 12
FX_ONE = 1 << FXSH
FX_MIN = -(1 << 31)
FX_MAX = (1 << 31) - 1


# --------------------------------------------------------------------------- #
# Configuration (used when run directly, not through the File > Export dialog)
# --------------------------------------------------------------------------- #
CONFIG = {
    "filepath": os.path.join(os.path.dirname(bpy.data.filepath or "."), "mesh.h"),
    "selected_only": False,   # export only selected objects, else all meshes
    "apply_modifiers": True,  # evaluate modifier stack before export
    "yup": True,              # convert Blender Z-up to Y-up (swap/flip axes)
    "srgb": True,             # gamma-encode linear material color to sRGB
}


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def to_fx(value):
    """Convert a float to a clamped Q20.12 fixed-point int32."""
    v = int(round(value * FX_ONE))
    if v < FX_MIN:
        v = FX_MIN
    elif v > FX_MAX:
        v = FX_MAX
    return v


def c_ident(name):
    """Sanitize an object name into a valid C identifier."""
    ident = re.sub(r"[^0-9a-zA-Z_]", "_", name)
    if not ident or ident[0].isdigit():
        ident = "_" + ident
    return ident


def convert_axis(v, yup):
    """Optionally convert Blender (Z-up) coords to a Y-up convention."""
    if yup:
        return Vector((v.x, v.z, -v.y))
    return Vector((v.x, v.y, v.z))


DEFAULT_COLOR = (0.8, 0.8, 0.8)


def linear_to_srgb(c):
    """Encode a single linear color channel (0..1) to sRGB (0..1)."""
    if c <= 0.0:
        return 0.0
    if c <= 0.0031308:
        return 12.92 * c
    return 1.055 * (c ** (1.0 / 2.4)) - 0.055


def material_color(mat):
    """Return an (r, g, b) tuple in 0..1 for a single material."""
    if mat is None:
        return DEFAULT_COLOR
    # Prefer the Principled BSDF base color, fall back to the viewport
    # diffuse color which always exists.
    if mat.use_nodes and mat.node_tree:
        for node in mat.node_tree.nodes:
            if node.type == "BSDF_PRINCIPLED":
                col = node.inputs["Base Color"].default_value
                return (col[0], col[1], col[2])
    col = mat.diffuse_color
    return (col[0], col[1], col[2])


def color_table(obj, srgb):
    """Return a list of (r, g, b) colors, one per material slot.

    Indexed by a face's material_index. Always has at least one entry.
    Blender stores base colors linear; when srgb is set they are gamma-encoded
    so the exported values match a display-referred (sRGB) framebuffer.
    """
    if obj.data.materials:
        cols = [material_color(mat) for mat in obj.data.materials]
    else:
        cols = [DEFAULT_COLOR]
    if srgb:
        cols = [tuple(linear_to_srgb(c) for c in rgb) for rgb in cols]
    return cols


def gather_mesh(obj, depsgraph, apply_modifiers, yup, srgb):
    """Return (vertices, faces) lists of tuples for a single object.

    vertices: [(x, y, z), ...]           floats in object-local space
    faces:    [(i0, i1, i2, nx, ny, nz, r, g, b), ...]
    """
    if apply_modifiers:
        eval_obj = obj.evaluated_get(depsgraph)
        mesh = eval_obj.to_mesh()
    else:
        mesh = obj.data

    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    colors = color_table(obj, srgb)

    vertices = []
    for v in bm.verts:
        p = convert_axis(v.co, yup)
        vertices.append((p.x, p.y, p.z))

    faces = []
    for f in bm.faces:
        i0, i1, i2 = (v.index for v in f.verts)
        # Store the *negated* unit face normal. The GPU pipeline (see
        # raster/tb_system.cpp process_face) rotates this normal and uses the
        # result directly for back-face culling (n.z >= 0) and lighting,
        # without recomputing it from the transformed edges. The runtime view
        # normal is -R*n_obj (R carries det=-1 from the Y-flip that maps screen
        # y downward), so pre-negating here lets the pipeline consume the
        # transform output as-is. bmesh face normals are already unit length.
        n = convert_axis(f.normal, yup)
        # Pick the color of the material assigned to this face.
        idx = f.material_index if f.material_index < len(colors) else 0
        r, g, b = colors[idx]
        faces.append((i0, i1, i2, -n.x, -n.y, -n.z, r, g, b))

    bm.free()
    if apply_modifiers:
        eval_obj.to_mesh_clear()

    return vertices, faces


def emit_object(out, name, vertices, faces):
    ident = c_ident(name)
    upper = ident.upper()

    out.append("/* ---- %s : %d vertices, %d faces ---- */"
               % (name, len(vertices), len(faces)))
    out.append("")

    out.append("static const GpuVertex_t %s_vtx[] = {" % (ident))
    for (x, y, z) in vertices:
        out.append("    { .x = %11d, .y = %11d, .z = %11d, ._pad = 0 },"
                   % (to_fx(x), to_fx(y), to_fx(z)))
    out.append("};")
    out.append("")

    out.append("static const GpuFace_t %s_face[] = {" % (ident))
    for (i0, i1, i2, nx, ny, nz, r, g, b) in faces:
        out.append(
            "    { .i0 = %d, .i1 = %d, .i2 = %d, "
            ".nx = %d, .ny = %d, .nz = %d, "
            ".r = %d, .g = %d, .b = %d, ._pad = 0 },"
            % (i0, i1, i2,
               to_fx(nx), to_fx(ny), to_fx(nz),
               to_fx(r), to_fx(g), to_fx(b))
        )
    out.append("};")
    out.append("")


def export_header(filepath, selected_only, apply_modifiers, yup, srgb):
    depsgraph = bpy.context.evaluated_depsgraph_get()

    if selected_only:
        objects = [o for o in bpy.context.selected_objects if o.type == "MESH"]
    else:
        objects = [o for o in bpy.data.objects if o.type == "MESH"]

    if not objects:
        raise RuntimeError("No mesh objects to export.")

    out = []
    out.append("/* Auto-generated by export_gpumesh.py from Blender. */")
    out.append("/* Fixed-point fx is Q20.12 (1.0 == %d). */" % FX_ONE)
    out.append("#pragma once")
    out.append("")
    out.append('#include "gputypes.h"')
    out.append("")

    for obj in objects:
        verts, faces = gather_mesh(obj, depsgraph, apply_modifiers, yup, srgb)
        emit_object(out, obj.name, verts, faces)

    out.append("")

    with open(filepath, "w") as fh:
        fh.write("\n".join(out))

    print("Exported %d object(s) to %s" % (len(objects), filepath))
    return len(objects)


# --------------------------------------------------------------------------- #
# Blender operator + File > Export menu integration
# --------------------------------------------------------------------------- #
from bpy.props import StringProperty, BoolProperty
from bpy_extras.io_utils import ExportHelper


class ExportGpuMesh(bpy.types.Operator, ExportHelper):
    """Export mesh data to a C header using GpuVertex_t / GpuFace_t"""
    bl_idname = "export_mesh.gpu_header"
    bl_label = "Export GPU Mesh C Header"
    bl_options = {"PRESET"}

    filename_ext = ".h"
    filter_glob: StringProperty(default="*.h", options={"HIDDEN"})

    selected_only: BoolProperty(
        name="Selected Only",
        description="Export only selected mesh objects",
        default=False,
    )
    apply_modifiers: BoolProperty(
        name="Apply Modifiers",
        description="Evaluate the modifier stack before exporting",
        default=True,
    )
    yup: BoolProperty(
        name="Y-up",
        description="Convert Blender Z-up coordinates to Y-up",
        default=True,
    )
    srgb: BoolProperty(
        name="sRGB Color",
        description="Gamma-encode linear material color to sRGB",
        default=True,
    )

    def execute(self, context):
        try:
            n = export_header(self.filepath, self.selected_only,
                              self.apply_modifiers, self.yup, self.srgb)
        except Exception as exc:  # noqa: BLE001
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, "Exported %d object(s)" % n)
        return {"FINISHED"}


def menu_func_export(self, context):
    self.layout.operator(ExportGpuMesh.bl_idname, text="GPU Mesh C Header (.h)")


classes = (ExportGpuMesh,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)
    for cls in classes:
        bpy.utils.unregister_class(cls)


bl_info = {
    "name": "Export GPU Mesh C Header",
    "author": "generated",
    "version": (1, 0, 0),
    "blender": (4, 0, 0),
    "location": "File > Export > GPU Mesh C Header (.h)",
    "description": "Export meshes to a C header using GpuVertex_t/GpuFace_t.",
    "category": "Import-Export",
}


if __name__ == "__main__":
    # Run directly: register the menu entry, and if launched headless with a
    # configured output path, perform the export immediately.
    try:
        register()
    except Exception:
        pass

    if bpy.app.background:
        export_header(
            CONFIG["filepath"],
            CONFIG["selected_only"],
            CONFIG["apply_modifiers"],
            CONFIG["yup"],
            CONFIG["srgb"],
        )
