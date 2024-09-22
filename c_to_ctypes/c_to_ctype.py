import subprocess
import os
from ctypes import *
import clang.cindex
import re

# Global cache for enum underlying types
enum_cache = {}

# Translation of C types to ctypes
c_to_ctypes_translation = {
    "int": c_int,
    "unsigned int": c_uint,
    "long": c_long,
    "unsigned long": c_ulong,
    "short": c_short,
    "unsigned short": c_ushort,
    "char": c_char,
    "unsigned char": c_ubyte,
    "float": c_float,
    "double": c_double,
    "char*": c_char_p,
    "void*": c_void_p,
    "bool": c_bool,
    "int8_t": c_int8,
    "uint8_t": c_uint8,
    "int16_t": c_int16,
    "uint16_t": c_uint16,
    "int32_t": c_int32,
    "uint32_t": c_uint32,
    "int64_t": c_int64,
    "uint64_t": c_uint64,
}

def gpp_preprocess(header_path, flags=None):
    if flags is None:
        flags = []
    return subprocess.run(
        ["g++", "-E"] + flags + [header_path], check=True, capture_output=True, text=True
    ).stdout

def preprocess_headers(headers, preprocess_function, flags=None):
    preprocessed_contents = {}
    for header in headers:
        if not os.path.isfile(header):
            print(f"Warning: {header} does not exist.")
            continue
        try:
            preprocessed_contents[header] = preprocess_function(header, flags)
        except subprocess.CalledProcessError as e:
            print(f"Error preprocessing {header}: {e}")
    return preprocessed_contents

def resolve_type(field_type):
    """Resolve namespaced or complex types, including enums."""
    if "::" in field_type:
        field_type = field_type.split("::")[-1]  # Strip namespace

    # Handle enum class types by underlying type
    enum_underlying_type = resolve_enum_underlying_type(field_type)
    if enum_underlying_type:
        return enum_underlying_type

    # Basic types check
    if field_type in c_to_ctypes_translation:
        return c_to_ctypes_translation[field_type]

    # Recursively resolve struct types
    nested_struct = find_struct(translation_unit.cursor, field_type)
    if nested_struct:
        nested_fields = extract_fields_from_struct(nested_struct)
        return type(field_type, (Structure,), {"_fields_": nested_fields})

    print(f"Warning: Unrecognized field type {field_type}")
    return None

def handle_array_type(field_type):
    """Handle C array types like unsigned char[7]."""
    match = re.match(r'(.+)\[(\d+)\]', field_type)
    if match:
        base_type = match.group(1)
        array_size = int(match.group(2))

        resolved_type = resolve_type(base_type)
        if resolved_type:
            return resolved_type * array_size
        print(f"Warning: Unable to resolve base type for array {field_type}")
    return None

def extract_fields_from_struct(struct):
    """Extract fields from the struct, handling nested structs, typedefs, arrays, and bitfields."""
    fields = []
    for field in struct.get_children():
        if field.kind == clang.cindex.CursorKind.FIELD_DECL:
            field_type = field.type.spelling
            bitfield_width = field.get_bitfield_width()

            # Handle bitfields
            if bitfield_width != -1:
                ctype = resolve_type(field_type)
                if ctype:
                    fields.append((field.spelling, ctype, bitfield_width))
                else:
                    print(f"Warning: Could not resolve bitfield {field.spelling} of type {field_type}")
                continue

            # Handle arrays
            if '[' in field_type:
                ctype = handle_array_type(field_type)
            else:
                # Resolve the type
                ctype = resolve_type(field_type)

            if ctype:
                fields.append((field.spelling, ctype))
            else:
                print(f"Warning: Could not resolve field {field.spelling} of type {field_type}")
    return fields

def set_libclang_path():
    try:
        libclang_path = subprocess.check_output(['llvm-config', '--libdir']).decode().strip()
        clang.cindex.Config.set_library_file(os.path.join(libclang_path, 'libclang.so'))
    except subprocess.CalledProcessError as e:
        print(f"Failed to set libclang path: {e}")
        raise

def find_struct(node, struct_name):
    if node.kind == clang.cindex.CursorKind.STRUCT_DECL:
        print(f"Found struct: '{node.spelling or 'anonymous'}'")
    if node.kind == clang.cindex.CursorKind.TYPEDEF_DECL and node.spelling == struct_name:
        print(f"Found typedef: '{node.spelling}'")
        for child in node.get_children():
            if child.kind == clang.cindex.CursorKind.STRUCT_DECL:
                return child
    for child in node.get_children():
        result = find_struct(child, struct_name)
        if result is not None:
            return result
    return None

def cache_enum_types(node):
    """Cache enum types and their underlying sizes."""
    if node.kind == clang.cindex.CursorKind.ENUM_DECL:
        enum_name = node.spelling
        underlying_type = node.enum_type.spelling if node.enum_type else "int"  # Default to int if not specified

        # Resolve the underlying type using our map or default it to `int`
        underlying_ctype = resolve_type(underlying_type)
        enum_cache[enum_name] = underlying_ctype
        print(f"Cached enum {enum_name} with underlying type {underlying_ctype}")

        # Traverse enum fields to ensure all cases are processed
        for enum_field in node.get_children():
            print(f"Enum Field: {enum_field.spelling} -> Value: {enum_field.enum_value}")


def resolve_enum_underlying_type(enum_name):
    """Resolve enum class types to their underlying types using cache."""
    if enum_name in enum_cache:
        return enum_cache[enum_name]
    else:
        print(f"Warning: Enum {enum_name} not cached.")
        return None

def make_ctype_struct(header_path: str, struct_name: str, preprocessor=gpp_preprocess, flags=None):
    set_libclang_path()
    preprocessed = preprocessor(header_path, flags)
    index = clang.cindex.Index.create()
    global translation_unit
    translation_unit = index.parse(
        "dummy.cpp",
        args=["-x", "c++", "-std=c++11"],
        unsaved_files=[("dummy.cpp", preprocessed)],
    )

    # Cache enums during parsing
    for node in translation_unit.cursor.get_children():
        cache_enum_types(node)

    struct = find_struct(translation_unit.cursor, struct_name)
    if struct is None:
        raise ValueError(f"Struct {struct_name} not found in {header_path}")
    fields = extract_fields_from_struct(struct)
    if not fields:
        raise ValueError(f"No fields found in struct {struct_name}")
    return type(struct_name, (Structure,), {"_fields_": fields, "_pack_": 1})

def print_structure_hierarchy(struct, indent=0):
    """Recursively print the structure hierarchy."""
    print(" " * indent + f"Struct: {struct.__name__}")
    for field in struct._fields_:
        if len(field) == 3:  # Bitfield
            print(" " * (indent + 2) + f"Field: {field[0]} -> Bitfield ({field[2]} bits) of {field[1].__name__}")
        elif issubclass(field[1], Structure):
            print(" " * (indent + 2) + f"Field: {field[0]} -> Nested Struct")
            print_structure_hierarchy(field[1], indent + 4)
        elif isinstance(field[1], type) and hasattr(field[1], "_length_"):
            print(" " * (indent + 2) + f"Field: {field[0]} -> Array [{field[1]._length_}] of {field[1]._type_.__name__}")
        else:
            print(" " * (indent + 2) + f"Field: {field[0]} -> {field[1].__name__}")

if __name__ == "__main__":
    headers = [
        "/home/pavel/workspace/rocx/rocx_mission_control/include/rocx_mission_control/icd/mmc_fcu_msgs.h",
    ]
    flags = ["-DMY_DEFINE=1"]
    preprocessed = preprocess_headers(headers, gpp_preprocess, flags)
    struct_name = "MCC_FCU_TLM_DATA_S"
    struct = make_ctype_struct(headers[0], struct_name, gpp_preprocess, flags)

    instance = struct()
    print(f"Size of structure: {sizeof(instance)} bytes")
    print(f"Fields of the structure: {instance._fields_}")
    
    # Print the hierarchical structure
    print("\nStructure hierarchy:")
    print_structure_hierarchy(struct)

    print(f"\nInstance of the structure: {instance}")
    print(f"Instance of the structure as bytes: {len(bytes(instance))}")
