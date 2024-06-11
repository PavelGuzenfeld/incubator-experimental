from calendar import c
import subprocess
import os
from ctypes import *
import clang.cindex

c_to_ctypes_translation = {
    # Standard C types
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
    # stdint.h / cstdint types
    "int8_t": c_int8,
    "uint8_t": c_uint8,
    "int16_t": c_int16,
    "uint16_t": c_uint16,
    "int32_t": c_int32,
    "uint32_t": c_uint32,
    "int64_t": c_int64,
    "uint64_t": c_uint64,
    # If you have other specific types, add them here.
}


def gpp_preprocess(header_path):
    return subprocess.run(
        ["g++", "-E", header_path], check=True, capture_output=True, text=True
    ).stdout


def clang_preprocess(header_path):
    return subprocess.run(
        ["clang", "-E", header_path], check=True, capture_output=True, text=True
    ).stdout


def preprocess_headers(headers, preprocess_function):
    preprocessed_contents = {}

    for header in headers:
        if not os.path.isfile(header):
            print(f"Warning: {header} does not exist.")
            continue

        try:
            preprocessed_contents[header] = preprocess_function(header)
        except subprocess.CalledProcessError as e:
            print(f"Error preprocessing {header}: {e}")

    return preprocessed_contents


def find_struct(node, struct_name):
    """Recursively find the struct definition in the AST node."""
    if (
        node.kind == clang.cindex.CursorKind.STRUCT_DECL
        and node.spelling == struct_name
    ):
        return node
    for child in node.get_children():
        result = find_struct(child, struct_name)
        if result is not None:
            return result
    return None


def make_ctype_struct(header_path: str, struct_name: str, preprocessor=gpp_preprocess):
    # Preprocess the header
    preprocessed = preprocessor(header_path)

    # Set up libclang
    clang.cindex.Config.set_library_file("/usr/lib/llvm-14/lib/libclang.so")
    index = clang.cindex.Index.create()
    translation_unit = index.parse(
        "dummy.cpp",
        args=["-x", "c++", "-std=c++11"],
        unsaved_files=[("dummy.cpp", preprocessed)],
    )

    # Find the struct definition
    struct = find_struct(translation_unit.cursor, struct_name)
    if struct is None:
        raise ValueError(f"Struct {struct_name} not found in {header_path}")

    # Extract fields from the struct
    fields = []
    for field in struct.get_children():
        if field.kind == clang.cindex.CursorKind.FIELD_DECL:
            field_type = field.type.spelling
            if field_type in c_to_ctypes_translation:
                ctype = c_to_ctypes_translation[field_type]
                fields.append((field.spelling, ctype))

    # Dynamically create the ctypes structure
    Struct = type(struct_name, (Structure,), {"_fields_": fields})
    return Struct


if __name__ == "__main__":
    # Example usage
    headers = [
        "/home/pavel/workspace/rocx/rocx_mission_control/include/rocx_mission_control/icd/mmc_fcu_msgs.h",
        "/home/pavel/workspace/rocx/rocx_mission_control/include/rocx_mission_control/icd/fcu_mmc_msgs.h",
    ]

    # Choose your preprocessing function here
    preprocessed = preprocess_headers(headers, gpp_preprocess)
    # Or, for Clang: preprocessed = preprocess_headers(headers, clang_preprocess)

    for header, content in preprocessed.items():
        print(
            f"Preprocessed content of {header}:\n{content[:100]}\n..."
        )  # Print first 500 chars

    # Create a ctypes structure from a header file
    struct = make_ctype_struct(headers[0], "ROCX_VER_S")

    # Create an instance of the structure
    instance = struct()
    print(f"Size of structure: {sizeof(instance)} bytes")
