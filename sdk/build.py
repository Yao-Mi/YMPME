#!/usr/bin/env python3
"""Build paired 16/32-bit all-in-one PME archives and validate native GRE sharing."""
import argparse
import hashlib
import json
import re
import shutil
import tempfile
from pathlib import Path
from build_modular import ROOT, main as build_variant, run, source_hash

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"build/YMPME_libs")
    parser.add_argument("--jobs",type=int,default=4)
    parser.add_argument("--gre-sdk",type=Path,default=ROOT.parent/"0502_YMGRE/build/YMGRE_libs")
    args=parser.parse_args()
    output=args.output.resolve(); build=(ROOT/"build").resolve()
    variants=build/"sdk-variants"
    if build not in output.parents or variants==output or variants in output.parents or output in variants.parents:
        parser.error("Output must be a dedicated directory under build, outside sdk-variants")
    if output.exists() and not (output/".ympme-sdk").is_file():
        parser.error("Refusing to replace a directory without .ympme-sdk marker")
    if not (args.gre_sdk/"cmake/YMGREConfig.cmake").is_file():
        parser.error("--gre-sdk must point to the paired GRE SDK for native sharing validation")
    before=source_hash()
    for bits in (16,32):
        build_variant(True,["--index-bits",str(bits),"--jobs",str(args.jobs),
            "--output",str(variants/f"index{bits}/YMPME_libs")])
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".pme-dual-stage-",dir=output.parent) as temp:
        stage=Path(temp)/"YMPME_libs"
        shutil.copytree(variants/"index32/YMPME_libs",stage)
        for folder in ("lib","cmake","bin","verification"):
            shutil.rmtree(stage/folder); (stage/folder).mkdir()
        components=re.search(r'set\(YMPME_AVAILABLE_COMPONENTS "([^"]+)"\)',
            (variants/"index32/YMPME_libs/cmake/YMPMEConfig.cmake").read_text()).group(1)
        definitions=re.search(r'INTERFACE_COMPILE_DEFINITIONS "([^"]+)"',
            (variants/"index32/YMPME_libs/cmake/YMPMETargets.cmake").read_text()).group(1)
        definitions=definitions.replace("YMPME_INDEX_BITS=32","YMPME_INDEX_BITS=${YMPME_INDEX_BITS}")
        for bits in (16,32):
            v=variants/f"index{bits}/YMPME_libs"
            shutil.copy2(v/"lib/libympme.a",stage/f"lib/libympme_index{bits}.a")
            shutil.copytree(v/"verification",stage/f"verification/index{bits}")
        # One header set, matching the GRE SDK default. Explicit -D selects direct-cc builds.
        header=(ROOT/"YMPME/CONFIG/YMPME_Index.h").read_text().replace("#define YMPME_INDEX_BITS 32","#define YMPME_INDEX_BITS 16")
        (stage/"include/YMPME/CONFIG/YMPME_Index.h").write_text(header)
        config=r"""get_filename_component(_pme_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
if(NOT DEFINED YMPME_INDEX_BITS)
    set(YMPME_INDEX_BITS 16 CACHE STRING "PME mesh index width")
endif()
if(NOT YMPME_INDEX_BITS STREQUAL "16" AND NOT YMPME_INDEX_BITS STREQUAL "32")
    message(FATAL_ERROR "YMPME_INDEX_BITS must be 16 or 32")
endif()
if(TARGET YMPME::ympme)
    get_target_property(_bits YMPME::ympme YMPME_SDK_INDEX_BITS)
    if(NOT _bits STREQUAL "${YMPME_INDEX_BITS}")
        message(FATAL_ERROR "Use one PME index width per consumer build directory")
    endif()
else()
    add_library(YMPME::ympme STATIC IMPORTED)
    set_target_properties(YMPME::ympme PROPERTIES
        IMPORTED_LOCATION "${_pme_root}/lib/libympme_index${YMPME_INDEX_BITS}.a"
        YMPME_SDK_INDEX_BITS "${YMPME_INDEX_BITS}"
        INTERFACE_INCLUDE_DIRECTORIES "${_pme_root}/include/YMPME"
        INTERFACE_COMPILE_DEFINITIONS "@DEFINITIONS@"
        INTERFACE_LINK_LIBRARIES m)
endif()
set(YMPME_SDK_UNIFIED ON)
set(YMPME_AVAILABLE_COMPONENTS "@COMPONENTS@")
foreach(_component IN LISTS YMPME_AVAILABLE_COMPONENTS)
    if(NOT TARGET YMPME::${_component})
        add_library(YMPME::${_component} INTERFACE IMPORTED)
        set_target_properties(YMPME::${_component} PROPERTIES INTERFACE_LINK_LIBRARIES YMPME::ympme)
    endif()
endforeach()
set(YMPME_FOUND TRUE)
foreach(_component IN LISTS YMPME_FIND_COMPONENTS)
    if(_component IN_LIST YMPME_AVAILABLE_COMPONENTS)
        set(YMPME_${_component}_FOUND TRUE)
    elseif(YMPME_FIND_REQUIRED_${_component})
        set(YMPME_FOUND FALSE)
        set(YMPME_NOT_FOUND_MESSAGE "Unknown PME component '${_component}'")
    endif()
endforeach()
"""
        (stage/"cmake/YMPMEConfig.cmake").write_text(config.replace("@DEFINITIONS@",definitions).replace("@COMPONENTS@",components))
        shutil.copy2(variants/"index32/YMPME_libs/cmake/YMPMEConfigVersion.cmake",stage/"cmake")
        with tempfile.TemporaryDirectory(prefix="pme GRE relocated ",dir=build) as moved_dir:
            moved=Path(moved_dir)/"YMPME_libs"; shutil.copytree(stage,moved)
            for bits in (16,32):
                dest=Path(moved_dir)/f"consumer{bits}"; log=stage/f"verification/native-gre-index{bits}.log"
                run(["cmake","-S",moved/"examples","-B",dest,"-DCMAKE_BUILD_TYPE=Release",
                     f"-DYMPME_DIR={moved/'cmake'}",f"-DYMPME_INDEX_BITS={bits}",
                     "-DYMPME_SDK_WITH_GRE=ON",f"-DYMGRE_DIR={args.gre_sdk.resolve()/'cmake'}"],log)
                run(["cmake","--build",dest,"--parallel",args.jobs],log)
                run(["ctest","--test-dir",dest,"--output-on-failure"],log)
                shutil.copytree(dest/"bin",stage/f"bin/index{bits}")
                # Whole-archive checks prove every function links from the final renamed archive.
                cc="cc"; probe=Path(moved_dir)/f"all{bits}"
                src=Path(moved_dir)/"all.c"; src.write_text('#include "YMPME.h"\nint main(void){return 0;}\n')
                run([cc,"-std=c99",f"-DYMPME_INDEX_BITS={bits}","-I",moved/"include/YMPME",src,
                    "-Wl,--whole-archive",moved/f"lib/libympme_index{bits}.a","-Wl,--no-whole-archive","-lm","-o",probe],log)
                run([probe],log)
        if source_hash()!=before:
            raise RuntimeError("Source changed during packaging; rerun for a consistent snapshot")
        info=json.loads((variants/"index32/YMPME_libs/BUILD_INFO.json").read_text())
        info.update(index_bits=[16,32],layout="unified-dual-index",default_index_bits=16,
            static_libraries=["libympme_index16.a","libympme_index32.a"],
            native_gre_sharing_verified=True,source_sha256=before,rebuild_command="python3 sdk/build.py")
        (stage/"BUILD_INFO.json").write_text(json.dumps(info,ensure_ascii=False,indent=2)+"\n")
        (stage/"checksums.sha256").unlink(missing_ok=True)
        (stage/"checksums.sha256").write_text("\n".join(hashlib.sha256(p.read_bytes()).hexdigest()+"  "+p.relative_to(stage).as_posix()
            for p in sorted(stage.rglob("*")) if p.is_file())+"\n")
        backup=output.with_name(output.name+".previous")
        if backup.exists(): raise RuntimeError(f"Backup exists: {backup}")
        if output.exists(): output.rename(backup)
        try: stage.rename(output)
        except Exception:
            if backup.exists(): backup.rename(output)
            raise
        if backup.exists(): shutil.rmtree(backup)
    print(f"Dual-index PME SDK ready: {output}; native GRE sharing PASS for 16 and 32 bits",flush=True)
if __name__=="__main__":
    main()
