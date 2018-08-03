ariles_copy_file_if_exists( TGT_ariles_copy_headers                         "format_octave_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/octave.h"    "${ARILES_BRIDGE_INCLUDE_DIR}/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                         "format_octave_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/octave"      "${ARILES_BRIDGE_INCLUDE_DIR}/octave")
add_dependencies("TGT_ariles_copy_headers_format_octave_h" "TGT_ariles_copy_headers_format_octave_dir")
add_dependencies("TGT_ariles_copy_headers_format_octave_h" "TGT_ariles_copy_headers_ariles")


set (TGT_OCTAVE            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
add_custom_target(${TGT_OCTAVE})
