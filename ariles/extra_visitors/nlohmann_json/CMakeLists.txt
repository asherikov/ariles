set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    find_package(nlohmann_json REQUIRED)

    set(ARILES_VISITOR_${ARILES_VISITOR}_LIBS "nlohmann_json::nlohmann_json")
endif()

add_library(${TGT_ARILES_VISITOR_LIB}
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp
)

target_link_libraries(${TGT_ARILES_VISITOR_LIB} PUBLIC ${ARILES_VISITOR_${ARILES_VISITOR}_LIBS})

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "nlohmann-json3-dev")
