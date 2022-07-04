function(generate_param_struct_header TARGET OUT_DIR YAML_FILE)
    add_custom_target(
            run ALL
            COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/gen_param_struct.py ${OUT_DIR} ${YAML_FILE} ${YAML_TARGET}
#            BYPRODUCTS ${YAML_TARGET}.h
            COMMENT "Generating yaml struct for ${YAML_FILE}"
    )
    add_dependencies(${TARGET} run)

endfunction()