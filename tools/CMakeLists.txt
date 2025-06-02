SET (AS_TOOLS
  polyAccEdit
  polyMeshEdit
)

FOREACH(FILE ${AS_TOOLS})
  add_executable(${FILE} ${FILE}.cpp)
  target_include_directories(${FILE}
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${DGTAL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/deps/polyscope/deps/glm
  )
  target_link_libraries (${FILE} polyscope ${DGTAL_LIBRARIES})  
  install(TARGETS ${FILE} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
ENDFOREACH(FILE)

# DGtal tools
# SET (AS_COMMANDS
#   compAccFromMesh
#   3dSDPViewer
#   meshViewer
# )
# FOREACH(FILE ${AS_COMMANDS})
#   add_executable(${FILE} ${FILE}.cpp)
#   target_link_libraries(${FILE} ${DGTAL_LIBRARIES})
#   target_include_directories(${FILE}
#   PUBLIC
#     ${PROJECT_SOURCE_DIR}/include
#     ${PROJECT_SOURCE_DIR}/src
#     ${DGTAL_INCLUDE_DIRS}
#   )
# ENDFOREACH(FILE)