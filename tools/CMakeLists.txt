SET (AS_TOOLS
  compAccFromMesh
  3dSDPViewer
  meshViewer
)

FOREACH(FILE ${AS_TOOLS})
  add_executable(${FILE} ${FILE}.cpp ${AS_SOURCE_FILE})
  target_link_libraries(${FILE} ${DGTAL_LIBRARIES})
  target_include_directories(${FILE}
  PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/ext
  )
ENDFOREACH(FILE)