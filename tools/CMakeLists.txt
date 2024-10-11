SET (AS_COMMANDS
  compAccFromMesh
  3dSDPViewer
  meshViewer
)
SET (AS_TOOLS
  polyAccEdit
)

FOREACH(FILE ${AS_COMMANDS})
  add_executable(${FILE} ${FILE}.cpp ${AS_COMMAND_SRC})
  target_link_libraries(${FILE} ${DGTAL_LIBRARIES})
  target_include_directories(${FILE}
  PUBLIC
    ${PROJECT_SOURCE_DIR}/src
  )
ENDFOREACH(FILE)

FOREACH(FILE ${AS_TOOLS})
  add_executable(${FILE} ${FILE}.cpp ${AS_TOOLS_SRC})
  target_include_directories(${FILE}
  PUBLIC
    ${PROJECT_SOURCE_DIR}/src
  )
  target_link_libraries (${FILE} polyscope ${DGTAL_LIBRARIES})  
  install(TARGETS ${FILE} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
ENDFOREACH(FILE)