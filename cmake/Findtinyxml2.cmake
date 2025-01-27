find_path(TINYXML2_INCLUDE_DIR tinyxml2.h)
find_library(TINYXML2_LIBRARY tinyxml2)

if(TINYXML2_INCLUDE_DIR AND TINYXML2_LIBRARY)
  set(TINYXML2_FOUND TRUE)
  set(TINYXML2_LIBRARIES ${TINYXML2_LIBRARY})
  set(TINYXML2_INCLUDE_DIRS ${TINYXML2_INCLUDE_DIR})
else()
  set(TINYXML2_FOUND FALSE)
endif()

if(NOT TINYXML2_FOUND)
  message(FATAL_ERROR "Could not find tinyxml2")
endif()

