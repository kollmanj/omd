ADD_EXECUTABLE (model2revjointtest ${PROJECT_SOURCE_DIR}/testSource/model2revjointtest/main.cpp)
IF (USE_BULLET)
   TARGET_LINK_LIBRARIES(model2revjointtest ${BULLET_LIBRARIES} OMD)
ELSE (USE_BULLET)
   TARGET_LINK_LIBRARIES(model2revjointtest OMD)
ENDIF (USE_BULLET)

