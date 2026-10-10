if(NOT TARGET ymgre_demo_host)
 message(FATAL_ERROR "statics_viewer requires the GRE window host")
endif()
add_executable(demo_statics_viewer main.c)
target_link_libraries(demo_statics_viewer PRIVATE YMPME::statics ymgre_demo_host)
# External GRE/GUI headers keep their own warning policy.
target_include_directories(demo_statics_viewer SYSTEM PRIVATE
 $<TARGET_PROPERTY:ymgre_demo_host,INTERFACE_INCLUDE_DIRECTORIES>)
if(NOT MSVC)
 target_compile_options(demo_statics_viewer PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
if(YMPME_BUILD_TESTS)
 add_test(NAME ympme_statics_viewer COMMAND demo_statics_viewer --smoke-test)
 set_tests_properties(ympme_statics_viewer PROPERTIES
  ENVIRONMENT "YMGRE_HEADLESS=1;YMGRE_WINDOW_SCALE=1;YMGRE_MAX_FRAMES=0" TIMEOUT 30)
endif()
