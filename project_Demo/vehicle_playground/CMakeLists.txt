add_library(ympme_playground STATIC playground.c terrain.c)
target_link_libraries(ympme_playground PUBLIC YMPME::vehicle ymgre)
add_executable(demo_vehicle_playground main.c)
target_link_libraries(demo_vehicle_playground PRIVATE ympme_playground ymgre_demo_host)
if(YMPME_BUILD_TESTS)
	add_executable(test_terrain_io test_vehicle.c)
	target_link_libraries(test_terrain_io PRIVATE ympme_playground)
	add_test(NAME ympme_terrain_io COMMAND test_terrain_io)
	add_test(NAME ympme_vehicle_playground COMMAND demo_vehicle_playground --smoke-test)
	set_tests_properties(ympme_vehicle_playground PROPERTIES
		ENVIRONMENT "YMGRE_HEADLESS=1;YMGRE_MAX_FRAMES=0;YMGRE_WINDOW_SCALE=1"
		TIMEOUT 90)
endif()
