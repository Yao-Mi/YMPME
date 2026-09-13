add_library(ympme_car_scene STATIC scene.c)
target_link_libraries(ympme_car_scene PUBLIC YMPME::drive ymgre)
add_executable(demo_three_wheel_car main.c)
target_link_libraries(demo_three_wheel_car PRIVATE ympme_car_scene ymgre_demo_host)
if(YMPME_BUILD_TESTS)
	add_executable(test_car_drive test_drive.c)
	target_link_libraries(test_car_drive PRIVATE ympme_car_scene)
	add_test(NAME ympme_car_drive COMMAND test_car_drive)
	add_test(NAME ympme_car_window COMMAND demo_three_wheel_car --smoke-test)
	set_tests_properties(ympme_car_window PROPERTIES
		ENVIRONMENT "YMGRE_HEADLESS=1;YMGRE_WINDOW_SCALE=1;YMGRE_MAX_FRAMES=0"
		TIMEOUT 45)
endif()
