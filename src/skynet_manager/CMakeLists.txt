cmake_minimum_required(VERSION 3.8)
project(skynet_manager)

###################################################
# Create the compilation database for clangd and move it out of the build dir.
###################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


###################################
# find dependencies
###################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(skynet_interfaces REQUIRED)
find_package(Doxygen REQUIRED)

###################################################
# Doxygen Rules
###################################################
if (DOXYGEN_FOUND)
    set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
    set(DOXYGEN_COLLABORATION_GRAPH YES)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_CLASS_DIAGRAMS YES)
    set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
    set(DOXYGEN_HAVE_DOT YES)
    set(DOXYGEN_CLASS_GRAPH YES)
    set(DOXYGEN_CALL_GRAPH YES)
    set(DOXYGEN_CALLER_GRAPH YES)
    set(DOXYGEN_COLLABORATION_GRAPH YES)
    set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
    set(DOXYGEN_EXTRACT_PRIVATE YES)
    set(DOXYGEN_EXTRACT_PACKAGE YES)
    set(DOXYGEN_EXTRACT_STATIC YES)
    set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
    set(DOXYGEN_UML_LOOK YES)
    set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
    set(DOXYGEN_TEMPLATE_RELATIONS YES)
    set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
    set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
    set(DOXYGEN_DOT_TRANSPARENT YES)
else ()
    message(FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation")
endif ()

###################################
# Specify Targets
###################################
# create a target to build documentation
doxygen_add_docs(docs    
        # List of files or directories
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/test
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    
    set(ament_cmake_copyright_FOUND TRUE)
    
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    
    add_executable(scoutManagerTest
            test/skynet_manager_integration_test.cc
    )
    
    target_link_libraries(scoutManagerTest
            catch_ros2::catch_ros2_with_node_main
    )
    ament_target_dependencies(scoutManagerTest
            rclcpp std_srvs std_msgs skynet_interfaces
    )
    install(TARGETS
            scoutManagerTest
            DESTINATION lib/${PROJECT_NAME}
    )

    catch_ros2_add_integration_test(SkynetScoutManager_Test
            LAUNCH_FILE integration_test.launch.yaml
    )
endif ()


add_executable(manager src/SkynetManager.cc)
target_include_directories(manager PUBLIC include)
ament_target_dependencies(manager rclcpp std_msgs skynet_interfaces tf2 tf2_ros)

install(TARGETS
        manager
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        models
        worlds
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
