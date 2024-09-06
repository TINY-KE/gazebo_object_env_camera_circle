# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;test_pkg;gazebo_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lconnect_gazebo".split(';') if "-lconnect_gazebo" != "" else []
PROJECT_NAME = "connect_gazebo"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
