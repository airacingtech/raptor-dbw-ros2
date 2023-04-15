cmake_minimum_required(VERSION 3.8)
project(raptor_dbw_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/DbwMisc.msg"
	"msg/DbwAccelpdlreport.msg"
	"msg/DbwSteeringreport.msg"
	"msg/DbwBrakereport.msg"
	"msg/DbwPrndreport.msg"
	"msg/DbwWheelpositionreport.msg"
	"msg/AkitAccelpdlrequest.msg"
	"msg/AkitGlobalenbl.msg"
	"msg/AkitOtheractuators.msg"
	"msg/AkitSteeringrequest.msg"
	"msg/AkitBrakerequest.msg"
	"msg/AkitPrndrequest.msg"
	"msg/DbwDriverinputs.msg"
	"msg/DbwTirepressreport.msg"
	"msg/DbwVinreport.msg"
	"msg/DbwReserved1.msg"
	"msg/DbwImureport.msg"
	"msg/DbwWheelspeedreport.msg"
	"msg/DbwRadarsonar.msg"
	"msg/DbwFaulttext.msg"
	"msg/DbwLowvoltsysreport.msg"
	"msg/DbwBrakereport2.msg"
	"msg/DbwSteeringreport2.msg"
	"msg/DbwOtheractuatorsreport.msg"
	"msg/DbwFaultactionsreport.msg"
	"msg/DbwGpsreference.msg"
	"msg/DbwGpsremainder.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES
        std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()