file(GLOB SOURCES "src/*.cpp")

idf_component_register(
	SRCS 
		"obd2-reader.cpp"
		${SOURCES}
                    
	INCLUDE_DIRS 
		"include"
)
