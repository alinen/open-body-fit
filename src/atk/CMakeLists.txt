set(SRC
    transform.cpp
    bvhreader.cpp
    bvhwriter.cpp
    skeleton.cpp
    joint.cpp
    pose.cpp
    motion.cpp
    )

add_library(atk STATIC ${SRC})