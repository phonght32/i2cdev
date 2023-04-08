set(srcs
    "i2cdev.c")

set(includes ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "." ${includes}
                       REQUIRES driver)