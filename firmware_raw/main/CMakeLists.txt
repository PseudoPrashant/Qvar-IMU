idf_component_register(SRCS "main.c"
                            "imu.c"
                            "ism330bx_reg.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver)
