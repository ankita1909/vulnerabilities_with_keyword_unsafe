source [find interface/kitprog3.cfg]
source [find target/psoc6_2m.cfg]
${TARGET}.cm0 configure -gdb-port disabled
${TARGET}.cm4 configure -rtos auto -rtos-wipe-on-reset-halt 1