#
#  Copyright (c) 2018 Analog Devices Inc.
#
#  SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(
    hl78xx.c
    hl78xx_sockets.c
    hl78xx_utility.c

)

zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/ip)
<<<<<<< HEAD
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/lib/sockets)
=======
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/lib/sockets)
>>>>>>> 6137499206e6fbb34aa7805f4e3a30768fbd123e
