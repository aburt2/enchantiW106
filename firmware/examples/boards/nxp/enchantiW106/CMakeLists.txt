#
# Copyright 2022-2023 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

if(CONFIG_NXP_RW6XX_BOOT_HEADER)
  zephyr_compile_definitions(BOARD_FLASH_SIZE=CONFIG_FLASH_SIZE*1024)
  zephyr_library()
  # This FCB is specific to the flash on this board, it won't work
  # for boards with different flash chips. If you flash this FCB
  # onto a board with a different flash chip you may break it.
  # See MCUXpresso config tools for making a correct one.
  zephyr_library_sources(fidelix.c)
endif()
