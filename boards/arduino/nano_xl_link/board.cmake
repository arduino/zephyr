# SPDX-License-Identifier: Apache-2.0

# Physical silicon is PIC32CK1025GC01144; the Zephyr SoC SKU used here is the
# pin-compatible 2051/144 variant (same die family, same peripherals/pinmux).
board_runner_args(jlink "--device=PIC32CK1025GC" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
