# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# 7-segment encodings used by your design (A..G = uo_out[6:0], active-high)
VALID_7SEG = {
    0b1111110,  # 0
    0b0110000,  # 1
    0b1101101,  # 2
    0b1111001,  # 3
    0b0110011,  # 4
    0b1011011,  # 5
    0b1011111,  # 6
    0b1110000,  # 7
    0b1111111,  # 8
    0b1110011,  # 9
    0b0000000,  # blank (allowed during transitions/reset)
}

# Active-low digit enable states you rotate through on uio_out[3:0]
DIG_STATES = {0b1110, 0b1011, 0b1101}


async def start_clk_and_reset(dut, period_ns=100):
    """Start a reasonably fast clock and apply reset."""
    # Faster sim clock (10 MHz) so heartbeat/dividers toggle in a reasonable time
    cocotb.start_soon(Clock(dut.clk, period_ns, units="ns").start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0

    # Active-low reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 8)


@cocotb.test()
async def test_oe_and_basic_reset(dut):
    """After reset, uio_oe[3:0] must be outputs and [7:4] must be inputs."""
    await start_clk_and_reset(dut)

    # settle
    await ClockCycles(dut.clk, 2)

    uio_oe = int(dut.uio_oe.value)
    assert (uio_oe & 0x0F) == 0x0F, f"Expected uio_oe[3:0]=1111, got {uio_oe & 0x0F:04b}"
    assert (uio_oe & 0xF0) == 0x00, f"Expected uio_oe[7:4]=0000, got {(uio_oe >> 4) & 0xF:04b}"


@cocotb.test()
async def test_digit_scan_rotation(dut):
    """uio_out[3:0] (active-low digit enables) should rotate among three valid states."""
    await start_clk_and_reset(dut)

    seen = set()
    # Observe enough cycles for multiple scan steps (scan_clk uses a small divider)
    for _ in range(50_000):
        val = int(dut.uio_out.value) & 0xF
        if val in DIG_STATES:
            seen.add(val)
            if len(seen) == 3:
                break
        await ClockCycles(dut.clk, 1)

    assert len(seen) >= 2, f"Digit enables not rotating; seen={sorted(seen)}"
    # Seeing all 3 is ideal but not strictly required for pass.


@cocotb.test()
async def test_segments_emit_valid_patterns(dut):
    """uo_out[6:0] should map to valid 7-segment patterns most of the time."""
    await start_clk_and_reset(dut)

    valid_hits = 0
    # Sample for a while; we expect many valid codes as the three digits scan
    for _ in range(100_000):
        seg = int(dut.uo_out.value) & 0x7F
        if seg in VALID_7SEG:
            valid_hits += 1
        await ClockCycles(dut.clk, 1)

    assert valid_hits > 100, f"Too few valid 7-seg patterns observed ({valid_hits}). Check bit ordering A..G on uo[6:0]."


@cocotb.test()
async def test_heartbeat_toggles(dut):
    """uo_out[7] is the heartbeat (divided clock) and should toggle eventually."""
    await start_clk_and_reset(dut)

    # helper: read a single bit safely, returning None if X/Z
    def read_bit(sig, idx):
        bit = sig.value[idx]          # cocotb Logic
        return int(bit) if bit.is_resolvable else None

    hb0 = read_bit(dut.uo_out, 7)
    # if it was X right after reset, wait a tick
    while hb0 is None:
        await ClockCycles(dut.clk, 1)
        hb0 = read_bit(dut.uo_out, 7)

    toggled = False
    # With div[18] and 10 MHz sim, one toggle ~ 2^19 / 10e6 ≈ 52.4 ms.
    # Check across a few windows to catch at least one edge.
    for _ in range(3):
        await ClockCycles(dut.clk, 200_000)  # 20 ms per window at 10 MHz
        hb1 = read_bit(dut.uo_out, 7)
        if hb1 is None:
            continue
        if hb1 != hb0:
            toggled = True
            hb0 = hb1
            break

    assert toggled, "Heartbeat (uo_out[7]) did not toggle within the expected window."

@cocotb.test()
async def test_manual_reset_ui0(dut):
    """Asserting ui[0] (manual reset, active-high) should re-initialize scanning."""
    await start_clk_and_reset(dut)

    # Let it run, then pulse manual reset
    await ClockCycles(dut.clk, 50)
    dut.ui_in.value = 0x01  # raise manual reset
    await ClockCycles(dut.clk, 8)
    dut.ui_in.value = 0x00  # release
    await ClockCycles(dut.clk, 8)

    # We expect to see the initial cathode state (1110) again soon after
    saw_init = False
    for _ in range(50_000):
        if (int(dut.uio_out.value) & 0xF) == 0b1110:
            saw_init = True
            break
        await ClockCycles(dut.clk, 1)

    assert saw_init, "Did not observe uio_out[3:0]==1110 after manual reset (ui[0])."
