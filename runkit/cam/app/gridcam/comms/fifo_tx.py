# Non-blocking send of the per-frame bytes: straight into the UART's
# hardware TX FIFO (uart.fast_tx, gridcam 1.27.1).
#
# Why: on the alif port UART.write() hands the bytes to the TX interrupt and
# then WAITS until the last one is in the FIFO (ports/alif/mpuart.c,
# mp_uart_tx_data), so a frame's 18 bytes cost ~1.6 ms of the vision loop at
# 115200. The UART has a 32-byte FIFO (Alif drivers/include/uart.h,
# UART_FIFO_DEPTH) that shifts bytes out by itself; 18 fit with room to spare,
# and the previous frame's bytes are long gone by the next frame.
#
# Register map: Alif soc.h (AE302F80F55D5 and AE722F80F55D5, rtss_hp):
# UART1_BASE 0x49019000, Synopsys DW_apb_uart layout. machine.UART(1) is
# uart_id 1 (ports/alif/machine_uart.c), the QWIIC pins.
#
# Nothing is written until the block has proved what it is (component type,
# FIFO present, 8 data bits, divisor latch off -- with DLAB set, offset 0 is
# the baud divisor, not THR). Only THR is ever written.

UART_BASE = {1: 0x49019000}
THR = 0x00
LCR = 0x0C
TFL = 0x80
CPR = 0xF4
CTR = 0xFC
DW_CTR = 0x44570110          # 'DW' + UART: DesignWare component type


def _mem32():
    import machine
    return machine.mem32


class FifoTx:
    def __init__(self, uart_id, mem=None):
        base = UART_BASE.get(uart_id)
        if base is None:
            raise ValueError('UART%d: no register map' % uart_id)
        m = mem if mem is not None else _mem32()
        ctr = m[base + CTR] & 0xFFFFFFFF
        if ctr != DW_CTR:
            raise ValueError('UART%d CTR 0x%08x, not DW_apb_uart' % (uart_id, ctr))
        depth = ((m[base + CPR] >> 16) & 0xFF) * 16
        if depth < 16:
            raise ValueError('UART%d has no TX FIFO (CPR)' % uart_id)
        lcr = m[base + LCR] & 0xFF
        if lcr & 0x80 or (lcr & 0x03) != 0x03:
            raise ValueError('UART%d LCR 0x%02x: not 8 bits / DLAB set'
                             % (uart_id, lcr))
        self._m = m
        self._thr = base + THR
        self._tfl = base + TFL
        self._lcr = base + LCR
        self.depth = depth

    def write(self, data):
        """All of `data` into the FIFO and True, or nothing and False."""
        m = self._m
        if m[self._lcr] & 0x80:              # a baud change in progress
            return False
        if self.depth - (m[self._tfl] & 0xFF) < len(data):
            return False
        thr = self._thr
        for b in data:
            m[thr] = b
        return True
