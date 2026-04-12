"""
serial_worker.py
================
Runs serial I/O in a dedicated background thread so the ROS2 executor
is never blocked waiting on the ESP32.

Design
------
- One thread owns the serial port (open, read, write).
- Commands are pushed onto a thread-safe ``_tx_queue``.
- Incoming lines are pushed onto a thread-safe ``_rx_queue``.
- The bridge node reads from ``_rx_queue`` via ``get_line()``.
- The bridge node sends via ``send_command()``.

Thread safety
-------------
``serial.Serial`` write() is NOT thread-safe in general, so all writes
go through ``_tx_queue`` and are executed inside the worker thread.
"""

import queue
import threading
import time
from typing import Optional

import serial

from .constants import SERIAL_READ_TIMEOUT_S


class SerialWorker:
    """Background thread that owns the serial port."""

    def __init__(self, port: str, baudrate: int, logger) -> None:
        self._port     = port
        self._baudrate = baudrate
        self._logger   = logger

        # TX: tuples of (line: str) sent to the ESP32
        self._tx_queue: queue.Queue = queue.Queue()

        # RX: raw lines received from the ESP32
        self._rx_queue: queue.Queue = queue.Queue()

        self._serial: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name='serial_worker', daemon=True
        )

    # ------------------------------------------------------------------
    # Public API (called from the ROS2 node / main thread)
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the serial port and start the background thread."""
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=SERIAL_READ_TIMEOUT_S,
        )
        # Flush any stale data from a previous session
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._logger.info(f'Serial port open: {self._port} @ {self._baudrate}')
        self._thread.start()

    def stop(self) -> None:
        """Signal the worker thread to exit and close the port."""
        self._stop_event.set()
        self._thread.join(timeout=3.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._logger.info('Serial worker stopped.')

    def send_command(self, cmd: str) -> None:
        """
        Queue a command for transmission to the ESP32.
        The trailing newline is added here — do NOT include it in ``cmd``.
        """
        self._tx_queue.put(cmd)

    def get_line(self, timeout: float = 0.0) -> Optional[str]:
        """
        Return the next line received from the ESP32, or None if the
        queue is empty within ``timeout`` seconds.
        """
        try:
            return self._rx_queue.get(timeout=timeout) if timeout > 0 else \
                   self._rx_queue.get_nowait()
        except queue.Empty:
            return None

    # ------------------------------------------------------------------
    # Worker thread
    # ------------------------------------------------------------------

    def _run(self) -> None:
        """Main loop: drain TX queue, read incoming bytes, push to RX queue."""
        buf = b''

        while not self._stop_event.is_set():
            # --- Transmit any queued commands ---
            try:
                while True:
                    cmd = self._tx_queue.get_nowait()
                    line = (cmd.strip() + '\n').encode('ascii', errors='replace')
                    self._serial.write(line)
                    self._logger.debug(f'TX → {cmd.strip()!r}')
            except queue.Empty:
                pass

            # --- Read available bytes ---
            try:
                waiting = self._serial.in_waiting
            except serial.SerialException as exc:
                self._logger.error(f'Serial error: {exc}')
                time.sleep(0.5)
                continue

            if waiting:
                buf += self._serial.read(waiting)
            else:
                # Block briefly (up to SERIAL_READ_TIMEOUT_S) so we're
                # not spinning at 100 % CPU when the ESP32 is silent.
                chunk = self._serial.read(1)
                if chunk:
                    buf += chunk

            # --- Extract complete lines from the buffer ---
            while b'\n' in buf:
                line_bytes, buf = buf.split(b'\n', 1)
                line = line_bytes.rstrip(b'\r').decode('ascii', errors='replace').strip()
                if line:
                    self._logger.debug(f'RX ← {line!r}')
                    self._rx_queue.put(line)
