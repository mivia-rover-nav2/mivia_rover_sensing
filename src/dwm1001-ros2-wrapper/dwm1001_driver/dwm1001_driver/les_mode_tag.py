#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import re
import threading
import queue
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped

import serial

# Try to import custom stamped Float64
try:
    from dwm1001_msg.msg import Float64Stamped  
    HAS_FLOAT64STAMPED = True
except Exception:
    Float64Stamped = None 
    HAS_FLOAT64STAMPED = False

# Regex for anchor lines matching: ID [x,y,z] = d
ANCHOR_PATTERN = re.compile(
    r'(?P<id>[0-9A-Fa-f]{4})\s*'
    r'\[\s*(?P<x>-?\d+(?:\.\d+)?)\s*,\s*(?P<y>-?\d+(?:\.\d+)?)\s*,\s*(?P<z>-?\d+(?:\.\d+)?)\s*\]'
    r'\s*=\s*(?P<d>-?\d+(?:\.\d+)?)'
)

# Regex for tag estimate line matching: est[x,y,z,q]
EST_PATTERN = re.compile(
    r'est\[\s*'
    r'(?P<x>-?\d+(?:\.\d+)?)\s*,\s*'
    r'(?P<y>-?\d+(?:\.\d+)?)\s*,\s*'
    r'(?P<z>-?\d+(?:\.\d+)?)\s*,\s*'
    r'(?P<q>-?\d+(?:\.\d+)?)\s*'
    r'\]'
)


class DWM1001LesPublisher(Node):
    """
    ROS 2 node that reads DWM1001 in LES mode from serial and publishes:
      - per-anchor range and anchor position
      - tag position and quality

    Robustness feature:
      - Detects serial disconnections / read failures.
      - Logs a warning once per disconnection event.
      - Auto-reconnects with backoff, re-initializes shell + (optionally) sends 'les',
        and resumes normal operation without restarting the node.
    """

    # --- state machine states---
    _STATE_CONNECTED = "CONNECTED"
    _STATE_DISCONNECTED = "DISCONNECTED"
    _STATE_CONNECTING = "CONNECTING"

    def __init__(self) -> None:
        super().__init__('dwm1001_les_mode', allow_undeclared_parameters=True)
        self._declare_parameters()

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.send_les_on_start = bool(self.get_parameter('send_les_on_start').value)

        # Reconnect tuning
        self.reconnect_enabled = bool(self.get_parameter('reconnect_enabled').value)
        self.reconnect_initial_delay_s = float(self.get_parameter('reconnect_initial_delay_s').value)
        self.reconnect_max_delay_s = float(self.get_parameter('reconnect_max_delay_s').value)
        self.reconnect_backoff = float(self.get_parameter('reconnect_backoff').value)
        self.reconnect_timer_period_s = float(self.get_parameter('reconnect_timer_period_s').value)

        if not self.serial_port:
            self.get_logger().fatal("Parameter 'serial_port' is empty. Aborting.")
            raise SystemExit(1)

        # Serial handle + lock
        self._ser_lock = threading.Lock()
        self.ser: Optional[serial.Serial] = None

        # Connection state and backoff bookkeeping
        self._state = self._STATE_DISCONNECTED
        self._last_disconnect_log_t = 0.0
        self._next_reconnect_attempt_t = 0.0
        self._current_reconnect_delay_s = self.reconnect_initial_delay_s

        # Thread-safe queue for lines read from serial
        self.line_queue: "queue.Queue[str]" = queue.Queue(maxsize=500)

        # Dynamic map for anchor publishers
        self._anchor_pubs: Dict[str, Dict[str, object]] = {}

        # Tag publishers
        self.tag_pos_pub = self.create_publisher(PointStamped, '~/tag/position', 10)
        if HAS_FLOAT64STAMPED:
            self.tag_quality_pub = self.create_publisher(Float64Stamped, '~/tag/quality', 10)  # type: ignore
            self.get_logger().info("Publishing tag quality on ~/tag/quality as Float64Stamped.")
        else:
            self.tag_quality_pub = self.create_publisher(Float64, '~/tag/quality', 10)
            self.get_logger().warn(
                "Float64Stamped not available (dwm1001_msg/msg/Float64Stamped). "
                "Publishing tag quality on ~/tag/quality as std_msgs/Float64 (no header)."
            )

        # Start serial once at boot (best effort)
        ok = self._connect_and_init()
        if not ok:
            self.get_logger().warn(
                "Initial serial connection failed. "
                "Auto-reconnect is %s.", "ENABLED" if self.reconnect_enabled else "DISABLED"
            )

        # Reader thread (kept alive even across reconnects)
        self._running = True
        self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.reader_thread.start()

        # Timer for line processing callback
        self.timer = self.create_timer(0.01, self._process_lines)  # 100 Hz

        # Timer for reconnect attempts (lightweight watchdog)
        self.reconnect_timer = self.create_timer(self.reconnect_timer_period_s, self._reconnect_watchdog)

    # ----------------- Parameters -----------------
    def _declare_parameters(self):
        serial_port_desc = ParameterDescriptor(
            description="Device/COM port for DWM1001 (e.g., /dev/ttyACM0 or /dev/serial/by-id/...)",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        baudrate_desc = ParameterDescriptor(
            description="Baudrate for serial port.",
            type=ParameterType.PARAMETER_INTEGER,
            read_only=True,
        )
        frame_id_desc = ParameterDescriptor(
            description="Frame id used for published PointStamped messages.",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        send_les_desc = ParameterDescriptor(
            description="Send 'les' command on startup to enable ranging stream (after reset+shell).",
            type=ParameterType.PARAMETER_BOOL,
            read_only=True,
        )

        reconnect_enabled_desc = ParameterDescriptor(
            description="Enable auto-reconnect on serial failures/disconnections.",
            type=ParameterType.PARAMETER_BOOL,
            read_only=True,
        )
        reconnect_initial_delay_desc = ParameterDescriptor(
            description="Initial reconnect delay (seconds).",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=True,
        )
        reconnect_max_delay_desc = ParameterDescriptor(
            description="Maximum reconnect delay (seconds).",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=True,
        )
        reconnect_backoff_desc = ParameterDescriptor(
            description="Backoff multiplier applied after each failed reconnect attempt.",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=True,
        )
        reconnect_timer_period_desc = ParameterDescriptor(
            description="Watchdog timer period for reconnect attempts (seconds).",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=True,
        )

        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-SEGGER_J-Link_000760176108-if00', serial_port_desc)
        self.declare_parameter('baudrate', 115200, baudrate_desc)
        self.declare_parameter('frame_id', 'dwm1001', frame_id_desc)
        self.declare_parameter('send_les_on_start', True, send_les_desc)

        # Reconnect parameters
        self.declare_parameter('reconnect_enabled', True, reconnect_enabled_desc)
        self.declare_parameter('reconnect_initial_delay_s', 0.5, reconnect_initial_delay_desc)
        self.declare_parameter('reconnect_max_delay_s', 10.0, reconnect_max_delay_desc)
        self.declare_parameter('reconnect_backoff', 1.6, reconnect_backoff_desc)
        self.declare_parameter('reconnect_timer_period_s', 0.2, reconnect_timer_period_desc)

    # ----------------- Serial connection / init -----------------
    def _connect_and_init(self) -> bool:
        """
        Best-effort: open serial, set lines, reset+shell, optionally start LES.
        Returns True if connected and initialized, False otherwise.
        """
        with self._ser_lock:
            if self._state == self._STATE_CONNECTING:
                return False

            self._state = self._STATE_CONNECTING

        try:
            ser = serial.Serial(
                self.serial_port,
                baudrate=self.baudrate,
                timeout=0.2,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )

            # Force known DTR/RTS state
            try:
                ser.setDTR(True)
                ser.setRTS(False)
            except Exception:
                pass

            # Swap in the new handle atomically
            with self._ser_lock:
                # Close previous, if any
                old = self.ser
                self.ser = ser
                self._state = self._STATE_CONNECTED
                self._current_reconnect_delay_s = self.reconnect_initial_delay_s
                self._next_reconnect_attempt_t = 0.0

            if old is not None:
                try:
                    old.close()
                except Exception:
                    pass

            self.get_logger().info(f"Opened serial port '{self.serial_port}' @ {self.baudrate} baud.")

            # Initialize DWM (shell + LES)
            self._reset_and_enter_shell()
            if self.send_les_on_start:
                self._start_les_stream()
            else:
                self.get_logger().warn(
                    "send_les_on_start=False: non invio 'les'. "
                    "Ensure LES mode is active on the DWM1001 tag."
                )

            # Flush queue to avoid parsing junk from previous session
            self._flush_line_queue()

            self.get_logger().info("Serial connected and initialized. Node is operational.")
            return True

        except serial.SerialException as e:
            with self._ser_lock:
                self._state = self._STATE_DISCONNECTED
            self._log_disconnect_once(f"Cannot open serial port '{self.serial_port}': {e}")
            return False
        except Exception as e:
            with self._ser_lock:
                self._state = self._STATE_DISCONNECTED
            self._log_disconnect_once(f"Serial init error: {e}")
            return False

    def _mark_disconnected(self, reason: str) -> None:
        """
        Mark state as DISCONNECTED, close serial handle, schedule next reconnect attempt.
        """
        with self._ser_lock:
            if self._state == self._STATE_DISCONNECTED:
                # Already handled
                return

            self._state = self._STATE_DISCONNECTED
            ser = self.ser
            self.ser = None

            now = time.time()
            # schedule next attempt (immediate if not set)
            if self._next_reconnect_attempt_t <= 0.0:
                self._next_reconnect_attempt_t = now + self._current_reconnect_delay_s

        # Close outside lock
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        self._log_disconnect_once(reason)

    def _log_disconnect_once(self, msg: str) -> None:
        """
        Avoid log spam: warn at most once every ~2 seconds during repeated failures.
        """
        now = time.time()
        if now - self._last_disconnect_log_t > 2.0:
            self.get_logger().warn(f"[SERIAL DISCONNECTED] {msg}")
            self._last_disconnect_log_t = now

    def _flush_line_queue(self) -> None:
        try:
            while True:
                _ = self.line_queue.get_nowait()
        except queue.Empty:
            pass

    # ----------------- Shell control -----------------
    def _send_shell_command(self, cmd: bytes, delay_s: float = 0.5) -> None:
        with self._ser_lock:
            ser = self.ser
            state = self._state

        if state != self._STATE_CONNECTED or ser is None:
            return

        try:
            ser.write(cmd)
            ser.write(b"\r")
            ser.flush()
            time.sleep(delay_s)
        except Exception as e:
            self._mark_disconnected(f"Failed to send shell command {cmd!r}: {e}")

    def _reset_and_enter_shell(self) -> None:
        self.get_logger().info("Resetting DWM1001...")
        self._send_shell_command(b"reset", delay_s=0.2)
        time.sleep(1.0)

        self.get_logger().info("Entering shell mode...")
        with self._ser_lock:
            ser = self.ser
            state = self._state

        if state != self._STATE_CONNECTED or ser is None:
            return

        try:
            ser.write(b"\r\r")
            ser.flush()
            time.sleep(1.0)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as e:
            self._mark_disconnected(f"Failed to enter shell mode: {e}")

    def _start_les_stream(self) -> None:
        self.get_logger().info("Starting LES stream (les)...")
        self._send_shell_command(b"les", delay_s=0.2)

        with self._ser_lock:
            ser = self.ser
            state = self._state

        if state != self._STATE_CONNECTED or ser is None:
            return

        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        time.sleep(0.1)
        try:
            peek = ser.read(ser.in_waiting or 1)
            if peek:
                self.get_logger().info(f"LES: received initial {len(peek)} bytes.")
        except Exception as e:
            self._mark_disconnected(f"LES start/peek failed: {e}")

    # ----------------- Reconnect watchdog -----------------
    def _reconnect_watchdog(self) -> None:
        if not self.reconnect_enabled:
            return

        with self._ser_lock:
            state = self._state
            next_t = self._next_reconnect_attempt_t
            delay = self._current_reconnect_delay_s

        if state != self._STATE_DISCONNECTED:
            return

        now = time.time()
        if next_t > 0.0 and now < next_t:
            return

        ok = self._connect_and_init()
        if ok:
            return

        # Failed attempt: schedule next with backoff
        with self._ser_lock:
            self._current_reconnect_delay_s = min(
                self.reconnect_max_delay_s,
                max(self.reconnect_initial_delay_s, delay) * self.reconnect_backoff
            )
            self._next_reconnect_attempt_t = time.time() + self._current_reconnect_delay_s

        self.get_logger().warn(
            f"Reconnect failed. Next attempt in {self._current_reconnect_delay_s:.2f} s."
        )

    # ----------------- Serial reading -----------------
    def _serial_reader_loop(self):
        self.get_logger().info("Starting serial reader thread (readline).")
        while self._running:
            with self._ser_lock:
                ser = self.ser
                state = self._state

            if state != self._STATE_CONNECTED or ser is None:
                time.sleep(0.05)
                continue

            try:
                raw = ser.readline()  # until \n or timeout
                if not raw:
                    continue

                decoded = raw.decode('utf-8', errors='ignore').strip()
                if not decoded:
                    continue

                try:
                    self.line_queue.put_nowait(decoded)
                except queue.Full:
                    _ = self.line_queue.get_nowait()
                    self.line_queue.put_nowait(decoded)

            except (serial.SerialException, OSError) as e:
                self._mark_disconnected(f"Serial read error: {e}")
                time.sleep(0.05)
            except Exception as e:
                # unexpected: treat as disconnection to be safe
                self._mark_disconnected(f"Unexpected serial read error: {e}")
                time.sleep(0.05)

    # ----------------- Parsing + Publish -----------------
    def _process_lines(self):
        # If disconnected, skip parsing work (queue may be stale, but we flush on reconnect)
        with self._ser_lock:
            if self._state != self._STATE_CONNECTED:
                return

        processed = 0
        while True:
            try:
                line = self.line_queue.get_nowait()
            except queue.Empty:
                break

            processed += 1

            line = line.lstrip()
            if line.startswith('#'):
                line = line.lstrip('#').strip()

            now_stamp = self.get_clock().now().to_msg()

            # 1) Anchor blocks
            matches = list(ANCHOR_PATTERN.finditer(line))
            for m in matches:
                anchor_id = m.group('id').upper()
                x = float(m.group('x'))
                y = float(m.group('y'))
                z = float(m.group('z'))
                dist = float(m.group('d'))

                pubs = self._get_or_create_anchor_pubs(anchor_id)

                if HAS_FLOAT64STAMPED:
                    r_msg = Float64Stamped()  
                    r_msg.header.stamp = now_stamp
                    r_msg.header.frame_id = self.frame_id
                    if hasattr(r_msg, "data"):
                        r_msg.data = dist  
                    else:
                        r_msg.value = dist 
                else:
                    r_msg = Float64()
                    r_msg.data = dist
                pubs['range_pub'].publish(r_msg)

                p_msg = PointStamped()
                p_msg.header.stamp = now_stamp
                p_msg.header.frame_id = self.frame_id
                p_msg.point.x = x
                p_msg.point.y = y
                p_msg.point.z = z
                pubs['pos_pub'].publish(p_msg)

            # 2) Tag estimate
            est = EST_PATTERN.search(line)
            if est:
                tx = float(est.group('x'))
                ty = float(est.group('y'))
                tz = float(est.group('z'))
                q = float(est.group('q'))

                t_msg = PointStamped()
                t_msg.header.stamp = now_stamp
                t_msg.header.frame_id = self.frame_id
                t_msg.point.x = tx
                t_msg.point.y = ty
                t_msg.point.z = tz
                self.tag_pos_pub.publish(t_msg)

                if HAS_FLOAT64STAMPED:
                    q_msg = Float64Stamped()  
                    q_msg.header.stamp = now_stamp
                    q_msg.header.frame_id = self.frame_id
                    if hasattr(q_msg, "data"):
                        q_msg.data = q  
                    else:
                        q_msg.value = q  
                    self.tag_quality_pub.publish(q_msg)  
                else:
                    q_msg = Float64()
                    q_msg.data = q
                    self.tag_quality_pub.publish(q_msg)

        if processed:
            self.get_logger().debug(f"Processed {processed} serial line(s).")

    def _get_or_create_anchor_pubs(self, anchor_id: str) -> Dict[str, object]:
        if anchor_id in self._anchor_pubs:
            return self._anchor_pubs[anchor_id]

        range_topic = f'~/anchors/anchor_{anchor_id}/range'
        pos_topic = f'~/anchors/anchor_{anchor_id}/position'

        if HAS_FLOAT64STAMPED:
            range_pub = self.create_publisher(Float64Stamped, range_topic, 10)  
        else:
            range_pub = self.create_publisher(Float64, range_topic, 10)

        pos_pub = self.create_publisher(PointStamped, pos_topic, 10)

        self.get_logger().info(
            f"Created publishers for anchor {anchor_id}: {range_topic}, {pos_topic}"
        )

        self._anchor_pubs[anchor_id] = {'range_pub': range_pub, 'pos_pub': pos_pub}
        return self._anchor_pubs[anchor_id]

    # ----------------- Shutdown -----------------
    def destroy_node(self):
        self._running = False
        try:
            if hasattr(self, "reader_thread") and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass

        with self._ser_lock:
            ser = self.ser
            self.ser = None
            self._state = self._STATE_DISCONNECTED

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DWM1001LesPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
