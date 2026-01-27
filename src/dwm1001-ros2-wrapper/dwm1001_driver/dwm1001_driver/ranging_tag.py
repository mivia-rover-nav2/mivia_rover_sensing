#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import re
import threading
import queue
from typing import Dict

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped

import serial

ANCHOR_PATTERN = re.compile(
    r'(?P<id>[0-9A-Fa-f]{4})\s*'
    r'\[\s*(?P<x>-?\d+(?:\.\d+)?)\s*,\s*(?P<y>-?\d+(?:\.\d+)?)\s*,\s*(?P<z>-?\d+(?:\.\d+)?)\s*\]'
    r'\s*=\s*(?P<d>-?\d+(?:\.\d+)?)'
)

class DWMLesPublisher(Node):
    def __init__(self) -> None:
        super().__init__('dwm_les_publisher', allow_undeclared_parameters=True)
        self._declare_parameters()

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.send_les_on_start = bool(self.get_parameter('send_les_on_start').value)

        if not self.serial_port:
            self.get_logger().fatal("Parameter 'serial_port' is empty. Aborting.")
            raise SystemExit(1)

        # Open serial port
        try:
            self.ser = serial.Serial(self.serial_port, baudrate=self.baudrate, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().fatal(f"Cannot open serial port '{self.serial_port}': {e}")
            raise SystemExit(1)

        self.get_logger().info(f"Opened serial port '{self.serial_port}' @ {self.baudrate} baud.")

       # Put the tag in LES mode
        if self.send_les_on_start:
            try:
                # Alcune shell accettano '\r\n'; altre bastano '\n'
                self.ser.write(b'les\r\n')
                self.ser.flush()
                self.get_logger().info("Sent 'les' command to enable ranging + estimated position output.")
            except Exception as e:
                self.get_logger().warn(f"Failed to send 'les' command: {e}")

        # Thread-safe queue for serial lines
        self.line_queue: "queue.Queue[str]" = queue.Queue(maxsize=100)

        # Dynamic map of anchor_id -> publishers
        self._anchor_pubs: Dict[str, Dict[str, object]] = {}

        # Thread for serial reading data
        self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._running = True
        self.reader_thread.start()

        # Timer to process lines, frequency is at 100 Hz, is different from reading frequency
        self.timer = self.create_timer(0.01, self._process_lines)  # 100 Hz

    # ----------------- Parameters -----------------
    def _declare_parameters(self):
        serial_port_desc = ParameterDescriptor(
            description="Device/COM port for DWM1001 (e.g., /dev/ttyACM0, COM3)",
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
            description="Send 'les' command on startup to (toggle) enable ranging stream.",
            type=ParameterType.PARAMETER_BOOL,
            read_only=True,
        )

        self.declare_parameter('serial_port', '/dev/ttyACM0', serial_port_desc)
        self.declare_parameter('baudrate', 115200, baudrate_desc)
        self.declare_parameter('frame_id', 'dwm1001', frame_id_desc)
        self.declare_parameter('send_les_on_start', True, send_les_desc)

    # ----------------- Serial Reading -----------------
    def _serial_reader_loop(self):
        """Legge linee dalla seriale e le mette in coda."""
        self.get_logger().info("Starting serial reader thread.")
        buffer = b''
        while self._running:
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buffer += chunk
                
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        decoded = line.decode('utf-8', errors='ignore').strip()
                        if decoded:
                            try:
                                self.line_queue.put_nowait(decoded)
                            except queue.Full:
                                _ = self.line_queue.get_nowait()
                                self.line_queue.put_nowait(decoded)
                    except Exception:
                        pass
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    # ----------------- Parsing + Publish -----------------
    def _process_lines(self):
        """Processa tutte le linee disponibili e pubblica le misure per ancora."""
        processed = 0
        while True:
            try:
                line = self.line_queue.get_nowait()
            except queue.Empty:
                break

            processed += 1
    
            matches = list(ANCHOR_PATTERN.finditer(line))
            if not matches:
                continue

            now_stamp = self.get_clock().now().to_msg()

            for m in matches:
                anchor_id = m.group('id').upper()
                x = float(m.group('x'))
                y = float(m.group('y'))
                z = float(m.group('z'))
                dist = float(m.group('d'))

                pubs = self._get_or_create__anchor_pubs(anchor_id)

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

        if processed:
            self.get_logger().debug(f"Processed {processed} serial line(s).")

    def _get_or_create__anchor_pubs(self, anchor_id: str) -> Dict[str, object]:
        """Crea i publisher per l'ancora se non esistono, altrimenti li riusa."""
        if anchor_id in self._anchor_pubs:
            return self._anchor_pubs[anchor_id]

        range_topic = f'~/anchors/anchor_{anchor_id}/range'
        pos_topic = f'~/anchors/anchor_{anchor_id}/position'

        range_pub = self.create_publisher(Float64, range_topic, 10)
        pos_pub = self.create_publisher(PointStamped, pos_topic, 10)

        self.get_logger().info(
            f"Created _anchor_pubs for anchor {anchor_id}: "
            f"{range_topic} (Float64), {pos_topic} (PointStamped)"
        )

        self._anchor_pubs[anchor_id] = {'range_pub': range_pub, 'pos_pub': pos_pub}
        return self._anchor_pubs[anchor_id]

    # ----------------- Shutdown -----------------
    def destroy_node(self):
        self._running = False
        try:
            if self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DWMLesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
