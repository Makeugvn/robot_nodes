#!/usr/bin/env python3
"""
keyboard_teleop_node.py
=======================
ROS2 node untuk kendali manual robot via keyboard WASD.
Kirim cmd_vel ke ESP32 via UDP port 5007.

Format JSON yang dikirim: {"lx": 0.3, "az": 0.5}
  lx = linear.x  (m/s)  — maju/mundur
  az = angular.z (rad/s) — putar kiri/kanan

Kontrol:
  W — maju
  S — mundur
  A — putar kiri
  D — putar kanan
  WA/WD — maju sambil belok
  SA/SD — mundur sambil belok
  Spasi / tidak ada tombol = stop (kirim lx=0, az=0)
  Q — keluar

Kecepatan bisa diatur via parameter ROS2:
  --ros-args -p linear_speed:=0.4 -p angular_speed:=1.2
"""

import rclpy
import json
import socket
import sys
import termios
import threading
import time
import tty
from rclpy.node import Node
from std_msgs.msg import String


# ══════════════════════════════════════════════════════════════
#  KONFIGURASI TAMPILAN TERMINAL
# ══════════════════════════════════════════════════════════════
BANNER = """
╔══════════════════════════════════════════════════╗
║         KEYBOARD TELEOP — UDP cmd_vel            ║
╠══════════════════════════════════════════════════╣
║  W        : Maju                                 ║
║  S        : Mundur                               ║
║  A        : Putar Kiri                           ║
║  D        : Putar Kanan                          ║
║  WA / WD  : Maju + Belok                         ║
║  SA / SD  : Mundur + Belok                       ║
║  Spasi    : Stop                                 ║
║  Q        : Keluar                               ║
╠══════════════════════════════════════════════════╣
║  + / -    : Naikkan / Turunkan kecepatan linear  ║
║  [ / ]    : Naikkan / Turunkan kecepatan angular ║
╚══════════════════════════════════════════════════╝
"""


# ══════════════════════════════════════════════════════════════
#  NODE
# ══════════════════════════════════════════════════════════════
class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # ── Parameter ──────────────────────────────────────────
        self.declare_parameter('esp32_ip',       '10.146.174.127')
        self.declare_parameter('port_cmdvel',     5007)
        self.declare_parameter('linear_speed',    0.3)    # m/s
        self.declare_parameter('angular_speed',   0.5)    # rad/sx``
        self.declare_parameter('send_rate_hz',    20.0)   # Hz publish cmd_vel
        self.declare_parameter('stop_on_release', True)   # kirim stop saat tombol dilepas

        self.esp32_ip      = self.get_parameter('esp32_ip').value
        self.port_cmdvel   = self.get_parameter('port_cmdvel').value
        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.send_rate     = self.get_parameter('send_rate_hz').value

        # ── State kecepatan aktif ──────────────────────────────
        self._lock    = threading.Lock()
        self._lin     = 0.0
        self._ang     = 0.0
        self._running = True

        # ── Socket UDP ─────────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── Publisher status (opsional, untuk monitoring) ──────
        self.pub_status = self.create_publisher(String, '/teleop_status', 10)

        # ── Thread keyboard ────────────────────────────────────
        self._kb_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        # ── Timer kirim UDP ────────────────────────────────────
        period = 1.0 / self.send_rate
        self.create_timer(period, self._send_cmdvel)

        # ── Timer publish status ───────────────────────────────
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Teleop node siap → ESP32 {self.esp32_ip}:{self.port_cmdvel}')
        self.get_logger().info(
            f'Linear={self.linear_speed:.2f} m/s  '
            f'Angular={self.angular_speed:.2f} rad/s  '
            f'Rate={self.send_rate:.0f} Hz')

    # ══════════════════════════════════════════════════════════
    #  KEYBOARD INPUT — non-blocking single-char read
    # ══════════════════════════════════════════════════════════
    def _get_key(self, fd, old_settings):
        """Baca satu karakter dari stdin (raw mode)."""
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        print(BANNER)
        print(f'Target : {self.esp32_ip}:{self.port_cmdvel}')
        print(f'Linear : {self.linear_speed:.2f} m/s  '
              f'Angular: {self.angular_speed:.2f} rad/s\n')

        try:
            while self._running and rclpy.ok():
                key = self._get_key(fd, old_settings)

                if key is None:
                    continue

                k = key.lower()

                # ── Keluar ─────────────────────────────────────
                if k == 'q':
                    print('\n[Teleop] Keluar...')
                    with self._lock:
                        self._lin = 0.0
                        self._ang = 0.0
                    self._send_now(0.0, 0.0)  # pastikan stop terkirim
                    self._running = False
                    rclpy.shutdown()
                    break

                # ── Sesuaikan kecepatan ────────────────────────
                elif k == '+':
                    self.linear_speed = min(self.linear_speed + 0.05, 1.0)
                    print(f'\r  Linear speed: {self.linear_speed:.2f} m/s    ',
                          end='', flush=True)
                    continue

                elif k == '-':
                    self.linear_speed = max(self.linear_speed - 0.05, 0.05)
                    print(f'\r  Linear speed: {self.linear_speed:.2f} m/s    ',
                          end='', flush=True)
                    continue

                elif k == ']':
                    self.angular_speed = min(self.angular_speed + 0.1, 5.0)
                    print(f'\r  Angular speed: {self.angular_speed:.2f} rad/s  ',
                          end='', flush=True)
                    continue

                elif k == '[':
                    self.angular_speed = max(self.angular_speed - 0.1, 0.1)
                    print(f'\r  Angular speed: {self.angular_speed:.2f} rad/s  ',
                          end='', flush=True)
                    continue

                # ── Mapping WASD ───────────────────────────────
                lin, ang = self._key_to_vel(k)

                with self._lock:
                    self._lin = lin
                    self._ang = ang

                # Tampilkan status
                self._print_status(k, lin, ang)

        except Exception as e:
            self.get_logger().error(f'Keyboard loop error: {e}')
        finally:
            # Restore terminal
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def _key_to_vel(self, key: str):
        """
        Mapping tombol ke (linear, angular).
        Kombinasi W+A, W+D, S+A, S+D ditangani dengan menekan
        dua tombol secara bersamaan — tapi stdin single-char
        hanya bisa baca 1 char. Solusinya: tekan dan tahan
        satu tombol, timer akan terus kirim kecepatan itu.
        Spasi = stop.
        """
        L = self.linear_speed
        A = self.angular_speed

        mapping = {
            'w': ( L,  0.0),   # maju
            's': (-L,  0.0),   # mundur
            'a': (0.0,  A),    # putar kiri (CCW)
            'd': (0.0, -A),    # putar kanan (CW)
            ' ': (0.0,  0.0),  # stop
        }
        return mapping.get(key, (0.0, 0.0))

    def _print_status(self, key, lin, ang):
        action = {
            'w': 'MAJU   ↑',
            's': 'MUNDUR ↓',
            'a': 'KIRI   ←',
            'd': 'KANAN  →',
            ' ': 'STOP   ■',
        }.get(key, f'KEY={key}  ■')
        print(f'\r  {action}  |  lx={lin:+.2f} m/s  az={ang:+.2f} rad/s    ',
              end='', flush=True)

    # ══════════════════════════════════════════════════════════
    #  KIRIM UDP cmd_vel
    # ══════════════════════════════════════════════════════════
    def _send_now(self, lin: float, ang: float):
        """Kirim satu paket UDP langsung."""
        payload = json.dumps({'lx': round(lin, 4), 'az': round(ang, 4)})
        try:
            self.sock.sendto(
                payload.encode(),
                (self.esp32_ip, self.port_cmdvel)
            )
        except Exception as e:
            self.get_logger().warn(f'UDP send error: {e}')

    def _send_cmdvel(self):
        """
        Timer callback 20Hz.
        Kalau tidak ada tombol ditekan (lin=0, ang=0) tetap kirim
        stop supaya ESP32 timeout tidak aktif dulu.
        Kalau _running=False, kirim stop lalu tidak lanjut.
        """
        if not self._running:
            return

        with self._lock:
            lin = self._lin
            ang = self._ang

        self._send_now(lin, ang)

    # ══════════════════════════════════════════════════════════
    #  STATUS PUBLISHER
    # ══════════════════════════════════════════════════════════
    def _publish_status(self):
        with self._lock:
            lin = self._lin
            ang = self._ang
        msg = String()
        msg.data = f'lx={lin:.2f} az={ang:.2f}'
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._running = False
        self._send_now(0.0, 0.0)   # kirim stop satu kali terakhir
        time.sleep(0.1)
        self.sock.close()
        super().destroy_node()


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    try:
        node = KeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()