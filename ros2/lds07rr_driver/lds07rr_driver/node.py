"""ROS 2 driver for the Roborock LDS07RR lidar wired straight to a Raspberry Pi (no ESP32).

Wiring: lidar TX -> a Pi UART RX (Pi 4: dtoverlay=uart3 -> GPIO5, /dev/ttyAMA3), motor via a BC547 on a
hardware-PWM pin (dtoverlay=pwm,pin=18,func=2 -> /sys/class/pwm/pwmchip0/pwm0).

Publishes sensor_msgs/LaserScan once per revolution (counter-clockwise, 1 degree per ray) and
regulates the motor to ~300 rpm with the same integrating controller as the ESP32 firmware,
using the speed field of the lidar's own 0xFA packets.
"""
import math
import os
import threading
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def neato_checksum(p):
    chk = 0
    for i in range(0, 20, 2):
        chk = (chk << 1) + (p[i] | p[i + 1] << 8)
    chk = (chk & 0x7FFF) + (chk >> 15)
    return chk & 0x7FFF


class SysfsPwm:
    """Hardware PWM via /sys/class/pwm (needs write access, see 99-lds07rr.rules)."""

    def __init__(self, chip, channel, freq):
        base = f"/sys/class/pwm/pwmchip{chip}"
        self.dir = f"{base}/pwm{channel}"
        if not os.path.isdir(self.dir):
            with open(f"{base}/export", "w") as f:
                f.write(str(channel))
        for _ in range(100):  # udev needs a moment to set the permissions of a fresh export
            if os.access(f"{self.dir}/period", os.W_OK):
                break
            time.sleep(0.05)
        self.period = int(1e9 / freq)
        try:  # a fresh export has period 0, which rejects any duty_cycle write: set the period first
            self._write("period", str(self.period))
        except OSError:  # an old duty_cycle larger than the new period blocks it: clear that first
            self._write("duty_cycle", "0")
            self._write("period", str(self.period))
        self._write("duty_cycle", "0")
        self._write("enable", "1")

    def _write(self, name, value):
        with open(f"{self.dir}/{name}", "w") as f:
            f.write(value)

    def set(self, fraction):
        self._write("duty_cycle", str(int(self.period * min(max(fraction, 0.0), 1.0))))


class Lds07rrNode(Node):
    def __init__(self):
        super().__init__("lds07rr")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyAMA3").value
        self.baud = p("baud", 115200).value
        self.frame_id = p("frame_id", "laser").value
        self.target_rpm = float(p("target_rpm", 300.0).value)
        self.range_min = float(p("range_min", 0.12).value)
        self.range_max = float(p("range_max", 6.0).value)
        self.clockwise = p("clockwise", True).value      # the LDS07RR measures clockwise
        self.angle_offset = int(p("angle_offset_deg", 0).value)
        pwm_chip = p("pwm_chip", 0).value
        pwm_channel = p("pwm_channel", 0).value
        pwm_freq = p("pwm_freq", 20000).value
        self.max_duty = float(p("max_duty", 0.7).value)   # the motor needs ~35-60 % for 300 rpm on 5 V
        self.give_up_s = float(p("give_up_s", 10.0).value)  # no scans for this long: stop the motor...
        self.retry_s = float(p("retry_s", 30.0).value)      # ...and try again after this pause

        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        # Open the serial port before touching the motor: if it fails, the motor never starts.
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self.pwm = SysfsPwm(pwm_chip, pwm_channel, pwm_freq)
        self.pwm_name = f"pwmchip{pwm_chip}/pwm{pwm_channel}"
        self.start_duty = 350 / 1023                      # soft start, like the firmware
        self.duty = self.start_duty
        self.pwm.set(self.duty)
        try:
            self._setup_rest()
        except Exception:
            self.pwm.set(0)
            raise

    def _setup_rest(self):
        self.rpm_sum, self.rpm_n = 0.0, 0
        self.last_pkt = 0.0
        self.stale_since = time.monotonic()
        self.paused_until = 0.0
        self.last_byte = time.monotonic()
        self.status_pkts = 0
        self.warned = set()
        self.lock = threading.Lock()

        self.running = True
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()
        self.create_timer(0.1, self.control)
        self.create_timer(2.0, self.health)
        self.get_logger().info(f"LDS07RR on {self.port}, motor PWM {self.pwm_name}, max duty {self.max_duty:.0%}, "
                               f"target {self.target_rpm:.0f} rpm")

    # ------------------------------------------------------------------ data
    def reader(self):
        try:
            self._read_loop()
        except Exception as e:  # noqa: BLE001 - during shutdown the port/publisher may already be gone
            if self.running:
                self.get_logger().error(f"reader stopped: {e!r}")

    def _read_loop(self):
        buf = bytearray()
        ranges = [math.inf] * 360
        intens = [0.0] * 360
        last_idx, first = None, True
        rev_start, rev_stamp = time.monotonic(), self.get_clock().now()
        while self.running:
            chunk = self.ser.read(2048)
            if not chunk:
                continue
            self.last_byte = time.monotonic()
            buf += chunk
            self.status_pkts += chunk.count(b"\xaa\x2a\x00")
            while True:
                i = buf.find(0xFA)
                if i < 0:
                    buf.clear()
                    break
                del buf[:i]
                if len(buf) < 22:
                    break
                pkt = bytes(buf[:22])
                if not (0xA0 <= pkt[1] <= 0xF9) or neato_checksum(pkt) != (pkt[20] | pkt[21] << 8):
                    del buf[:1]
                    continue
                del buf[:22]
                idx = pkt[1] - 0xA0
                rpm = (pkt[2] | pkt[3] << 8) / 64
                with self.lock:
                    self.rpm_sum += rpm
                    self.rpm_n += 1
                    self.last_pkt = time.monotonic()
                if last_idx is not None and idx < last_idx:   # new revolution
                    now, stamp = time.monotonic(), self.get_clock().now()
                    if not first and self.running:
                        self.publish(ranges, intens, now - rev_start, rev_stamp)
                    first, rev_start, rev_stamp = False, now, stamp
                    ranges = [math.inf] * 360
                    intens = [0.0] * 360
                last_idx = idx
                for k in range(4):
                    o = 4 + k * 4
                    if pkt[o + 1] & 0x80:                     # invalid sample
                        continue
                    dist = (pkt[o] | (pkt[o + 1] & 0x3F) << 8) / 1000.0
                    if not self.range_min <= dist <= self.range_max:
                        continue
                    a = (idx * 4 + k + self.angle_offset) % 360
                    ray = (360 - a) % 360 if self.clockwise else a
                    ranges[ray] = dist
                    intens[ray] = float(pkt[o + 2] | pkt[o + 3] << 8)

    def publish(self, ranges, intens, scan_time, stamp):
        msg = LaserScan()
        msg.header.stamp = stamp.to_msg()                 # start of the revolution
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_increment = 2 * math.pi / 360
        msg.angle_max = msg.angle_increment * 359
        msg.scan_time = float(scan_time)
        # The head turns clockwise but the rays are stored counter-clockwise, so the measurement time
        # is not linear in the ray index; 0 = "unknown" (no per-ray de-skew) rather than a wrong value.
        msg.time_increment = 0.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intens
        self.pub.publish(msg)

    # ------------------------------------------------------------------ motor
    def control(self):
        now = time.monotonic()
        with self.lock:
            n, s = self.rpm_n, self.rpm_sum
            self.rpm_sum, self.rpm_n = 0.0, 0
            stale = now - self.last_pkt > 0.5
        if now < self.paused_until:          # gave up earlier: motor off until the retry time
            self.duty = 0.0
        elif stale:
            if self.duty == 0.0:             # retry after a pause: soft start again
                self.duty = self.start_duty
                self.stale_since = now
            if now - self.stale_since > self.give_up_s:
                self.duty = 0.0
                self.paused_until = now + self.retry_s
                self.get_logger().warn(f"no scans for {self.give_up_s:.0f} s: motor off, "
                                       f"retrying in {self.retry_s:.0f} s")
            else:
                self.duty += 20 / 1023       # no measurement packets: ramp up gently
        else:
            self.stale_since = now
            if n:
                self.duty += 0.3 * (self.target_rpm - s / n) / 1023
        self.duty = min(max(self.duty, 0.0), self.max_duty)
        self.pwm.set(self.duty)
        self.rpm = s / n if n else 0.0

    def health(self):
        silent = time.monotonic() - self.last_byte > 3
        spinning = time.monotonic() - self.last_pkt < 1
        if silent:
            self._warn("silent", "no data from the lidar: check the wiring and its 5 V supply")
        elif not spinning and self.status_pkts:
            self._warn("status", f"lidar sends status packets but no scans (duty {self.duty:.0%}): "
                                 "is the motor turning?")
        else:
            self.warned.clear()
            self.get_logger().debug(f"{getattr(self, 'rpm', 0):.0f} rpm, duty {self.duty:.0%}")

    def _warn(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        try:
            self.pwm.set(0)
        except OSError:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = Lds07rrNode()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
