# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import time

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf_transformations import quaternion_from_euler

# NMEA GGA fix quality -> human-readable label, purely for diagnostics/logging.
GGA_FIX_QUALITY_LABELS = {
    -1: 'unknown',
    0: 'invalid',
    1: 'gps_fix (sps)',
    2: 'dgps_fix',
    3: 'pps_fix',
    4: 'rtk_fixed',
    5: 'rtk_float',
    6: 'dead_reckoning',
    7: 'manual_input',
    8: 'simulation',
    9: 'waas',
}


class NavsatDriver(Node):
    def __init__(self):
        super().__init__('leap_navsat_driver')

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)
        # Raw quality signals (fix quality, sat count, DOP, GST std devs, fix-age),
        # published every epoch for visibility into the accept/reject gate below,
        # whether or not that epoch actually made it onto /fix.
        self.diag_pub = self.create_publisher(DiagnosticArray, 'gps_diagnostics', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', False).value
        self.frame_id = self.declare_parameter('frame_id', 'reach').value
        self.valid_fix = False

        # ------------------------------------------------------------------
        # Quality gate. This driver is the single point of "is this fix good
        # enough to hand to the rest of the stack" -- downstream nodes (e.g.
        # gps_odom_node.py) transform whatever we publish without re-checking
        # fix status, DOP, or error estimates themselves. An epoch that fails
        # any of these checks is not published on /fix at all.
        # ------------------------------------------------------------------
        self.max_pdop = self.declare_parameter('max_pdop', 5.0).value
        self.max_hdop = self.declare_parameter('max_hdop', 4.0).value
        # If True (default), a fix is rejected when PDOP/HDOP haven't been
        # seen yet (e.g. no GSA received this epoch) rather than being passed
        # through unchecked. Fail-closed, since this is a filtering node.
        self.require_dop_for_fix = self.declare_parameter('require_dop_for_fix', True).value
        self._warned_gate_reject = False

        # ------------------------------------------------------------------
        # Per-epoch scratch state. GSA sentences (one per constellation) all
        # arrive between one GGA/RMC epoch and the next, so we accumulate them
        # and flush into a combined picture on the next GGA.
        # ------------------------------------------------------------------
        self._gsa_satellites_used = 0
        self._gsa_pdop = float('nan')
        self._gsa_hdop = float('nan')
        self._gsa_vdop = float('nan')

        self._last_gst = None  # dict: rms, semi_major, semi_minor, orientation, std_lat, std_lon, std_alt

        self._utc_date = None  # (day, month, year) from ZDA, if seen

        # Fix-quality stability tracking: lets a downstream node distinguish a
        # fix that just reacquired (higher cycle-slip risk) from one that has
        # held for a while.
        self._last_fix_quality = None
        self._fix_quality_since_wall_time = None

        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    # ----------------------------------------------------------------------
    # Public entry point
    # ----------------------------------------------------------------------
    def add_sentence(self, nmea_string):
        """Parse one NMEA sentence and publish whatever it produces.

        Returns True if the sentence was well-formed (valid checksum, known
        talker), False otherwise. This does not raise on malformed input --
        the driver is expected to run continuously against a live serial
        stream where the occasional truncated/garbled line is normal.
        """
        nmea_string = nmea_string.strip()
        if not nmea_string:
            return False

        if not self._checksum_is_valid(nmea_string):
            self.get_logger().warn(
                'Rejected NMEA sentence with bad/missing checksum: {0}'.format(nmea_string))
            return False

        try:
            body = nmea_string.lstrip('$').split('*', 1)[0]
            fields = body.split(',')
            sentence_type = fields[0][2:]  # e.g. 'GNGGA' -> 'GGA', 'GPGSA' -> 'GSA'
        except Exception as e:
            self.get_logger().warn('Could not parse NMEA sentence {0}: {1}'.format(nmea_string, e))
            return False

        handler = {
            'GGA': self._handle_gga,
            'RMC': self._handle_rmc,
            'GSA': self._handle_gsa,
            'GST': self._handle_gst,
            'VTG': self._handle_vtg,
            'ZDA': self._handle_zda,
        }.get(sentence_type)

        if handler is None:
            return True  # Recognized-but-unhandled (GSV, EBP, ...) is not an error.

        try:
            handler(fields)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(
                'Failed to parse {0} sentence fields {1}: {2}'.format(sentence_type, fields, e))
            return False

        return True

    # ----------------------------------------------------------------------
    # Quality gate
    # ----------------------------------------------------------------------
    def _fix_passes_quality_gate(self, nav_status):
        """Decide whether this epoch is good enough to publish on /fix.

        This is the only place accept/reject happens. Everything downstream
        (gps_odom_node.py included) trusts that anything arriving on /fix has
        already cleared this gate and does no further filtering of its own.
        Returns (passed, reason_if_rejected).
        """
        if nav_status == NavSatStatus.STATUS_NO_FIX:
            return False, 'no fix (invalid/unknown GGA quality)'

        pdop = self._gsa_pdop
        hdop = self._gsa_hdop

        if math.isnan(pdop) or math.isnan(hdop):
            if self.require_dop_for_fix:
                return False, 'PDOP/HDOP not yet available from GSA'
        else:
            if pdop > self.max_pdop:
                return False, f'PDOP {pdop:.2f} exceeds max_pdop {self.max_pdop:.2f}'
            if hdop > self.max_hdop:
                return False, f'HDOP {hdop:.2f} exceeds max_hdop {self.max_hdop:.2f}'

        return True, None

    # ----------------------------------------------------------------------
    # Sentence handlers
    # ----------------------------------------------------------------------
    def _handle_gga(self, f):
        # $GNGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,M,geoidSep,M,diffAge,diffStation*CS
        fix_quality = int(f[6]) if f[6] else -1
        num_satellites = int(f[7]) if f[7] else 0
        hdop = float(f[8]) if f[8] else float('nan')
        altitude = float(f[9]) if f[9] else float('nan')
        latitude = self._dm_to_dd(f[2], f[3])
        longitude = self._dm_to_dd(f[4], f[5])

        self._track_fix_quality_stability(fix_quality)

        default_epe, nav_status, default_cov_type = self.gps_qualities.get(
            fix_quality, self.gps_qualities[-1])

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = self.frame_id
        fix_msg.status.status = nav_status
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = latitude
        fix_msg.longitude = longitude
        fix_msg.altitude = altitude

        # Prefer the receiver's own GST error estimate over the static epe table,
        # but only when this epoch actually has a fix -- otherwise a stale GST
        # from an earlier good epoch would get reused and make a NO_FIX epoch
        # look like it has known-good covariance.
        if nav_status == NavSatStatus.STATUS_NO_FIX:
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        elif self._last_gst is not None and not math.isnan(self._last_gst['std_lat']):
            std_lon = self._last_gst['std_lon']
            std_lat = self._last_gst['std_lat']
            std_alt = self._last_gst['std_alt']
            fix_msg.position_covariance[0] = std_lon ** 2
            fix_msg.position_covariance[4] = std_lat ** 2
            fix_msg.position_covariance[8] = std_alt ** 2
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            err = hdop * default_epe if not math.isnan(hdop) else default_epe
            fix_msg.position_covariance[0] = err ** 2
            fix_msg.position_covariance[4] = err ** 2
            fix_msg.position_covariance[8] = (2 * err) ** 2
            fix_msg.position_covariance_type = default_cov_type

        self._publish_diagnostics(fix_quality, num_satellites, hdop, fix_msg.header.stamp)

        passed, reject_reason = self._fix_passes_quality_gate(nav_status)
        self.valid_fix = passed

        if passed:
            self._warned_gate_reject = False
            self.fix_pub.publish(fix_msg)
        elif not self._warned_gate_reject:
            self.get_logger().warn(
                f'Rejecting GPS epoch, not publishing /fix: {reject_reason} '
                '(further rejections logged only on change until a good fix returns)'
            )
            self._warned_gate_reject = True

        # GSA sentences for this epoch have already been consumed by the time
        # the next GGA arrives; reset the accumulator for the new epoch.
        self._gsa_satellites_used = 0

    def _handle_rmc(self, f):
        # $GNRMC,time,status(A/V),lat,NS,lon,EW,speed_knots,track,date,magvar,magvarEW,mode*CS
        status = f[2]
        if status != 'A':
            return  # Receiver flags this fix as invalid; nothing reliable to publish.

        speed_knots = float(f[7]) if f[7] else 0.0
        track_true_deg = float(f[8]) if f[8] else float('nan')

        if not math.isnan(track_true_deg):
            self._publish_heading(track_true_deg)

        speed_mps = speed_knots * 0.514444
        self._publish_velocity(speed_mps, track_true_deg)

    def _handle_gsa(self, f):
        # $__GSA,mode1,mode2,sv1..sv12,PDOP,HDOP,VDOP*CS
        sv_fields = f[3:15]
        satellites_this_talker = sum(1 for sv in sv_fields if sv)
        self._gsa_satellites_used += satellites_this_talker

        try:
            self._gsa_pdop = float(f[15]) if f[15] else self._gsa_pdop
            self._gsa_hdop = float(f[16]) if f[16] else self._gsa_hdop
            self._gsa_vdop = float(f[17]) if f[17] else self._gsa_vdop
        except IndexError:
            pass

    def _handle_gst(self, f):
        # $GNGST,time,rms,semiMajor,semiMinor,orientation,stdLat,stdLon,stdAlt*CS
        def _fnan(s):
            return float(s) if s else float('nan')

        self._last_gst = {
            'rms': _fnan(f[2]),
            'semi_major': _fnan(f[3]),
            'semi_minor': _fnan(f[4]),
            'orientation': _fnan(f[5]),
            'std_lat': _fnan(f[6]),
            'std_lon': _fnan(f[7]),
            'std_alt': _fnan(f[8]),
        }
        self.lat_std_dev = self._last_gst['std_lat']
        self.lon_std_dev = self._last_gst['std_lon']
        self.alt_std_dev = self._last_gst['std_alt']
        self.using_receiver_epe = True

    def _handle_vtg(self, f):
        # $GNVTG,trackTrue,T,trackMag,M,speedKnots,N,speedKmh,K,mode*CS
        track_true_deg = float(f[1]) if f[1] else float('nan')
        speed_knots = float(f[5]) if f[5] else 0.0
        speed_mps = speed_knots * 0.514444
        self._publish_velocity(speed_mps, track_true_deg)

    def _handle_zda(self, f):
        # $GNZDA,time,day,month,year,localZoneHours,localZoneMinutes*CS
        if f[2] and f[3] and f[4]:
            self._utc_date = (int(f[2]), int(f[3]), int(f[4]))

        time_ref = TimeReference()
        time_ref.header.stamp = self.get_clock().now().to_msg()
        time_ref.source = self.time_ref_source
        self.time_ref_pub.publish(time_ref)

    # ----------------------------------------------------------------------
    # Publish helpers
    # ----------------------------------------------------------------------
    def _publish_velocity(self, speed_mps, track_true_deg):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        if not math.isnan(track_true_deg):
            heading_rad = math.radians(track_true_deg)
            twist.twist.linear.x = speed_mps * math.cos(heading_rad)
            twist.twist.linear.y = speed_mps * math.sin(heading_rad)
        else:
            twist.twist.linear.x = speed_mps
        self.vel_pub.publish(twist)

    def _publish_heading(self, track_true_deg):
        heading_rad = math.radians(track_true_deg)
        q = quaternion_from_euler(0.0, 0.0, heading_rad)
        heading_msg = QuaternionStamped()
        heading_msg.header.stamp = self.get_clock().now().to_msg()
        heading_msg.header.frame_id = 'base_link'
        heading_msg.quaternion.x = q[0]
        heading_msg.quaternion.y = q[1]
        heading_msg.quaternion.z = q[2]
        heading_msg.quaternion.w = q[3]
        self.heading_pub.publish(heading_msg)

    def _publish_diagnostics(self, fix_quality, num_satellites_gga, hdop_gga, stamp):
        """Publish the raw signals behind the accept/reject decision.

        These are published every epoch, whether or not the fix passed the
        quality gate in _fix_passes_quality_gate, so you can see *why* /fix
        went quiet (fix_quality, satellite count, DOP, GST std devs, and how
        long the current fix_quality has held).
        """
        status = DiagnosticStatus()
        status.name = 'gps_quality'
        status.hardware_id = 'gps'
        status.level = DiagnosticStatus.OK if fix_quality in (4,) else DiagnosticStatus.WARN
        status.message = GGA_FIX_QUALITY_LABELS.get(fix_quality, 'unknown')

        fix_age = 0.0
        if self._fix_quality_since_wall_time is not None:
            fix_age = time.monotonic() - self._fix_quality_since_wall_time

        # Prefer GSA's satellite-used count (summed across constellations) when
        # available; GGA's numSV should agree and is used as a fallback/cross-check.
        satellites_used = self._gsa_satellites_used if self._gsa_satellites_used else num_satellites_gga

        values = [
            ('fix_quality', str(fix_quality)),
            ('satellites_used', str(satellites_used)),
            ('satellites_used_gga', str(num_satellites_gga)),
            ('hdop_gga', '{0:.2f}'.format(hdop_gga) if not math.isnan(hdop_gga) else 'nan'),
            ('pdop', '{0:.2f}'.format(self._gsa_pdop)),
            ('hdop', '{0:.2f}'.format(self._gsa_hdop)),
            ('vdop', '{0:.2f}'.format(self._gsa_vdop)),
            ('fix_quality_duration_sec', '{0:.2f}'.format(fix_age)),
        ]
        if self._last_gst is not None:
            values.extend([
                ('gst_rms', '{0:.3f}'.format(self._last_gst['rms'])),
                ('gst_std_lat', '{0:.3f}'.format(self._last_gst['std_lat'])),
                ('gst_std_lon', '{0:.3f}'.format(self._last_gst['std_lon'])),
                ('gst_std_alt', '{0:.3f}'.format(self._last_gst['std_alt'])),
            ])

        status.values = [KeyValue(key=k, value=v) for k, v in values]

        diag_array = DiagnosticArray()
        diag_array.header.stamp = stamp
        diag_array.status = [status]
        self.diag_pub.publish(diag_array)

    def _track_fix_quality_stability(self, fix_quality):
        if fix_quality != self._last_fix_quality:
            self._last_fix_quality = fix_quality
            self._fix_quality_since_wall_time = time.monotonic()

    # ----------------------------------------------------------------------
    # Small parsing utilities
    # ----------------------------------------------------------------------
    @staticmethod
    def _checksum_is_valid(nmea_string):
        if '*' not in nmea_string or not nmea_string.startswith('$'):
            return False
        data, _, checksum_str = nmea_string.rpartition('*')
        data = data[1:]  # drop leading '$'
        checksum = 0
        for c in data:
            checksum ^= ord(c)
        try:
            return checksum == int(checksum_str, 16)
        except ValueError:
            return False

    @staticmethod
    def _dm_to_dd(dm_str, hemisphere):
        """Convert NMEA ddmm.mmmm / dddmm.mmmm to signed decimal degrees."""
        if not dm_str:
            return float('nan')
        value = float(dm_str)
        degrees = int(value / 100)
        minutes = value - degrees * 100
        decimal_degrees = degrees + minutes / 60.0
        if hemisphere in ('S', 'W'):
            decimal_degrees = -decimal_degrees
        return decimal_degrees