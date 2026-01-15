# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

from math import degrees, sqrt

from cras_bag_tools import DeserializedMessageFilter
from cras_bag_tools.message_filter import tags_for_generated_msg
from gps_common.msg import GPSFix, GPSStatus
import message_filters
import rospy
from sensor_msgs.msg import NavSatFix


class ComputeFixFromPVT(DeserializedMessageFilter):

    def __init__(self, source_topic_prefix, fix_topic, fix_detail_topic=None, **kwargs):
        self.pvt_topic = rospy.names.ns_join(source_topic_prefix, 'pvtgeodetic')
        self.cov_topic = rospy.names.ns_join(source_topic_prefix, 'poscovgeodetic')
        super(ComputeFixFromPVT, self).__init__(include_topics=[self.pvt_topic, self.cov_topic], **kwargs)

        self.fix_topic = fix_topic
        self.fix_detail_topic = fix_detail_topic
        if fix_detail_topic is None:
            self.fix_detail_topic = self.fix_topic + '_detail'

        self.last_pvt = message_filters.Cache(message_filters.SimpleFilter(), 10)
        self.last_cov = message_filters.Cache(message_filters.SimpleFilter(), 10)

    def filter(self, topic, msg, stamp, header, tags):  # noqa: A003
        result = [(topic, msg, stamp, header, tags)]

        is_pvt = topic == self.pvt_topic
        if is_pvt:
            self.last_pvt.add(msg)
        else:
            self.last_cov.add(msg)

        pvt = None
        cov = None
        dt = rospy.Duration(0.05)
        if is_pvt:
            msgs = self.last_cov.getInterval(msg.header.stamp - dt, msg.header.stamp + dt)
            if len(msgs) >= 1:
                pvt = msg
                cov = msgs[0]
        else:
            msgs = self.last_pvt.getInterval(msg.header.stamp - dt, msg.header.stamp + dt)
            if len(msgs) >= 1:
                pvt = msgs[0]
                cov = msg

        if pvt is not None and cov is not None:
            nav_msg = NavSatFix()
            gps_msg = GPSFix()

            nav_msg.header = gps_msg.header = pvt.header
            nav_msg.latitude = gps_msg.latitude = degrees(pvt.latitude)
            nav_msg.longitude = gps_msg.longitude = degrees(pvt.longitude)
            nav_msg.altitude = gps_msg.altitude = pvt.height

            gps_in_pvt = False
            glo_in_pvt = False
            com_in_pvt = False
            gal_in_pvt = False
            mask_2 = 1
            for bit in range(32):
                in_use = pvt.signal_info & mask_2
                if bit <= 5 and in_use:
                    gps_in_pvt = True
                if 8 <= bit <= 12 and in_use:
                    glo_in_pvt = True
                if ((13 <= bit <= 14) or (28 <= bit <= 30)) and in_use:
                    com_in_pvt = True
                if (bit == 17 or (19 <= bit <= 22)) and in_use:
                    gal_in_pvt = True
                mask_2 *= 2
            service = gps_in_pvt * 1 + glo_in_pvt * 2 + com_in_pvt * 4 + gal_in_pvt * 8
            nav_msg.status.service = service

            pvt_mode = pvt.mode & 15
            if pvt_mode == 0:  # evNoPVT
                status = GPSStatus.STATUS_NO_FIX
            elif pvt_mode == 1:  # evStandAlone
                status = GPSStatus.STATUS_FIX
            elif pvt_mode == 3:  # evFixed
                status = GPSStatus.STATUS_FIX
            elif pvt_mode == 2:  # evDGPS
                status = GPSStatus.STATUS_DGPS_FIX
            elif pvt_mode == 4:  # evRTKFixed
                status = GPSStatus.STATUS_DGPS_FIX
            elif pvt_mode == 5:  # evRTKFloat
                status = GPSStatus.STATUS_DGPS_FIX
            elif pvt_mode == 7:  # evMovingBaseRTKFixed
                status = GPSStatus.STATUS_DGPS_FIX
            elif pvt_mode == 8:  # evMovingBaseRTKFloat
                status = GPSStatus.STATUS_DGPS_FIX
            elif pvt_mode == 9:  # evPPP
                status = GPSStatus.STATUS_DGPS_FIX
            elif pvt_mode == 6:  # evSBAS
                reference_id = pvt.reference_id
                # Here come the PRNs of the 3 WAAS satellites..
                if reference_id in (131, 133, 135):
                    status = GPSStatus.STATUS_WAAS_FIX
                else:
                    status = GPSStatus.STATUS_SBAS_FIX
            else:
                status = GPSStatus.STATUS_NO_FIX
            nav_msg.status.status = gps_msg.status.status = status

            nav_msg.position_covariance[0] = gps_msg.position_covariance[0] = cov.cov_lonlon
            nav_msg.position_covariance[1] = gps_msg.position_covariance[1] = cov.cov_latlon
            nav_msg.position_covariance[2] = gps_msg.position_covariance[2] = cov.cov_lonhgt
            nav_msg.position_covariance[3] = gps_msg.position_covariance[3] = cov.cov_latlon
            nav_msg.position_covariance[4] = gps_msg.position_covariance[4] = cov.cov_latlat
            nav_msg.position_covariance[5] = gps_msg.position_covariance[5] = cov.cov_lathgt
            nav_msg.position_covariance[6] = gps_msg.position_covariance[6] = cov.cov_lonhgt
            nav_msg.position_covariance[7] = gps_msg.position_covariance[7] = cov.cov_lathgt
            nav_msg.position_covariance[8] = gps_msg.position_covariance[8] = cov.cov_hgthgt
            nav_msg.position_covariance_type = gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

            gps_msg.err_time = 2 * sqrt(cov.cov_bb)
            gps_msg.status.motion_source = GPSStatus.SOURCE_POINTS
            gps_msg.status.orientation_source = GPSStatus.SOURCE_POINTS
            gps_msg.status.position_source = GPSStatus.SOURCE_GPS
            gps_msg.track = pvt.cog
            gps_msg.speed = sqrt(pow(pvt.vn, 2) + pow(pvt.ve, 2))
            gps_msg.climb = pvt.vu
            gps_msg.gdop = gps_msg.pdop = max(pvt.h_accuracy / 100.0, pvt.v_accuracy / 100.0)
            gps_msg.hdop = pvt.h_accuracy / 100.0
            gps_msg.vdop = pvt.v_accuracy / 100.0
            gps_msg.tdop = -1.0
            gps_msg.time = pvt.block_header.tow / 1000.0 + pvt.block_header.wnc / 604800.0

            gps_msg.err = 2 * sqrt(cov.cov_latlat + cov.cov_lonlon + cov.cov_hgthgt)
            gps_msg.err_horz = 2 * sqrt(cov.cov_latlat + cov.cov_lonlon)
            gps_msg.err_vert = 2 * sqrt(cov.cov_hgthgt)
            gps_msg.err_track = 2 * (sqrt(pow(1.0 / (pvt.vn + pow(pvt.ve, 2) / pvt.vn), 2) * cov.cov_lonlon +
                                          pow(pvt.ve / (pow(pvt.vn, 2) + pow(pvt.ve, 2)), 2) * cov.cov_latlat))

            self.last_pvt.cache_msgs.remove(pvt)
            self.last_pvt.cache_times.remove(pvt.header.stamp)
            self.last_cov.cache_msgs.remove(cov)
            self.last_cov.cache_times.remove(cov.header.stamp)

            gen_tags = tags_for_generated_msg(tags)
            result.append((self.fix_topic, nav_msg, stamp, header, gen_tags))
            result.append((self.fix_detail_topic, gps_msg, stamp, header, gen_tags))

        return result

    def reset(self):
        del self.last_pvt.cache_times[:]
        del self.last_pvt.cache_msgs[:]
        del self.last_cov.cache_times[:]
        del self.last_cov.cache_msgs[:]
        super(ComputeFixFromPVT, self).reset()

    def _str_params(self):
        parts = [
            'pvt_topic=' + self.pvt_topic,
            'cov_topic=' + self.cov_topic,
            'fix_topic=' + self.fix_topic,
            'fix_detail_topic=' + self.fix_detail_topic,
        ]
        parent_params = super(ComputeFixFromPVT, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ', '.join(parts)
