/** @file
 *	@brief MAVLink comm testsuite protocol generated from common.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "common.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(common, SYS_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SYS_STATUS packet_in{};
    packet_in.onboard_control_sensors_present = 963497464;
    packet_in.onboard_control_sensors_enabled = 963497672;
    packet_in.onboard_control_sensors_health = 963497880;
    packet_in.load = 17859;
    packet_in.voltage_battery = 17963;
    packet_in.current_battery = 18067;
    packet_in.battery_remaining = -33;
    packet_in.drop_rate_comm = 18171;
    packet_in.errors_comm = 18275;
    packet_in.errors_count1 = 18379;
    packet_in.errors_count2 = 18483;
    packet_in.errors_count3 = 18587;
    packet_in.errors_count4 = 18691;
    packet_in.onboard_control_sensors_present_extended = 963499076;
    packet_in.onboard_control_sensors_enabled_extended = 963499284;
    packet_in.onboard_control_sensors_health_extended = 963499492;

    mavlink::common::msg::SYS_STATUS packet1{};
    mavlink::common::msg::SYS_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.onboard_control_sensors_present, packet2.onboard_control_sensors_present);
    EXPECT_EQ(packet1.onboard_control_sensors_enabled, packet2.onboard_control_sensors_enabled);
    EXPECT_EQ(packet1.onboard_control_sensors_health, packet2.onboard_control_sensors_health);
    EXPECT_EQ(packet1.load, packet2.load);
    EXPECT_EQ(packet1.voltage_battery, packet2.voltage_battery);
    EXPECT_EQ(packet1.current_battery, packet2.current_battery);
    EXPECT_EQ(packet1.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet1.drop_rate_comm, packet2.drop_rate_comm);
    EXPECT_EQ(packet1.errors_comm, packet2.errors_comm);
    EXPECT_EQ(packet1.errors_count1, packet2.errors_count1);
    EXPECT_EQ(packet1.errors_count2, packet2.errors_count2);
    EXPECT_EQ(packet1.errors_count3, packet2.errors_count3);
    EXPECT_EQ(packet1.errors_count4, packet2.errors_count4);
    EXPECT_EQ(packet1.onboard_control_sensors_present_extended, packet2.onboard_control_sensors_present_extended);
    EXPECT_EQ(packet1.onboard_control_sensors_enabled_extended, packet2.onboard_control_sensors_enabled_extended);
    EXPECT_EQ(packet1.onboard_control_sensors_health_extended, packet2.onboard_control_sensors_health_extended);
}

#ifdef TEST_INTEROP
TEST(common_interop, SYS_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_sys_status_t packet_c {
         963497464, 963497672, 963497880, 17859, 17963, 18067, 18171, 18275, 18379, 18483, 18587, 18691, -33, 963499076, 963499284, 963499492
    };

    mavlink::common::msg::SYS_STATUS packet_in{};
    packet_in.onboard_control_sensors_present = 963497464;
    packet_in.onboard_control_sensors_enabled = 963497672;
    packet_in.onboard_control_sensors_health = 963497880;
    packet_in.load = 17859;
    packet_in.voltage_battery = 17963;
    packet_in.current_battery = 18067;
    packet_in.battery_remaining = -33;
    packet_in.drop_rate_comm = 18171;
    packet_in.errors_comm = 18275;
    packet_in.errors_count1 = 18379;
    packet_in.errors_count2 = 18483;
    packet_in.errors_count3 = 18587;
    packet_in.errors_count4 = 18691;
    packet_in.onboard_control_sensors_present_extended = 963499076;
    packet_in.onboard_control_sensors_enabled_extended = 963499284;
    packet_in.onboard_control_sensors_health_extended = 963499492;

    mavlink::common::msg::SYS_STATUS packet2{};

    mavlink_msg_sys_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.onboard_control_sensors_present, packet2.onboard_control_sensors_present);
    EXPECT_EQ(packet_in.onboard_control_sensors_enabled, packet2.onboard_control_sensors_enabled);
    EXPECT_EQ(packet_in.onboard_control_sensors_health, packet2.onboard_control_sensors_health);
    EXPECT_EQ(packet_in.load, packet2.load);
    EXPECT_EQ(packet_in.voltage_battery, packet2.voltage_battery);
    EXPECT_EQ(packet_in.current_battery, packet2.current_battery);
    EXPECT_EQ(packet_in.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet_in.drop_rate_comm, packet2.drop_rate_comm);
    EXPECT_EQ(packet_in.errors_comm, packet2.errors_comm);
    EXPECT_EQ(packet_in.errors_count1, packet2.errors_count1);
    EXPECT_EQ(packet_in.errors_count2, packet2.errors_count2);
    EXPECT_EQ(packet_in.errors_count3, packet2.errors_count3);
    EXPECT_EQ(packet_in.errors_count4, packet2.errors_count4);
    EXPECT_EQ(packet_in.onboard_control_sensors_present_extended, packet2.onboard_control_sensors_present_extended);
    EXPECT_EQ(packet_in.onboard_control_sensors_enabled_extended, packet2.onboard_control_sensors_enabled_extended);
    EXPECT_EQ(packet_in.onboard_control_sensors_health_extended, packet2.onboard_control_sensors_health_extended);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SYSTEM_TIME)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SYSTEM_TIME packet_in{};
    packet_in.time_unix_usec = 93372036854775807ULL;
    packet_in.time_boot_ms = 963497880;

    mavlink::common::msg::SYSTEM_TIME packet1{};
    mavlink::common::msg::SYSTEM_TIME packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_unix_usec, packet2.time_unix_usec);
    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
}

#ifdef TEST_INTEROP
TEST(common_interop, SYSTEM_TIME)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_system_time_t packet_c {
         93372036854775807ULL, 963497880
    };

    mavlink::common::msg::SYSTEM_TIME packet_in{};
    packet_in.time_unix_usec = 93372036854775807ULL;
    packet_in.time_boot_ms = 963497880;

    mavlink::common::msg::SYSTEM_TIME packet2{};

    mavlink_msg_system_time_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_unix_usec, packet2.time_unix_usec);
    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PING)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PING packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;
    packet_in.target_system = 41;
    packet_in.target_component = 108;

    mavlink::common::msg::PING packet1{};
    mavlink::common::msg::PING packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, PING)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_ping_t packet_c {
         93372036854775807ULL, 963497880, 41, 108
    };

    mavlink::common::msg::PING packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;
    packet_in.target_system = 41;
    packet_in.target_component = 108;

    mavlink::common::msg::PING packet2{};

    mavlink_msg_ping_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CHANGE_OPERATOR_CONTROL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet_in{};
    packet_in.target_system = 5;
    packet_in.control_request = 72;
    packet_in.version = 139;
    packet_in.passkey = to_char_array("DEFGHIJKLMNOPQRSTUVWXYZA");

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet1{};
    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.control_request, packet2.control_request);
    EXPECT_EQ(packet1.version, packet2.version);
    EXPECT_EQ(packet1.passkey, packet2.passkey);
}

#ifdef TEST_INTEROP
TEST(common_interop, CHANGE_OPERATOR_CONTROL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_change_operator_control_t packet_c {
         5, 72, 139, "DEFGHIJKLMNOPQRSTUVWXYZA"
    };

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet_in{};
    packet_in.target_system = 5;
    packet_in.control_request = 72;
    packet_in.version = 139;
    packet_in.passkey = to_char_array("DEFGHIJKLMNOPQRSTUVWXYZA");

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet2{};

    mavlink_msg_change_operator_control_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.control_request, packet2.control_request);
    EXPECT_EQ(packet_in.version, packet2.version);
    EXPECT_EQ(packet_in.passkey, packet2.passkey);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CHANGE_OPERATOR_CONTROL_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet_in{};
    packet_in.gcs_system_id = 5;
    packet_in.control_request = 72;
    packet_in.ack = 139;

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet1{};
    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.gcs_system_id, packet2.gcs_system_id);
    EXPECT_EQ(packet1.control_request, packet2.control_request);
    EXPECT_EQ(packet1.ack, packet2.ack);
}

#ifdef TEST_INTEROP
TEST(common_interop, CHANGE_OPERATOR_CONTROL_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_change_operator_control_ack_t packet_c {
         5, 72, 139
    };

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet_in{};
    packet_in.gcs_system_id = 5;
    packet_in.control_request = 72;
    packet_in.ack = 139;

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet2{};

    mavlink_msg_change_operator_control_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.gcs_system_id, packet2.gcs_system_id);
    EXPECT_EQ(packet_in.control_request, packet2.control_request);
    EXPECT_EQ(packet_in.ack, packet2.ack);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, AUTH_KEY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::AUTH_KEY packet_in{};
    packet_in.key = to_char_array("ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");

    mavlink::common::msg::AUTH_KEY packet1{};
    mavlink::common::msg::AUTH_KEY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.key, packet2.key);
}

#ifdef TEST_INTEROP
TEST(common_interop, AUTH_KEY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auth_key_t packet_c {
         "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE"
    };

    mavlink::common::msg::AUTH_KEY packet_in{};
    packet_in.key = to_char_array("ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");

    mavlink::common::msg::AUTH_KEY packet2{};

    mavlink_msg_auth_key_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.key, packet2.key);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LINK_NODE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LINK_NODE_STATUS packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.tx_buf = 235;
    packet_in.rx_buf = 46;
    packet_in.tx_rate = 963497880;
    packet_in.rx_rate = 963498088;
    packet_in.rx_parse_err = 18691;
    packet_in.tx_overflows = 18795;
    packet_in.rx_overflows = 18899;
    packet_in.messages_sent = 963498296;
    packet_in.messages_received = 963498504;
    packet_in.messages_lost = 963498712;

    mavlink::common::msg::LINK_NODE_STATUS packet1{};
    mavlink::common::msg::LINK_NODE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.tx_buf, packet2.tx_buf);
    EXPECT_EQ(packet1.rx_buf, packet2.rx_buf);
    EXPECT_EQ(packet1.tx_rate, packet2.tx_rate);
    EXPECT_EQ(packet1.rx_rate, packet2.rx_rate);
    EXPECT_EQ(packet1.rx_parse_err, packet2.rx_parse_err);
    EXPECT_EQ(packet1.tx_overflows, packet2.tx_overflows);
    EXPECT_EQ(packet1.rx_overflows, packet2.rx_overflows);
    EXPECT_EQ(packet1.messages_sent, packet2.messages_sent);
    EXPECT_EQ(packet1.messages_received, packet2.messages_received);
    EXPECT_EQ(packet1.messages_lost, packet2.messages_lost);
}

#ifdef TEST_INTEROP
TEST(common_interop, LINK_NODE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_link_node_status_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 963498504, 963498712, 18691, 18795, 18899, 235, 46
    };

    mavlink::common::msg::LINK_NODE_STATUS packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.tx_buf = 235;
    packet_in.rx_buf = 46;
    packet_in.tx_rate = 963497880;
    packet_in.rx_rate = 963498088;
    packet_in.rx_parse_err = 18691;
    packet_in.tx_overflows = 18795;
    packet_in.rx_overflows = 18899;
    packet_in.messages_sent = 963498296;
    packet_in.messages_received = 963498504;
    packet_in.messages_lost = 963498712;

    mavlink::common::msg::LINK_NODE_STATUS packet2{};

    mavlink_msg_link_node_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.tx_buf, packet2.tx_buf);
    EXPECT_EQ(packet_in.rx_buf, packet2.rx_buf);
    EXPECT_EQ(packet_in.tx_rate, packet2.tx_rate);
    EXPECT_EQ(packet_in.rx_rate, packet2.rx_rate);
    EXPECT_EQ(packet_in.rx_parse_err, packet2.rx_parse_err);
    EXPECT_EQ(packet_in.tx_overflows, packet2.tx_overflows);
    EXPECT_EQ(packet_in.rx_overflows, packet2.rx_overflows);
    EXPECT_EQ(packet_in.messages_sent, packet2.messages_sent);
    EXPECT_EQ(packet_in.messages_received, packet2.messages_received);
    EXPECT_EQ(packet_in.messages_lost, packet2.messages_lost);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_MODE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_MODE packet_in{};
    packet_in.target_system = 17;
    packet_in.base_mode = 84;
    packet_in.custom_mode = 963497464;

    mavlink::common::msg::SET_MODE packet1{};
    mavlink::common::msg::SET_MODE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.base_mode, packet2.base_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_MODE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_mode_t packet_c {
         963497464, 17, 84
    };

    mavlink::common::msg::SET_MODE packet_in{};
    packet_in.target_system = 17;
    packet_in.base_mode = 84;
    packet_in.custom_mode = 963497464;

    mavlink::common::msg::SET_MODE packet2{};

    mavlink_msg_set_mode_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.base_mode, packet2.base_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_REQUEST_READ)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::common::msg::PARAM_REQUEST_READ packet1{};
    mavlink::common::msg::PARAM_REQUEST_READ packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_REQUEST_READ)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_request_read_t packet_c {
         17235, 139, 206, "EFGHIJKLMNOPQRS"
    };

    mavlink::common::msg::PARAM_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::common::msg::PARAM_REQUEST_READ packet2{};

    mavlink_msg_param_request_read_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::PARAM_REQUEST_LIST packet1{};
    mavlink::common::msg::PARAM_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_request_list_t packet_c {
         5, 72
    };

    mavlink::common::msg::PARAM_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::PARAM_REQUEST_LIST packet2{};

    mavlink_msg_param_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_VALUE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_VALUE packet_in{};
    packet_in.param_id = to_char_array("IJKLMNOPQRSTUVW");
    packet_in.param_value = 17.0;
    packet_in.param_type = 77;
    packet_in.param_count = 17443;
    packet_in.param_index = 17547;

    mavlink::common::msg::PARAM_VALUE packet1{};
    mavlink::common::msg::PARAM_VALUE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
    EXPECT_EQ(packet1.param_count, packet2.param_count);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_VALUE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_value_t packet_c {
         17.0, 17443, 17547, "IJKLMNOPQRSTUVW", 77
    };

    mavlink::common::msg::PARAM_VALUE packet_in{};
    packet_in.param_id = to_char_array("IJKLMNOPQRSTUVW");
    packet_in.param_value = 17.0;
    packet_in.param_type = 77;
    packet_in.param_count = 17443;
    packet_in.param_index = 17547;

    mavlink::common::msg::PARAM_VALUE packet2{};

    mavlink_msg_param_value_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);
    EXPECT_EQ(packet_in.param_count, packet2.param_count);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_SET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_SET packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;

    mavlink::common::msg::PARAM_SET packet1{};
    mavlink::common::msg::PARAM_SET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_SET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_set_t packet_c {
         17.0, 17, 84, "GHIJKLMNOPQRSTU", 199
    };

    mavlink::common::msg::PARAM_SET packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;

    mavlink::common::msg::PARAM_SET packet2{};

    mavlink_msg_param_set_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_RAW_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_RAW_INT packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 89;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18275;
    packet_in.epv = 18379;
    packet_in.vel = 18483;
    packet_in.cog = 18587;
    packet_in.satellites_visible = 156;
    packet_in.alt_ellipsoid = 963499024;
    packet_in.h_acc = 963499232;
    packet_in.v_acc = 963499440;
    packet_in.vel_acc = 963499648;
    packet_in.hdg_acc = 963499856;
    packet_in.yaw = 19835;

    mavlink::common::msg::GPS_RAW_INT packet1{};
    mavlink::common::msg::GPS_RAW_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.fix_type, packet2.fix_type);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.eph, packet2.eph);
    EXPECT_EQ(packet1.epv, packet2.epv);
    EXPECT_EQ(packet1.vel, packet2.vel);
    EXPECT_EQ(packet1.cog, packet2.cog);
    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet1.alt_ellipsoid, packet2.alt_ellipsoid);
    EXPECT_EQ(packet1.h_acc, packet2.h_acc);
    EXPECT_EQ(packet1.v_acc, packet2.v_acc);
    EXPECT_EQ(packet1.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet1.hdg_acc, packet2.hdg_acc);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_RAW_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_raw_int_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 18275, 18379, 18483, 18587, 89, 156, 963499024, 963499232, 963499440, 963499648, 963499856, 19835
    };

    mavlink::common::msg::GPS_RAW_INT packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 89;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18275;
    packet_in.epv = 18379;
    packet_in.vel = 18483;
    packet_in.cog = 18587;
    packet_in.satellites_visible = 156;
    packet_in.alt_ellipsoid = 963499024;
    packet_in.h_acc = 963499232;
    packet_in.v_acc = 963499440;
    packet_in.vel_acc = 963499648;
    packet_in.hdg_acc = 963499856;
    packet_in.yaw = 19835;

    mavlink::common::msg::GPS_RAW_INT packet2{};

    mavlink_msg_gps_raw_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.fix_type, packet2.fix_type);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.eph, packet2.eph);
    EXPECT_EQ(packet_in.epv, packet2.epv);
    EXPECT_EQ(packet_in.vel, packet2.vel);
    EXPECT_EQ(packet_in.cog, packet2.cog);
    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet_in.alt_ellipsoid, packet2.alt_ellipsoid);
    EXPECT_EQ(packet_in.h_acc, packet2.h_acc);
    EXPECT_EQ(packet_in.v_acc, packet2.v_acc);
    EXPECT_EQ(packet_in.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet_in.hdg_acc, packet2.hdg_acc);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_STATUS packet_in{};
    packet_in.satellites_visible = 5;
    packet_in.satellite_prn = {{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 }};
    packet_in.satellite_used = {{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 }};
    packet_in.satellite_elevation = {{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }};
    packet_in.satellite_azimuth = {{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }};
    packet_in.satellite_snr = {{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }};

    mavlink::common::msg::GPS_STATUS packet1{};
    mavlink::common::msg::GPS_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet1.satellite_prn, packet2.satellite_prn);
    EXPECT_EQ(packet1.satellite_used, packet2.satellite_used);
    EXPECT_EQ(packet1.satellite_elevation, packet2.satellite_elevation);
    EXPECT_EQ(packet1.satellite_azimuth, packet2.satellite_azimuth);
    EXPECT_EQ(packet1.satellite_snr, packet2.satellite_snr);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_status_t packet_c {
         5, { 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 }, { 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 }, { 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }, { 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }, { 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }
    };

    mavlink::common::msg::GPS_STATUS packet_in{};
    packet_in.satellites_visible = 5;
    packet_in.satellite_prn = {{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 }};
    packet_in.satellite_used = {{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 }};
    packet_in.satellite_elevation = {{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }};
    packet_in.satellite_azimuth = {{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }};
    packet_in.satellite_snr = {{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }};

    mavlink::common::msg::GPS_STATUS packet2{};

    mavlink_msg_gps_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet_in.satellite_prn, packet2.satellite_prn);
    EXPECT_EQ(packet_in.satellite_used, packet2.satellite_used);
    EXPECT_EQ(packet_in.satellite_elevation, packet2.satellite_elevation);
    EXPECT_EQ(packet_in.satellite_azimuth, packet2.satellite_azimuth);
    EXPECT_EQ(packet_in.satellite_snr, packet2.satellite_snr);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SCALED_IMU)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SCALED_IMU packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.xacc = 17443;
    packet_in.yacc = 17547;
    packet_in.zacc = 17651;
    packet_in.xgyro = 17755;
    packet_in.ygyro = 17859;
    packet_in.zgyro = 17963;
    packet_in.xmag = 18067;
    packet_in.ymag = 18171;
    packet_in.zmag = 18275;
    packet_in.temperature = 18379;

    mavlink::common::msg::SCALED_IMU packet1{};
    mavlink::common::msg::SCALED_IMU packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.xmag, packet2.xmag);
    EXPECT_EQ(packet1.ymag, packet2.ymag);
    EXPECT_EQ(packet1.zmag, packet2.zmag);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
}

#ifdef TEST_INTEROP
TEST(common_interop, SCALED_IMU)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_scaled_imu_t packet_c {
         963497464, 17443, 17547, 17651, 17755, 17859, 17963, 18067, 18171, 18275, 18379
    };

    mavlink::common::msg::SCALED_IMU packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.xacc = 17443;
    packet_in.yacc = 17547;
    packet_in.zacc = 17651;
    packet_in.xgyro = 17755;
    packet_in.ygyro = 17859;
    packet_in.zgyro = 17963;
    packet_in.xmag = 18067;
    packet_in.ymag = 18171;
    packet_in.zmag = 18275;
    packet_in.temperature = 18379;

    mavlink::common::msg::SCALED_IMU packet2{};

    mavlink_msg_scaled_imu_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.xmag, packet2.xmag);
    EXPECT_EQ(packet_in.ymag, packet2.ymag);
    EXPECT_EQ(packet_in.zmag, packet2.zmag);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RAW_IMU)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RAW_IMU packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 17651;
    packet_in.yacc = 17755;
    packet_in.zacc = 17859;
    packet_in.xgyro = 17963;
    packet_in.ygyro = 18067;
    packet_in.zgyro = 18171;
    packet_in.xmag = 18275;
    packet_in.ymag = 18379;
    packet_in.zmag = 18483;
    packet_in.id = 211;
    packet_in.temperature = 18639;

    mavlink::common::msg::RAW_IMU packet1{};
    mavlink::common::msg::RAW_IMU packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.xmag, packet2.xmag);
    EXPECT_EQ(packet1.ymag, packet2.ymag);
    EXPECT_EQ(packet1.zmag, packet2.zmag);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
}

#ifdef TEST_INTEROP
TEST(common_interop, RAW_IMU)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_raw_imu_t packet_c {
         93372036854775807ULL, 17651, 17755, 17859, 17963, 18067, 18171, 18275, 18379, 18483, 211, 18639
    };

    mavlink::common::msg::RAW_IMU packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 17651;
    packet_in.yacc = 17755;
    packet_in.zacc = 17859;
    packet_in.xgyro = 17963;
    packet_in.ygyro = 18067;
    packet_in.zgyro = 18171;
    packet_in.xmag = 18275;
    packet_in.ymag = 18379;
    packet_in.zmag = 18483;
    packet_in.id = 211;
    packet_in.temperature = 18639;

    mavlink::common::msg::RAW_IMU packet2{};

    mavlink_msg_raw_imu_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.xmag, packet2.xmag);
    EXPECT_EQ(packet_in.ymag, packet2.ymag);
    EXPECT_EQ(packet_in.zmag, packet2.zmag);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RAW_PRESSURE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RAW_PRESSURE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.press_abs = 17651;
    packet_in.press_diff1 = 17755;
    packet_in.press_diff2 = 17859;
    packet_in.temperature = 17963;

    mavlink::common::msg::RAW_PRESSURE packet1{};
    mavlink::common::msg::RAW_PRESSURE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.press_abs, packet2.press_abs);
    EXPECT_EQ(packet1.press_diff1, packet2.press_diff1);
    EXPECT_EQ(packet1.press_diff2, packet2.press_diff2);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
}

#ifdef TEST_INTEROP
TEST(common_interop, RAW_PRESSURE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_raw_pressure_t packet_c {
         93372036854775807ULL, 17651, 17755, 17859, 17963
    };

    mavlink::common::msg::RAW_PRESSURE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.press_abs = 17651;
    packet_in.press_diff1 = 17755;
    packet_in.press_diff2 = 17859;
    packet_in.temperature = 17963;

    mavlink::common::msg::RAW_PRESSURE packet2{};

    mavlink_msg_raw_pressure_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.press_abs, packet2.press_abs);
    EXPECT_EQ(packet_in.press_diff1, packet2.press_diff1);
    EXPECT_EQ(packet_in.press_diff2, packet2.press_diff2);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SCALED_PRESSURE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SCALED_PRESSURE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;
    packet_in.temperature_press_diff = 17963;

    mavlink::common::msg::SCALED_PRESSURE packet1{};
    mavlink::common::msg::SCALED_PRESSURE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.press_abs, packet2.press_abs);
    EXPECT_EQ(packet1.press_diff, packet2.press_diff);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.temperature_press_diff, packet2.temperature_press_diff);
}

#ifdef TEST_INTEROP
TEST(common_interop, SCALED_PRESSURE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_scaled_pressure_t packet_c {
         963497464, 45.0, 73.0, 17859, 17963
    };

    mavlink::common::msg::SCALED_PRESSURE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;
    packet_in.temperature_press_diff = 17963;

    mavlink::common::msg::SCALED_PRESSURE packet2{};

    mavlink_msg_scaled_pressure_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.press_abs, packet2.press_abs);
    EXPECT_EQ(packet_in.press_diff, packet2.press_diff);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.temperature_press_diff, packet2.temperature_press_diff);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::common::msg::ATTITUDE packet1{};
    mavlink::common::msg::ATTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::common::msg::ATTITUDE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::common::msg::ATTITUDE packet2{};

    mavlink_msg_attitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE_QUATERNION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE_QUATERNION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;
    packet_in.repr_offset_q = {{ 241.0, 242.0, 243.0, 244.0 }};

    mavlink::common::msg::ATTITUDE_QUATERNION packet1{};
    mavlink::common::msg::ATTITUDE_QUATERNION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.q1, packet2.q1);
    EXPECT_EQ(packet1.q2, packet2.q2);
    EXPECT_EQ(packet1.q3, packet2.q3);
    EXPECT_EQ(packet1.q4, packet2.q4);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet1.repr_offset_q, packet2.repr_offset_q);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE_QUATERNION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_quaternion_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, { 241.0, 242.0, 243.0, 244.0 }
    };

    mavlink::common::msg::ATTITUDE_QUATERNION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;
    packet_in.repr_offset_q = {{ 241.0, 242.0, 243.0, 244.0 }};

    mavlink::common::msg::ATTITUDE_QUATERNION packet2{};

    mavlink_msg_attitude_quaternion_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.q1, packet2.q1);
    EXPECT_EQ(packet_in.q2, packet2.q2);
    EXPECT_EQ(packet_in.q3, packet2.q3);
    EXPECT_EQ(packet_in.q4, packet2.q4);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet_in.repr_offset_q, packet2.repr_offset_q);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOCAL_POSITION_NED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOCAL_POSITION_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;

    mavlink::common::msg::LOCAL_POSITION_NED packet1{};
    mavlink::common::msg::LOCAL_POSITION_NED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOCAL_POSITION_NED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_local_position_ned_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::common::msg::LOCAL_POSITION_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;

    mavlink::common::msg::LOCAL_POSITION_NED packet2{};

    mavlink_msg_local_position_ned_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GLOBAL_POSITION_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GLOBAL_POSITION_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.relative_alt = 963498296;
    packet_in.vx = 18275;
    packet_in.vy = 18379;
    packet_in.vz = 18483;
    packet_in.hdg = 18587;

    mavlink::common::msg::GLOBAL_POSITION_INT packet1{};
    mavlink::common::msg::GLOBAL_POSITION_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.hdg, packet2.hdg);
}

#ifdef TEST_INTEROP
TEST(common_interop, GLOBAL_POSITION_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_global_position_int_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296, 18275, 18379, 18483, 18587
    };

    mavlink::common::msg::GLOBAL_POSITION_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.relative_alt = 963498296;
    packet_in.vx = 18275;
    packet_in.vy = 18379;
    packet_in.vz = 18483;
    packet_in.hdg = 18587;

    mavlink::common::msg::GLOBAL_POSITION_INT packet2{};

    mavlink_msg_global_position_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.hdg, packet2.hdg);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RC_CHANNELS_SCALED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RC_CHANNELS_SCALED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.port = 65;
    packet_in.chan1_scaled = 17443;
    packet_in.chan2_scaled = 17547;
    packet_in.chan3_scaled = 17651;
    packet_in.chan4_scaled = 17755;
    packet_in.chan5_scaled = 17859;
    packet_in.chan6_scaled = 17963;
    packet_in.chan7_scaled = 18067;
    packet_in.chan8_scaled = 18171;
    packet_in.rssi = 132;

    mavlink::common::msg::RC_CHANNELS_SCALED packet1{};
    mavlink::common::msg::RC_CHANNELS_SCALED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.port, packet2.port);
    EXPECT_EQ(packet1.chan1_scaled, packet2.chan1_scaled);
    EXPECT_EQ(packet1.chan2_scaled, packet2.chan2_scaled);
    EXPECT_EQ(packet1.chan3_scaled, packet2.chan3_scaled);
    EXPECT_EQ(packet1.chan4_scaled, packet2.chan4_scaled);
    EXPECT_EQ(packet1.chan5_scaled, packet2.chan5_scaled);
    EXPECT_EQ(packet1.chan6_scaled, packet2.chan6_scaled);
    EXPECT_EQ(packet1.chan7_scaled, packet2.chan7_scaled);
    EXPECT_EQ(packet1.chan8_scaled, packet2.chan8_scaled);
    EXPECT_EQ(packet1.rssi, packet2.rssi);
}

#ifdef TEST_INTEROP
TEST(common_interop, RC_CHANNELS_SCALED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_rc_channels_scaled_t packet_c {
         963497464, 17443, 17547, 17651, 17755, 17859, 17963, 18067, 18171, 65, 132
    };

    mavlink::common::msg::RC_CHANNELS_SCALED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.port = 65;
    packet_in.chan1_scaled = 17443;
    packet_in.chan2_scaled = 17547;
    packet_in.chan3_scaled = 17651;
    packet_in.chan4_scaled = 17755;
    packet_in.chan5_scaled = 17859;
    packet_in.chan6_scaled = 17963;
    packet_in.chan7_scaled = 18067;
    packet_in.chan8_scaled = 18171;
    packet_in.rssi = 132;

    mavlink::common::msg::RC_CHANNELS_SCALED packet2{};

    mavlink_msg_rc_channels_scaled_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.port, packet2.port);
    EXPECT_EQ(packet_in.chan1_scaled, packet2.chan1_scaled);
    EXPECT_EQ(packet_in.chan2_scaled, packet2.chan2_scaled);
    EXPECT_EQ(packet_in.chan3_scaled, packet2.chan3_scaled);
    EXPECT_EQ(packet_in.chan4_scaled, packet2.chan4_scaled);
    EXPECT_EQ(packet_in.chan5_scaled, packet2.chan5_scaled);
    EXPECT_EQ(packet_in.chan6_scaled, packet2.chan6_scaled);
    EXPECT_EQ(packet_in.chan7_scaled, packet2.chan7_scaled);
    EXPECT_EQ(packet_in.chan8_scaled, packet2.chan8_scaled);
    EXPECT_EQ(packet_in.rssi, packet2.rssi);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RC_CHANNELS_RAW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RC_CHANNELS_RAW packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.port = 65;
    packet_in.chan1_raw = 17443;
    packet_in.chan2_raw = 17547;
    packet_in.chan3_raw = 17651;
    packet_in.chan4_raw = 17755;
    packet_in.chan5_raw = 17859;
    packet_in.chan6_raw = 17963;
    packet_in.chan7_raw = 18067;
    packet_in.chan8_raw = 18171;
    packet_in.rssi = 132;

    mavlink::common::msg::RC_CHANNELS_RAW packet1{};
    mavlink::common::msg::RC_CHANNELS_RAW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.port, packet2.port);
    EXPECT_EQ(packet1.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet1.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet1.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet1.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet1.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet1.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet1.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet1.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet1.rssi, packet2.rssi);
}

#ifdef TEST_INTEROP
TEST(common_interop, RC_CHANNELS_RAW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_rc_channels_raw_t packet_c {
         963497464, 17443, 17547, 17651, 17755, 17859, 17963, 18067, 18171, 65, 132
    };

    mavlink::common::msg::RC_CHANNELS_RAW packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.port = 65;
    packet_in.chan1_raw = 17443;
    packet_in.chan2_raw = 17547;
    packet_in.chan3_raw = 17651;
    packet_in.chan4_raw = 17755;
    packet_in.chan5_raw = 17859;
    packet_in.chan6_raw = 17963;
    packet_in.chan7_raw = 18067;
    packet_in.chan8_raw = 18171;
    packet_in.rssi = 132;

    mavlink::common::msg::RC_CHANNELS_RAW packet2{};

    mavlink_msg_rc_channels_raw_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.port, packet2.port);
    EXPECT_EQ(packet_in.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet_in.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet_in.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet_in.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet_in.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet_in.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet_in.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet_in.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet_in.rssi, packet2.rssi);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SERVO_OUTPUT_RAW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SERVO_OUTPUT_RAW packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.port = 65;
    packet_in.servo1_raw = 17443;
    packet_in.servo2_raw = 17547;
    packet_in.servo3_raw = 17651;
    packet_in.servo4_raw = 17755;
    packet_in.servo5_raw = 17859;
    packet_in.servo6_raw = 17963;
    packet_in.servo7_raw = 18067;
    packet_in.servo8_raw = 18171;
    packet_in.servo9_raw = 18327;
    packet_in.servo10_raw = 18431;
    packet_in.servo11_raw = 18535;
    packet_in.servo12_raw = 18639;
    packet_in.servo13_raw = 18743;
    packet_in.servo14_raw = 18847;
    packet_in.servo15_raw = 18951;
    packet_in.servo16_raw = 19055;

    mavlink::common::msg::SERVO_OUTPUT_RAW packet1{};
    mavlink::common::msg::SERVO_OUTPUT_RAW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.port, packet2.port);
    EXPECT_EQ(packet1.servo1_raw, packet2.servo1_raw);
    EXPECT_EQ(packet1.servo2_raw, packet2.servo2_raw);
    EXPECT_EQ(packet1.servo3_raw, packet2.servo3_raw);
    EXPECT_EQ(packet1.servo4_raw, packet2.servo4_raw);
    EXPECT_EQ(packet1.servo5_raw, packet2.servo5_raw);
    EXPECT_EQ(packet1.servo6_raw, packet2.servo6_raw);
    EXPECT_EQ(packet1.servo7_raw, packet2.servo7_raw);
    EXPECT_EQ(packet1.servo8_raw, packet2.servo8_raw);
    EXPECT_EQ(packet1.servo9_raw, packet2.servo9_raw);
    EXPECT_EQ(packet1.servo10_raw, packet2.servo10_raw);
    EXPECT_EQ(packet1.servo11_raw, packet2.servo11_raw);
    EXPECT_EQ(packet1.servo12_raw, packet2.servo12_raw);
    EXPECT_EQ(packet1.servo13_raw, packet2.servo13_raw);
    EXPECT_EQ(packet1.servo14_raw, packet2.servo14_raw);
    EXPECT_EQ(packet1.servo15_raw, packet2.servo15_raw);
    EXPECT_EQ(packet1.servo16_raw, packet2.servo16_raw);
}

#ifdef TEST_INTEROP
TEST(common_interop, SERVO_OUTPUT_RAW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_servo_output_raw_t packet_c {
         963497464, 17443, 17547, 17651, 17755, 17859, 17963, 18067, 18171, 65, 18327, 18431, 18535, 18639, 18743, 18847, 18951, 19055
    };

    mavlink::common::msg::SERVO_OUTPUT_RAW packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.port = 65;
    packet_in.servo1_raw = 17443;
    packet_in.servo2_raw = 17547;
    packet_in.servo3_raw = 17651;
    packet_in.servo4_raw = 17755;
    packet_in.servo5_raw = 17859;
    packet_in.servo6_raw = 17963;
    packet_in.servo7_raw = 18067;
    packet_in.servo8_raw = 18171;
    packet_in.servo9_raw = 18327;
    packet_in.servo10_raw = 18431;
    packet_in.servo11_raw = 18535;
    packet_in.servo12_raw = 18639;
    packet_in.servo13_raw = 18743;
    packet_in.servo14_raw = 18847;
    packet_in.servo15_raw = 18951;
    packet_in.servo16_raw = 19055;

    mavlink::common::msg::SERVO_OUTPUT_RAW packet2{};

    mavlink_msg_servo_output_raw_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.port, packet2.port);
    EXPECT_EQ(packet_in.servo1_raw, packet2.servo1_raw);
    EXPECT_EQ(packet_in.servo2_raw, packet2.servo2_raw);
    EXPECT_EQ(packet_in.servo3_raw, packet2.servo3_raw);
    EXPECT_EQ(packet_in.servo4_raw, packet2.servo4_raw);
    EXPECT_EQ(packet_in.servo5_raw, packet2.servo5_raw);
    EXPECT_EQ(packet_in.servo6_raw, packet2.servo6_raw);
    EXPECT_EQ(packet_in.servo7_raw, packet2.servo7_raw);
    EXPECT_EQ(packet_in.servo8_raw, packet2.servo8_raw);
    EXPECT_EQ(packet_in.servo9_raw, packet2.servo9_raw);
    EXPECT_EQ(packet_in.servo10_raw, packet2.servo10_raw);
    EXPECT_EQ(packet_in.servo11_raw, packet2.servo11_raw);
    EXPECT_EQ(packet_in.servo12_raw, packet2.servo12_raw);
    EXPECT_EQ(packet_in.servo13_raw, packet2.servo13_raw);
    EXPECT_EQ(packet_in.servo14_raw, packet2.servo14_raw);
    EXPECT_EQ(packet_in.servo15_raw, packet2.servo15_raw);
    EXPECT_EQ(packet_in.servo16_raw, packet2.servo16_raw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_REQUEST_PARTIAL_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_REQUEST_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::common::msg::MISSION_REQUEST_PARTIAL_LIST packet1{};
    mavlink::common::msg::MISSION_REQUEST_PARTIAL_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.start_index, packet2.start_index);
    EXPECT_EQ(packet1.end_index, packet2.end_index);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_REQUEST_PARTIAL_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_partial_list_t packet_c {
         17235, 17339, 17, 84, 151
    };

    mavlink::common::msg::MISSION_REQUEST_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::common::msg::MISSION_REQUEST_PARTIAL_LIST packet2{};

    mavlink_msg_mission_request_partial_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.start_index, packet2.start_index);
    EXPECT_EQ(packet_in.end_index, packet2.end_index);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_WRITE_PARTIAL_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST packet1{};
    mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.start_index, packet2.start_index);
    EXPECT_EQ(packet1.end_index, packet2.end_index);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_WRITE_PARTIAL_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_write_partial_list_t packet_c {
         17235, 17339, 17, 84, 151
    };

    mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST packet2{};

    mavlink_msg_mission_write_partial_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.start_index, packet2.start_index);
    EXPECT_EQ(packet_in.end_index, packet2.end_index);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_ITEM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_ITEM packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.seq = 18691;
    packet_in.frame = 235;
    packet_in.command = 18795;
    packet_in.current = 46;
    packet_in.autocontinue = 113;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 129.0;
    packet_in.y = 157.0;
    packet_in.z = 185.0;
    packet_in.mission_type = 180;

    mavlink::common::msg::MISSION_ITEM packet1{};
    mavlink::common::msg::MISSION_ITEM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.current, packet2.current);
    EXPECT_EQ(packet1.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet1.param1, packet2.param1);
    EXPECT_EQ(packet1.param2, packet2.param2);
    EXPECT_EQ(packet1.param3, packet2.param3);
    EXPECT_EQ(packet1.param4, packet2.param4);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_ITEM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_item_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 18691, 18795, 101, 168, 235, 46, 113, 180
    };

    mavlink::common::msg::MISSION_ITEM packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.seq = 18691;
    packet_in.frame = 235;
    packet_in.command = 18795;
    packet_in.current = 46;
    packet_in.autocontinue = 113;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 129.0;
    packet_in.y = 157.0;
    packet_in.z = 185.0;
    packet_in.mission_type = 180;

    mavlink::common::msg::MISSION_ITEM packet2{};

    mavlink_msg_mission_item_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.current, packet2.current);
    EXPECT_EQ(packet_in.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet_in.param1, packet2.param1);
    EXPECT_EQ(packet_in.param2, packet2.param2);
    EXPECT_EQ(packet_in.param3, packet2.param3);
    EXPECT_EQ(packet_in.param4, packet2.param4);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_REQUEST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_REQUEST packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.seq = 17235;
    packet_in.mission_type = 17;

    mavlink::common::msg::MISSION_REQUEST packet1{};
    mavlink::common::msg::MISSION_REQUEST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_REQUEST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_t packet_c {
         17235, 139, 206, 17
    };

    mavlink::common::msg::MISSION_REQUEST packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.seq = 17235;
    packet_in.mission_type = 17;

    mavlink::common::msg::MISSION_REQUEST packet2{};

    mavlink_msg_mission_request_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_SET_CURRENT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_SET_CURRENT packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.seq = 17235;

    mavlink::common::msg::MISSION_SET_CURRENT packet1{};
    mavlink::common::msg::MISSION_SET_CURRENT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_SET_CURRENT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_set_current_t packet_c {
         17235, 139, 206
    };

    mavlink::common::msg::MISSION_SET_CURRENT packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.seq = 17235;

    mavlink::common::msg::MISSION_SET_CURRENT packet2{};

    mavlink_msg_mission_set_current_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_CURRENT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_CURRENT packet_in{};
    packet_in.seq = 17235;
    packet_in.total = 17339;
    packet_in.mission_state = 17;
    packet_in.mission_mode = 84;
    packet_in.mission_id = 963497776;
    packet_in.fence_id = 963497984;
    packet_in.rally_points_id = 963498192;

    mavlink::common::msg::MISSION_CURRENT packet1{};
    mavlink::common::msg::MISSION_CURRENT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.total, packet2.total);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.mission_mode, packet2.mission_mode);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.fence_id, packet2.fence_id);
    EXPECT_EQ(packet1.rally_points_id, packet2.rally_points_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_CURRENT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_current_t packet_c {
         17235, 17339, 17, 84, 963497776, 963497984, 963498192
    };

    mavlink::common::msg::MISSION_CURRENT packet_in{};
    packet_in.seq = 17235;
    packet_in.total = 17339;
    packet_in.mission_state = 17;
    packet_in.mission_mode = 84;
    packet_in.mission_id = 963497776;
    packet_in.fence_id = 963497984;
    packet_in.rally_points_id = 963498192;

    mavlink::common::msg::MISSION_CURRENT packet2{};

    mavlink_msg_mission_current_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.total, packet2.total);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.mission_mode, packet2.mission_mode);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.fence_id, packet2.fence_id);
    EXPECT_EQ(packet_in.rally_points_id, packet2.rally_points_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.mission_type = 139;

    mavlink::common::msg::MISSION_REQUEST_LIST packet1{};
    mavlink::common::msg::MISSION_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_list_t packet_c {
         5, 72, 139
    };

    mavlink::common::msg::MISSION_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.mission_type = 139;

    mavlink::common::msg::MISSION_REQUEST_LIST packet2{};

    mavlink_msg_mission_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_COUNT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_COUNT packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.count = 17235;
    packet_in.mission_type = 17;
    packet_in.opaque_id = 963497724;

    mavlink::common::msg::MISSION_COUNT packet1{};
    mavlink::common::msg::MISSION_COUNT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.count, packet2.count);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.opaque_id, packet2.opaque_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_COUNT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_count_t packet_c {
         17235, 139, 206, 17, 963497724
    };

    mavlink::common::msg::MISSION_COUNT packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.count = 17235;
    packet_in.mission_type = 17;
    packet_in.opaque_id = 963497724;

    mavlink::common::msg::MISSION_COUNT packet2{};

    mavlink_msg_mission_count_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.count, packet2.count);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.opaque_id, packet2.opaque_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_CLEAR_ALL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_CLEAR_ALL packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.mission_type = 139;

    mavlink::common::msg::MISSION_CLEAR_ALL packet1{};
    mavlink::common::msg::MISSION_CLEAR_ALL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_CLEAR_ALL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_clear_all_t packet_c {
         5, 72, 139
    };

    mavlink::common::msg::MISSION_CLEAR_ALL packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.mission_type = 139;

    mavlink::common::msg::MISSION_CLEAR_ALL packet2{};

    mavlink_msg_mission_clear_all_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_ITEM_REACHED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_ITEM_REACHED packet_in{};
    packet_in.seq = 17235;

    mavlink::common::msg::MISSION_ITEM_REACHED packet1{};
    mavlink::common::msg::MISSION_ITEM_REACHED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_ITEM_REACHED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_item_reached_t packet_c {
         17235
    };

    mavlink::common::msg::MISSION_ITEM_REACHED packet_in{};
    packet_in.seq = 17235;

    mavlink::common::msg::MISSION_ITEM_REACHED packet2{};

    mavlink_msg_mission_item_reached_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_ACK packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.type = 139;
    packet_in.mission_type = 206;
    packet_in.opaque_id = 963497672;

    mavlink::common::msg::MISSION_ACK packet1{};
    mavlink::common::msg::MISSION_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.opaque_id, packet2.opaque_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_ack_t packet_c {
         5, 72, 139, 206, 963497672
    };

    mavlink::common::msg::MISSION_ACK packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.type = 139;
    packet_in.mission_type = 206;
    packet_in.opaque_id = 963497672;

    mavlink::common::msg::MISSION_ACK packet2{};

    mavlink_msg_mission_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.opaque_id, packet2.opaque_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_GPS_GLOBAL_ORIGIN)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.target_system = 41;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.time_usec = 93372036854776626ULL;

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet1{};
    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_GPS_GLOBAL_ORIGIN)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_gps_global_origin_t packet_c {
         963497464, 963497672, 963497880, 41, 93372036854776626ULL
    };

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.target_system = 41;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.time_usec = 93372036854776626ULL;

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet2{};

    mavlink_msg_set_gps_global_origin_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_GLOBAL_ORIGIN)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.time_usec = 93372036854776563ULL;

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet1{};
    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_GLOBAL_ORIGIN)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_global_origin_t packet_c {
         963497464, 963497672, 963497880, 93372036854776563ULL
    };

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.time_usec = 93372036854776563ULL;

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet2{};

    mavlink_msg_gps_global_origin_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_MAP_RC)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_MAP_RC packet_in{};
    packet_in.target_system = 187;
    packet_in.target_component = 254;
    packet_in.param_id = to_char_array("UVWXYZABCDEFGHI");
    packet_in.param_index = 18067;
    packet_in.parameter_rc_channel_index = 113;
    packet_in.param_value0 = 17.0;
    packet_in.scale = 45.0;
    packet_in.param_value_min = 73.0;
    packet_in.param_value_max = 101.0;

    mavlink::common::msg::PARAM_MAP_RC packet1{};
    mavlink::common::msg::PARAM_MAP_RC packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
    EXPECT_EQ(packet1.parameter_rc_channel_index, packet2.parameter_rc_channel_index);
    EXPECT_EQ(packet1.param_value0, packet2.param_value0);
    EXPECT_EQ(packet1.scale, packet2.scale);
    EXPECT_EQ(packet1.param_value_min, packet2.param_value_min);
    EXPECT_EQ(packet1.param_value_max, packet2.param_value_max);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_MAP_RC)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_map_rc_t packet_c {
         17.0, 45.0, 73.0, 101.0, 18067, 187, 254, "UVWXYZABCDEFGHI", 113
    };

    mavlink::common::msg::PARAM_MAP_RC packet_in{};
    packet_in.target_system = 187;
    packet_in.target_component = 254;
    packet_in.param_id = to_char_array("UVWXYZABCDEFGHI");
    packet_in.param_index = 18067;
    packet_in.parameter_rc_channel_index = 113;
    packet_in.param_value0 = 17.0;
    packet_in.scale = 45.0;
    packet_in.param_value_min = 73.0;
    packet_in.param_value_max = 101.0;

    mavlink::common::msg::PARAM_MAP_RC packet2{};

    mavlink_msg_param_map_rc_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);
    EXPECT_EQ(packet_in.parameter_rc_channel_index, packet2.parameter_rc_channel_index);
    EXPECT_EQ(packet_in.param_value0, packet2.param_value0);
    EXPECT_EQ(packet_in.scale, packet2.scale);
    EXPECT_EQ(packet_in.param_value_min, packet2.param_value_min);
    EXPECT_EQ(packet_in.param_value_max, packet2.param_value_max);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_REQUEST_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_REQUEST_INT packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.seq = 17235;
    packet_in.mission_type = 17;

    mavlink::common::msg::MISSION_REQUEST_INT packet1{};
    mavlink::common::msg::MISSION_REQUEST_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_REQUEST_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_int_t packet_c {
         17235, 139, 206, 17
    };

    mavlink::common::msg::MISSION_REQUEST_INT packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.seq = 17235;
    packet_in.mission_type = 17;

    mavlink::common::msg::MISSION_REQUEST_INT packet2{};

    mavlink_msg_mission_request_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SAFETY_SET_ALLOWED_AREA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SAFETY_SET_ALLOWED_AREA packet_in{};
    packet_in.target_system = 77;
    packet_in.target_component = 144;
    packet_in.frame = 211;
    packet_in.p1x = 17.0;
    packet_in.p1y = 45.0;
    packet_in.p1z = 73.0;
    packet_in.p2x = 101.0;
    packet_in.p2y = 129.0;
    packet_in.p2z = 157.0;

    mavlink::common::msg::SAFETY_SET_ALLOWED_AREA packet1{};
    mavlink::common::msg::SAFETY_SET_ALLOWED_AREA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.p1x, packet2.p1x);
    EXPECT_EQ(packet1.p1y, packet2.p1y);
    EXPECT_EQ(packet1.p1z, packet2.p1z);
    EXPECT_EQ(packet1.p2x, packet2.p2x);
    EXPECT_EQ(packet1.p2y, packet2.p2y);
    EXPECT_EQ(packet1.p2z, packet2.p2z);
}

#ifdef TEST_INTEROP
TEST(common_interop, SAFETY_SET_ALLOWED_AREA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_safety_set_allowed_area_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 77, 144, 211
    };

    mavlink::common::msg::SAFETY_SET_ALLOWED_AREA packet_in{};
    packet_in.target_system = 77;
    packet_in.target_component = 144;
    packet_in.frame = 211;
    packet_in.p1x = 17.0;
    packet_in.p1y = 45.0;
    packet_in.p1z = 73.0;
    packet_in.p2x = 101.0;
    packet_in.p2y = 129.0;
    packet_in.p2z = 157.0;

    mavlink::common::msg::SAFETY_SET_ALLOWED_AREA packet2{};

    mavlink_msg_safety_set_allowed_area_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.p1x, packet2.p1x);
    EXPECT_EQ(packet_in.p1y, packet2.p1y);
    EXPECT_EQ(packet_in.p1z, packet2.p1z);
    EXPECT_EQ(packet_in.p2x, packet2.p2x);
    EXPECT_EQ(packet_in.p2y, packet2.p2y);
    EXPECT_EQ(packet_in.p2z, packet2.p2z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SAFETY_ALLOWED_AREA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SAFETY_ALLOWED_AREA packet_in{};
    packet_in.frame = 77;
    packet_in.p1x = 17.0;
    packet_in.p1y = 45.0;
    packet_in.p1z = 73.0;
    packet_in.p2x = 101.0;
    packet_in.p2y = 129.0;
    packet_in.p2z = 157.0;

    mavlink::common::msg::SAFETY_ALLOWED_AREA packet1{};
    mavlink::common::msg::SAFETY_ALLOWED_AREA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.p1x, packet2.p1x);
    EXPECT_EQ(packet1.p1y, packet2.p1y);
    EXPECT_EQ(packet1.p1z, packet2.p1z);
    EXPECT_EQ(packet1.p2x, packet2.p2x);
    EXPECT_EQ(packet1.p2y, packet2.p2y);
    EXPECT_EQ(packet1.p2z, packet2.p2z);
}

#ifdef TEST_INTEROP
TEST(common_interop, SAFETY_ALLOWED_AREA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_safety_allowed_area_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 77
    };

    mavlink::common::msg::SAFETY_ALLOWED_AREA packet_in{};
    packet_in.frame = 77;
    packet_in.p1x = 17.0;
    packet_in.p1y = 45.0;
    packet_in.p1z = 73.0;
    packet_in.p2x = 101.0;
    packet_in.p2y = 129.0;
    packet_in.p2z = 157.0;

    mavlink::common::msg::SAFETY_ALLOWED_AREA packet2{};

    mavlink_msg_safety_allowed_area_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.p1x, packet2.p1x);
    EXPECT_EQ(packet_in.p1y, packet2.p1y);
    EXPECT_EQ(packet_in.p1z, packet2.p1z);
    EXPECT_EQ(packet_in.p2x, packet2.p2x);
    EXPECT_EQ(packet_in.p2y, packet2.p2y);
    EXPECT_EQ(packet_in.p2z, packet2.p2z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE_QUATERNION_COV)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE_QUATERNION_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.q = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.rollspeed = 185.0;
    packet_in.pitchspeed = 213.0;
    packet_in.yawspeed = 241.0;
    packet_in.covariance = {{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0 }};

    mavlink::common::msg::ATTITUDE_QUATERNION_COV packet1{};
    mavlink::common::msg::ATTITUDE_QUATERNION_COV packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE_QUATERNION_COV)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_quaternion_cov_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0 }, 185.0, 213.0, 241.0, { 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0 }
    };

    mavlink::common::msg::ATTITUDE_QUATERNION_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.q = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.rollspeed = 185.0;
    packet_in.pitchspeed = 213.0;
    packet_in.yawspeed = 241.0;
    packet_in.covariance = {{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0 }};

    mavlink::common::msg::ATTITUDE_QUATERNION_COV packet2{};

    mavlink_msg_attitude_quaternion_cov_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, NAV_CONTROLLER_OUTPUT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::NAV_CONTROLLER_OUTPUT packet_in{};
    packet_in.nav_roll = 17.0;
    packet_in.nav_pitch = 45.0;
    packet_in.nav_bearing = 18275;
    packet_in.target_bearing = 18379;
    packet_in.wp_dist = 18483;
    packet_in.alt_error = 73.0;
    packet_in.aspd_error = 101.0;
    packet_in.xtrack_error = 129.0;

    mavlink::common::msg::NAV_CONTROLLER_OUTPUT packet1{};
    mavlink::common::msg::NAV_CONTROLLER_OUTPUT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.nav_roll, packet2.nav_roll);
    EXPECT_EQ(packet1.nav_pitch, packet2.nav_pitch);
    EXPECT_EQ(packet1.nav_bearing, packet2.nav_bearing);
    EXPECT_EQ(packet1.target_bearing, packet2.target_bearing);
    EXPECT_EQ(packet1.wp_dist, packet2.wp_dist);
    EXPECT_EQ(packet1.alt_error, packet2.alt_error);
    EXPECT_EQ(packet1.aspd_error, packet2.aspd_error);
    EXPECT_EQ(packet1.xtrack_error, packet2.xtrack_error);
}

#ifdef TEST_INTEROP
TEST(common_interop, NAV_CONTROLLER_OUTPUT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_nav_controller_output_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 18275, 18379, 18483
    };

    mavlink::common::msg::NAV_CONTROLLER_OUTPUT packet_in{};
    packet_in.nav_roll = 17.0;
    packet_in.nav_pitch = 45.0;
    packet_in.nav_bearing = 18275;
    packet_in.target_bearing = 18379;
    packet_in.wp_dist = 18483;
    packet_in.alt_error = 73.0;
    packet_in.aspd_error = 101.0;
    packet_in.xtrack_error = 129.0;

    mavlink::common::msg::NAV_CONTROLLER_OUTPUT packet2{};

    mavlink_msg_nav_controller_output_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.nav_roll, packet2.nav_roll);
    EXPECT_EQ(packet_in.nav_pitch, packet2.nav_pitch);
    EXPECT_EQ(packet_in.nav_bearing, packet2.nav_bearing);
    EXPECT_EQ(packet_in.target_bearing, packet2.target_bearing);
    EXPECT_EQ(packet_in.wp_dist, packet2.wp_dist);
    EXPECT_EQ(packet_in.alt_error, packet2.alt_error);
    EXPECT_EQ(packet_in.aspd_error, packet2.aspd_error);
    EXPECT_EQ(packet_in.xtrack_error, packet2.xtrack_error);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GLOBAL_POSITION_INT_COV)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GLOBAL_POSITION_INT_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.estimator_type = 33;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.relative_alt = 963498504;
    packet_in.vx = 185.0;
    packet_in.vy = 213.0;
    packet_in.vz = 241.0;
    packet_in.covariance = {{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0, 290.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0 }};

    mavlink::common::msg::GLOBAL_POSITION_INT_COV packet1{};
    mavlink::common::msg::GLOBAL_POSITION_INT_COV packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.estimator_type, packet2.estimator_type);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
}

#ifdef TEST_INTEROP
TEST(common_interop, GLOBAL_POSITION_INT_COV)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_global_position_int_cov_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 963498504, 185.0, 213.0, 241.0, { 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0, 290.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0 }, 33
    };

    mavlink::common::msg::GLOBAL_POSITION_INT_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.estimator_type = 33;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.relative_alt = 963498504;
    packet_in.vx = 185.0;
    packet_in.vy = 213.0;
    packet_in.vz = 241.0;
    packet_in.covariance = {{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0, 290.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0 }};

    mavlink::common::msg::GLOBAL_POSITION_INT_COV packet2{};

    mavlink_msg_global_position_int_cov_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.estimator_type, packet2.estimator_type);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOCAL_POSITION_NED_COV)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOCAL_POSITION_NED_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.estimator_type = 165;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.vx = 157.0;
    packet_in.vy = 185.0;
    packet_in.vz = 213.0;
    packet_in.ax = 241.0;
    packet_in.ay = 269.0;
    packet_in.az = 297.0;
    packet_in.covariance = {{ 325.0, 326.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0, 337.0, 338.0, 339.0, 340.0, 341.0, 342.0, 343.0, 344.0, 345.0, 346.0, 347.0, 348.0, 349.0, 350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0, 358.0, 359.0, 360.0, 361.0, 362.0, 363.0, 364.0, 365.0, 366.0, 367.0, 368.0, 369.0 }};

    mavlink::common::msg::LOCAL_POSITION_NED_COV packet1{};
    mavlink::common::msg::LOCAL_POSITION_NED_COV packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.estimator_type, packet2.estimator_type);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.ax, packet2.ax);
    EXPECT_EQ(packet1.ay, packet2.ay);
    EXPECT_EQ(packet1.az, packet2.az);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOCAL_POSITION_NED_COV)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_local_position_ned_cov_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, { 325.0, 326.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0, 337.0, 338.0, 339.0, 340.0, 341.0, 342.0, 343.0, 344.0, 345.0, 346.0, 347.0, 348.0, 349.0, 350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0, 358.0, 359.0, 360.0, 361.0, 362.0, 363.0, 364.0, 365.0, 366.0, 367.0, 368.0, 369.0 }, 165
    };

    mavlink::common::msg::LOCAL_POSITION_NED_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.estimator_type = 165;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.vx = 157.0;
    packet_in.vy = 185.0;
    packet_in.vz = 213.0;
    packet_in.ax = 241.0;
    packet_in.ay = 269.0;
    packet_in.az = 297.0;
    packet_in.covariance = {{ 325.0, 326.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0, 337.0, 338.0, 339.0, 340.0, 341.0, 342.0, 343.0, 344.0, 345.0, 346.0, 347.0, 348.0, 349.0, 350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0, 358.0, 359.0, 360.0, 361.0, 362.0, 363.0, 364.0, 365.0, 366.0, 367.0, 368.0, 369.0 }};

    mavlink::common::msg::LOCAL_POSITION_NED_COV packet2{};

    mavlink_msg_local_position_ned_cov_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.estimator_type, packet2.estimator_type);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.ax, packet2.ax);
    EXPECT_EQ(packet_in.ay, packet2.ay);
    EXPECT_EQ(packet_in.az, packet2.az);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RC_CHANNELS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RC_CHANNELS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.chancount = 125;
    packet_in.chan1_raw = 17443;
    packet_in.chan2_raw = 17547;
    packet_in.chan3_raw = 17651;
    packet_in.chan4_raw = 17755;
    packet_in.chan5_raw = 17859;
    packet_in.chan6_raw = 17963;
    packet_in.chan7_raw = 18067;
    packet_in.chan8_raw = 18171;
    packet_in.chan9_raw = 18275;
    packet_in.chan10_raw = 18379;
    packet_in.chan11_raw = 18483;
    packet_in.chan12_raw = 18587;
    packet_in.chan13_raw = 18691;
    packet_in.chan14_raw = 18795;
    packet_in.chan15_raw = 18899;
    packet_in.chan16_raw = 19003;
    packet_in.chan17_raw = 19107;
    packet_in.chan18_raw = 19211;
    packet_in.rssi = 192;

    mavlink::common::msg::RC_CHANNELS packet1{};
    mavlink::common::msg::RC_CHANNELS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.chancount, packet2.chancount);
    EXPECT_EQ(packet1.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet1.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet1.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet1.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet1.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet1.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet1.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet1.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet1.chan9_raw, packet2.chan9_raw);
    EXPECT_EQ(packet1.chan10_raw, packet2.chan10_raw);
    EXPECT_EQ(packet1.chan11_raw, packet2.chan11_raw);
    EXPECT_EQ(packet1.chan12_raw, packet2.chan12_raw);
    EXPECT_EQ(packet1.chan13_raw, packet2.chan13_raw);
    EXPECT_EQ(packet1.chan14_raw, packet2.chan14_raw);
    EXPECT_EQ(packet1.chan15_raw, packet2.chan15_raw);
    EXPECT_EQ(packet1.chan16_raw, packet2.chan16_raw);
    EXPECT_EQ(packet1.chan17_raw, packet2.chan17_raw);
    EXPECT_EQ(packet1.chan18_raw, packet2.chan18_raw);
    EXPECT_EQ(packet1.rssi, packet2.rssi);
}

#ifdef TEST_INTEROP
TEST(common_interop, RC_CHANNELS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_rc_channels_t packet_c {
         963497464, 17443, 17547, 17651, 17755, 17859, 17963, 18067, 18171, 18275, 18379, 18483, 18587, 18691, 18795, 18899, 19003, 19107, 19211, 125, 192
    };

    mavlink::common::msg::RC_CHANNELS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.chancount = 125;
    packet_in.chan1_raw = 17443;
    packet_in.chan2_raw = 17547;
    packet_in.chan3_raw = 17651;
    packet_in.chan4_raw = 17755;
    packet_in.chan5_raw = 17859;
    packet_in.chan6_raw = 17963;
    packet_in.chan7_raw = 18067;
    packet_in.chan8_raw = 18171;
    packet_in.chan9_raw = 18275;
    packet_in.chan10_raw = 18379;
    packet_in.chan11_raw = 18483;
    packet_in.chan12_raw = 18587;
    packet_in.chan13_raw = 18691;
    packet_in.chan14_raw = 18795;
    packet_in.chan15_raw = 18899;
    packet_in.chan16_raw = 19003;
    packet_in.chan17_raw = 19107;
    packet_in.chan18_raw = 19211;
    packet_in.rssi = 192;

    mavlink::common::msg::RC_CHANNELS packet2{};

    mavlink_msg_rc_channels_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.chancount, packet2.chancount);
    EXPECT_EQ(packet_in.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet_in.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet_in.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet_in.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet_in.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet_in.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet_in.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet_in.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet_in.chan9_raw, packet2.chan9_raw);
    EXPECT_EQ(packet_in.chan10_raw, packet2.chan10_raw);
    EXPECT_EQ(packet_in.chan11_raw, packet2.chan11_raw);
    EXPECT_EQ(packet_in.chan12_raw, packet2.chan12_raw);
    EXPECT_EQ(packet_in.chan13_raw, packet2.chan13_raw);
    EXPECT_EQ(packet_in.chan14_raw, packet2.chan14_raw);
    EXPECT_EQ(packet_in.chan15_raw, packet2.chan15_raw);
    EXPECT_EQ(packet_in.chan16_raw, packet2.chan16_raw);
    EXPECT_EQ(packet_in.chan17_raw, packet2.chan17_raw);
    EXPECT_EQ(packet_in.chan18_raw, packet2.chan18_raw);
    EXPECT_EQ(packet_in.rssi, packet2.rssi);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, REQUEST_DATA_STREAM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::REQUEST_DATA_STREAM packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.req_stream_id = 17;
    packet_in.req_message_rate = 17235;
    packet_in.start_stop = 84;

    mavlink::common::msg::REQUEST_DATA_STREAM packet1{};
    mavlink::common::msg::REQUEST_DATA_STREAM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.req_stream_id, packet2.req_stream_id);
    EXPECT_EQ(packet1.req_message_rate, packet2.req_message_rate);
    EXPECT_EQ(packet1.start_stop, packet2.start_stop);
}

#ifdef TEST_INTEROP
TEST(common_interop, REQUEST_DATA_STREAM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_request_data_stream_t packet_c {
         17235, 139, 206, 17, 84
    };

    mavlink::common::msg::REQUEST_DATA_STREAM packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.req_stream_id = 17;
    packet_in.req_message_rate = 17235;
    packet_in.start_stop = 84;

    mavlink::common::msg::REQUEST_DATA_STREAM packet2{};

    mavlink_msg_request_data_stream_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.req_stream_id, packet2.req_stream_id);
    EXPECT_EQ(packet_in.req_message_rate, packet2.req_message_rate);
    EXPECT_EQ(packet_in.start_stop, packet2.start_stop);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, DATA_STREAM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::DATA_STREAM packet_in{};
    packet_in.stream_id = 139;
    packet_in.message_rate = 17235;
    packet_in.on_off = 206;

    mavlink::common::msg::DATA_STREAM packet1{};
    mavlink::common::msg::DATA_STREAM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.stream_id, packet2.stream_id);
    EXPECT_EQ(packet1.message_rate, packet2.message_rate);
    EXPECT_EQ(packet1.on_off, packet2.on_off);
}

#ifdef TEST_INTEROP
TEST(common_interop, DATA_STREAM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_data_stream_t packet_c {
         17235, 139, 206
    };

    mavlink::common::msg::DATA_STREAM packet_in{};
    packet_in.stream_id = 139;
    packet_in.message_rate = 17235;
    packet_in.on_off = 206;

    mavlink::common::msg::DATA_STREAM packet2{};

    mavlink_msg_data_stream_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.stream_id, packet2.stream_id);
    EXPECT_EQ(packet_in.message_rate, packet2.message_rate);
    EXPECT_EQ(packet_in.on_off, packet2.on_off);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MANUAL_CONTROL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MANUAL_CONTROL packet_in{};
    packet_in.target = 163;
    packet_in.x = 17235;
    packet_in.y = 17339;
    packet_in.z = 17443;
    packet_in.r = 17547;
    packet_in.buttons = 17651;
    packet_in.buttons2 = 17807;
    packet_in.enabled_extensions = 108;
    packet_in.s = 17963;
    packet_in.t = 18067;
    packet_in.aux1 = 18171;
    packet_in.aux2 = 18275;
    packet_in.aux3 = 18379;
    packet_in.aux4 = 18483;
    packet_in.aux5 = 18587;
    packet_in.aux6 = 18691;

    mavlink::common::msg::MANUAL_CONTROL packet1{};
    mavlink::common::msg::MANUAL_CONTROL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target, packet2.target);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.r, packet2.r);
    EXPECT_EQ(packet1.buttons, packet2.buttons);
    EXPECT_EQ(packet1.buttons2, packet2.buttons2);
    EXPECT_EQ(packet1.enabled_extensions, packet2.enabled_extensions);
    EXPECT_EQ(packet1.s, packet2.s);
    EXPECT_EQ(packet1.t, packet2.t);
    EXPECT_EQ(packet1.aux1, packet2.aux1);
    EXPECT_EQ(packet1.aux2, packet2.aux2);
    EXPECT_EQ(packet1.aux3, packet2.aux3);
    EXPECT_EQ(packet1.aux4, packet2.aux4);
    EXPECT_EQ(packet1.aux5, packet2.aux5);
    EXPECT_EQ(packet1.aux6, packet2.aux6);
}

#ifdef TEST_INTEROP
TEST(common_interop, MANUAL_CONTROL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_manual_control_t packet_c {
         17235, 17339, 17443, 17547, 17651, 163, 17807, 108, 17963, 18067, 18171, 18275, 18379, 18483, 18587, 18691
    };

    mavlink::common::msg::MANUAL_CONTROL packet_in{};
    packet_in.target = 163;
    packet_in.x = 17235;
    packet_in.y = 17339;
    packet_in.z = 17443;
    packet_in.r = 17547;
    packet_in.buttons = 17651;
    packet_in.buttons2 = 17807;
    packet_in.enabled_extensions = 108;
    packet_in.s = 17963;
    packet_in.t = 18067;
    packet_in.aux1 = 18171;
    packet_in.aux2 = 18275;
    packet_in.aux3 = 18379;
    packet_in.aux4 = 18483;
    packet_in.aux5 = 18587;
    packet_in.aux6 = 18691;

    mavlink::common::msg::MANUAL_CONTROL packet2{};

    mavlink_msg_manual_control_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target, packet2.target);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.r, packet2.r);
    EXPECT_EQ(packet_in.buttons, packet2.buttons);
    EXPECT_EQ(packet_in.buttons2, packet2.buttons2);
    EXPECT_EQ(packet_in.enabled_extensions, packet2.enabled_extensions);
    EXPECT_EQ(packet_in.s, packet2.s);
    EXPECT_EQ(packet_in.t, packet2.t);
    EXPECT_EQ(packet_in.aux1, packet2.aux1);
    EXPECT_EQ(packet_in.aux2, packet2.aux2);
    EXPECT_EQ(packet_in.aux3, packet2.aux3);
    EXPECT_EQ(packet_in.aux4, packet2.aux4);
    EXPECT_EQ(packet_in.aux5, packet2.aux5);
    EXPECT_EQ(packet_in.aux6, packet2.aux6);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RC_CHANNELS_OVERRIDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RC_CHANNELS_OVERRIDE packet_in{};
    packet_in.target_system = 53;
    packet_in.target_component = 120;
    packet_in.chan1_raw = 17235;
    packet_in.chan2_raw = 17339;
    packet_in.chan3_raw = 17443;
    packet_in.chan4_raw = 17547;
    packet_in.chan5_raw = 17651;
    packet_in.chan6_raw = 17755;
    packet_in.chan7_raw = 17859;
    packet_in.chan8_raw = 17963;
    packet_in.chan9_raw = 18171;
    packet_in.chan10_raw = 18275;
    packet_in.chan11_raw = 18379;
    packet_in.chan12_raw = 18483;
    packet_in.chan13_raw = 18587;
    packet_in.chan14_raw = 18691;
    packet_in.chan15_raw = 18795;
    packet_in.chan16_raw = 18899;
    packet_in.chan17_raw = 19003;
    packet_in.chan18_raw = 19107;

    mavlink::common::msg::RC_CHANNELS_OVERRIDE packet1{};
    mavlink::common::msg::RC_CHANNELS_OVERRIDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet1.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet1.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet1.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet1.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet1.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet1.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet1.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet1.chan9_raw, packet2.chan9_raw);
    EXPECT_EQ(packet1.chan10_raw, packet2.chan10_raw);
    EXPECT_EQ(packet1.chan11_raw, packet2.chan11_raw);
    EXPECT_EQ(packet1.chan12_raw, packet2.chan12_raw);
    EXPECT_EQ(packet1.chan13_raw, packet2.chan13_raw);
    EXPECT_EQ(packet1.chan14_raw, packet2.chan14_raw);
    EXPECT_EQ(packet1.chan15_raw, packet2.chan15_raw);
    EXPECT_EQ(packet1.chan16_raw, packet2.chan16_raw);
    EXPECT_EQ(packet1.chan17_raw, packet2.chan17_raw);
    EXPECT_EQ(packet1.chan18_raw, packet2.chan18_raw);
}

#ifdef TEST_INTEROP
TEST(common_interop, RC_CHANNELS_OVERRIDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_rc_channels_override_t packet_c {
         17235, 17339, 17443, 17547, 17651, 17755, 17859, 17963, 53, 120, 18171, 18275, 18379, 18483, 18587, 18691, 18795, 18899, 19003, 19107
    };

    mavlink::common::msg::RC_CHANNELS_OVERRIDE packet_in{};
    packet_in.target_system = 53;
    packet_in.target_component = 120;
    packet_in.chan1_raw = 17235;
    packet_in.chan2_raw = 17339;
    packet_in.chan3_raw = 17443;
    packet_in.chan4_raw = 17547;
    packet_in.chan5_raw = 17651;
    packet_in.chan6_raw = 17755;
    packet_in.chan7_raw = 17859;
    packet_in.chan8_raw = 17963;
    packet_in.chan9_raw = 18171;
    packet_in.chan10_raw = 18275;
    packet_in.chan11_raw = 18379;
    packet_in.chan12_raw = 18483;
    packet_in.chan13_raw = 18587;
    packet_in.chan14_raw = 18691;
    packet_in.chan15_raw = 18795;
    packet_in.chan16_raw = 18899;
    packet_in.chan17_raw = 19003;
    packet_in.chan18_raw = 19107;

    mavlink::common::msg::RC_CHANNELS_OVERRIDE packet2{};

    mavlink_msg_rc_channels_override_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet_in.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet_in.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet_in.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet_in.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet_in.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet_in.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet_in.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet_in.chan9_raw, packet2.chan9_raw);
    EXPECT_EQ(packet_in.chan10_raw, packet2.chan10_raw);
    EXPECT_EQ(packet_in.chan11_raw, packet2.chan11_raw);
    EXPECT_EQ(packet_in.chan12_raw, packet2.chan12_raw);
    EXPECT_EQ(packet_in.chan13_raw, packet2.chan13_raw);
    EXPECT_EQ(packet_in.chan14_raw, packet2.chan14_raw);
    EXPECT_EQ(packet_in.chan15_raw, packet2.chan15_raw);
    EXPECT_EQ(packet_in.chan16_raw, packet2.chan16_raw);
    EXPECT_EQ(packet_in.chan17_raw, packet2.chan17_raw);
    EXPECT_EQ(packet_in.chan18_raw, packet2.chan18_raw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MISSION_ITEM_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MISSION_ITEM_INT packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.seq = 18691;
    packet_in.frame = 235;
    packet_in.command = 18795;
    packet_in.current = 46;
    packet_in.autocontinue = 113;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 963498296;
    packet_in.y = 963498504;
    packet_in.z = 185.0;
    packet_in.mission_type = 180;

    mavlink::common::msg::MISSION_ITEM_INT packet1{};
    mavlink::common::msg::MISSION_ITEM_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.current, packet2.current);
    EXPECT_EQ(packet1.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet1.param1, packet2.param1);
    EXPECT_EQ(packet1.param2, packet2.param2);
    EXPECT_EQ(packet1.param3, packet2.param3);
    EXPECT_EQ(packet1.param4, packet2.param4);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, MISSION_ITEM_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_item_int_t packet_c {
         17.0, 45.0, 73.0, 101.0, 963498296, 963498504, 185.0, 18691, 18795, 101, 168, 235, 46, 113, 180
    };

    mavlink::common::msg::MISSION_ITEM_INT packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.seq = 18691;
    packet_in.frame = 235;
    packet_in.command = 18795;
    packet_in.current = 46;
    packet_in.autocontinue = 113;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 963498296;
    packet_in.y = 963498504;
    packet_in.z = 185.0;
    packet_in.mission_type = 180;

    mavlink::common::msg::MISSION_ITEM_INT packet2{};

    mavlink_msg_mission_item_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.current, packet2.current);
    EXPECT_EQ(packet_in.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet_in.param1, packet2.param1);
    EXPECT_EQ(packet_in.param2, packet2.param2);
    EXPECT_EQ(packet_in.param3, packet2.param3);
    EXPECT_EQ(packet_in.param4, packet2.param4);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VFR_HUD)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VFR_HUD packet_in{};
    packet_in.airspeed = 17.0;
    packet_in.groundspeed = 45.0;
    packet_in.heading = 18067;
    packet_in.throttle = 18171;
    packet_in.alt = 73.0;
    packet_in.climb = 101.0;

    mavlink::common::msg::VFR_HUD packet1{};
    mavlink::common::msg::VFR_HUD packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.airspeed, packet2.airspeed);
    EXPECT_EQ(packet1.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet1.heading, packet2.heading);
    EXPECT_EQ(packet1.throttle, packet2.throttle);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.climb, packet2.climb);
}

#ifdef TEST_INTEROP
TEST(common_interop, VFR_HUD)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vfr_hud_t packet_c {
         17.0, 45.0, 73.0, 101.0, 18067, 18171
    };

    mavlink::common::msg::VFR_HUD packet_in{};
    packet_in.airspeed = 17.0;
    packet_in.groundspeed = 45.0;
    packet_in.heading = 18067;
    packet_in.throttle = 18171;
    packet_in.alt = 73.0;
    packet_in.climb = 101.0;

    mavlink::common::msg::VFR_HUD packet2{};

    mavlink_msg_vfr_hud_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.airspeed, packet2.airspeed);
    EXPECT_EQ(packet_in.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet_in.heading, packet2.heading);
    EXPECT_EQ(packet_in.throttle, packet2.throttle);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.climb, packet2.climb);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_INT packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.frame = 101;
    packet_in.command = 18691;
    packet_in.current = 168;
    packet_in.autocontinue = 235;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 963498296;
    packet_in.y = 963498504;
    packet_in.z = 185.0;

    mavlink::common::msg::COMMAND_INT packet1{};
    mavlink::common::msg::COMMAND_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.current, packet2.current);
    EXPECT_EQ(packet1.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet1.param1, packet2.param1);
    EXPECT_EQ(packet1.param2, packet2.param2);
    EXPECT_EQ(packet1.param3, packet2.param3);
    EXPECT_EQ(packet1.param4, packet2.param4);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_int_t packet_c {
         17.0, 45.0, 73.0, 101.0, 963498296, 963498504, 185.0, 18691, 223, 34, 101, 168, 235
    };

    mavlink::common::msg::COMMAND_INT packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.frame = 101;
    packet_in.command = 18691;
    packet_in.current = 168;
    packet_in.autocontinue = 235;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 963498296;
    packet_in.y = 963498504;
    packet_in.z = 185.0;

    mavlink::common::msg::COMMAND_INT packet2{};

    mavlink_msg_command_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.current, packet2.current);
    EXPECT_EQ(packet_in.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet_in.param1, packet2.param1);
    EXPECT_EQ(packet_in.param2, packet2.param2);
    EXPECT_EQ(packet_in.param3, packet2.param3);
    EXPECT_EQ(packet_in.param4, packet2.param4);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_LONG)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_LONG packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.command = 18691;
    packet_in.confirmation = 101;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.param5 = 129.0;
    packet_in.param6 = 157.0;
    packet_in.param7 = 185.0;

    mavlink::common::msg::COMMAND_LONG packet1{};
    mavlink::common::msg::COMMAND_LONG packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.confirmation, packet2.confirmation);
    EXPECT_EQ(packet1.param1, packet2.param1);
    EXPECT_EQ(packet1.param2, packet2.param2);
    EXPECT_EQ(packet1.param3, packet2.param3);
    EXPECT_EQ(packet1.param4, packet2.param4);
    EXPECT_EQ(packet1.param5, packet2.param5);
    EXPECT_EQ(packet1.param6, packet2.param6);
    EXPECT_EQ(packet1.param7, packet2.param7);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_LONG)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_long_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 18691, 223, 34, 101
    };

    mavlink::common::msg::COMMAND_LONG packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.command = 18691;
    packet_in.confirmation = 101;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.param5 = 129.0;
    packet_in.param6 = 157.0;
    packet_in.param7 = 185.0;

    mavlink::common::msg::COMMAND_LONG packet2{};

    mavlink_msg_command_long_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.confirmation, packet2.confirmation);
    EXPECT_EQ(packet_in.param1, packet2.param1);
    EXPECT_EQ(packet_in.param2, packet2.param2);
    EXPECT_EQ(packet_in.param3, packet2.param3);
    EXPECT_EQ(packet_in.param4, packet2.param4);
    EXPECT_EQ(packet_in.param5, packet2.param5);
    EXPECT_EQ(packet_in.param6, packet2.param6);
    EXPECT_EQ(packet_in.param7, packet2.param7);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_ACK packet_in{};
    packet_in.command = 17235;
    packet_in.result = 139;
    packet_in.progress = 206;
    packet_in.result_param2 = 963497672;
    packet_in.target_system = 29;
    packet_in.target_component = 96;

    mavlink::common::msg::COMMAND_ACK packet1{};
    mavlink::common::msg::COMMAND_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.result, packet2.result);
    EXPECT_EQ(packet1.progress, packet2.progress);
    EXPECT_EQ(packet1.result_param2, packet2.result_param2);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_ack_t packet_c {
         17235, 139, 206, 963497672, 29, 96
    };

    mavlink::common::msg::COMMAND_ACK packet_in{};
    packet_in.command = 17235;
    packet_in.result = 139;
    packet_in.progress = 206;
    packet_in.result_param2 = 963497672;
    packet_in.target_system = 29;
    packet_in.target_component = 96;

    mavlink::common::msg::COMMAND_ACK packet2{};

    mavlink_msg_command_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.result, packet2.result);
    EXPECT_EQ(packet_in.progress, packet2.progress);
    EXPECT_EQ(packet_in.result_param2, packet2.result_param2);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_CANCEL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_CANCEL packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.command = 17235;

    mavlink::common::msg::COMMAND_CANCEL packet1{};
    mavlink::common::msg::COMMAND_CANCEL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.command, packet2.command);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_CANCEL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_cancel_t packet_c {
         17235, 139, 206
    };

    mavlink::common::msg::COMMAND_CANCEL packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.command = 17235;

    mavlink::common::msg::COMMAND_CANCEL packet2{};

    mavlink_msg_command_cancel_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.command, packet2.command);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MANUAL_SETPOINT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MANUAL_SETPOINT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.thrust = 129.0;
    packet_in.mode_switch = 65;
    packet_in.manual_override_switch = 132;

    mavlink::common::msg::MANUAL_SETPOINT packet1{};
    mavlink::common::msg::MANUAL_SETPOINT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.thrust, packet2.thrust);
    EXPECT_EQ(packet1.mode_switch, packet2.mode_switch);
    EXPECT_EQ(packet1.manual_override_switch, packet2.manual_override_switch);
}

#ifdef TEST_INTEROP
TEST(common_interop, MANUAL_SETPOINT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_manual_setpoint_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 65, 132
    };

    mavlink::common::msg::MANUAL_SETPOINT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.thrust = 129.0;
    packet_in.mode_switch = 65;
    packet_in.manual_override_switch = 132;

    mavlink::common::msg::MANUAL_SETPOINT packet2{};

    mavlink_msg_manual_setpoint_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.thrust, packet2.thrust);
    EXPECT_EQ(packet_in.mode_switch, packet2.mode_switch);
    EXPECT_EQ(packet_in.manual_override_switch, packet2.manual_override_switch);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_ATTITUDE_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_ATTITUDE_TARGET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 113;
    packet_in.target_component = 180;
    packet_in.type_mask = 247;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.body_roll_rate = 157.0;
    packet_in.body_pitch_rate = 185.0;
    packet_in.body_yaw_rate = 213.0;
    packet_in.thrust = 241.0;
    packet_in.thrust_body = {{ 290.0, 291.0, 292.0 }};

    mavlink::common::msg::SET_ATTITUDE_TARGET packet1{};
    mavlink::common::msg::SET_ATTITUDE_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.type_mask, packet2.type_mask);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.body_roll_rate, packet2.body_roll_rate);
    EXPECT_EQ(packet1.body_pitch_rate, packet2.body_pitch_rate);
    EXPECT_EQ(packet1.body_yaw_rate, packet2.body_yaw_rate);
    EXPECT_EQ(packet1.thrust, packet2.thrust);
    EXPECT_EQ(packet1.thrust_body, packet2.thrust_body);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_ATTITUDE_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_attitude_target_t packet_c {
         963497464, { 45.0, 46.0, 47.0, 48.0 }, 157.0, 185.0, 213.0, 241.0, 113, 180, 247, { 290.0, 291.0, 292.0 }
    };

    mavlink::common::msg::SET_ATTITUDE_TARGET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 113;
    packet_in.target_component = 180;
    packet_in.type_mask = 247;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.body_roll_rate = 157.0;
    packet_in.body_pitch_rate = 185.0;
    packet_in.body_yaw_rate = 213.0;
    packet_in.thrust = 241.0;
    packet_in.thrust_body = {{ 290.0, 291.0, 292.0 }};

    mavlink::common::msg::SET_ATTITUDE_TARGET packet2{};

    mavlink_msg_set_attitude_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.type_mask, packet2.type_mask);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.body_roll_rate, packet2.body_roll_rate);
    EXPECT_EQ(packet_in.body_pitch_rate, packet2.body_pitch_rate);
    EXPECT_EQ(packet_in.body_yaw_rate, packet2.body_yaw_rate);
    EXPECT_EQ(packet_in.thrust, packet2.thrust);
    EXPECT_EQ(packet_in.thrust_body, packet2.thrust_body);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE_TARGET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.type_mask = 113;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.body_roll_rate = 157.0;
    packet_in.body_pitch_rate = 185.0;
    packet_in.body_yaw_rate = 213.0;
    packet_in.thrust = 241.0;

    mavlink::common::msg::ATTITUDE_TARGET packet1{};
    mavlink::common::msg::ATTITUDE_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.type_mask, packet2.type_mask);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.body_roll_rate, packet2.body_roll_rate);
    EXPECT_EQ(packet1.body_pitch_rate, packet2.body_pitch_rate);
    EXPECT_EQ(packet1.body_yaw_rate, packet2.body_yaw_rate);
    EXPECT_EQ(packet1.thrust, packet2.thrust);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_target_t packet_c {
         963497464, { 45.0, 46.0, 47.0, 48.0 }, 157.0, 185.0, 213.0, 241.0, 113
    };

    mavlink::common::msg::ATTITUDE_TARGET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.type_mask = 113;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.body_roll_rate = 157.0;
    packet_in.body_pitch_rate = 185.0;
    packet_in.body_yaw_rate = 213.0;
    packet_in.thrust = 241.0;

    mavlink::common::msg::ATTITUDE_TARGET packet2{};

    mavlink_msg_attitude_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.type_mask, packet2.type_mask);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.body_roll_rate, packet2.body_roll_rate);
    EXPECT_EQ(packet_in.body_pitch_rate, packet2.body_pitch_rate);
    EXPECT_EQ(packet_in.body_yaw_rate, packet2.body_yaw_rate);
    EXPECT_EQ(packet_in.thrust, packet2.thrust);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_POSITION_TARGET_LOCAL_NED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 27;
    packet_in.target_component = 94;
    packet_in.coordinate_frame = 161;
    packet_in.type_mask = 19731;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED packet1{};
    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet1.type_mask, packet2.type_mask);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.afx, packet2.afx);
    EXPECT_EQ(packet1.afy, packet2.afy);
    EXPECT_EQ(packet1.afz, packet2.afz);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.yaw_rate, packet2.yaw_rate);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_POSITION_TARGET_LOCAL_NED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_position_target_local_ned_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 19731, 27, 94, 161
    };

    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 27;
    packet_in.target_component = 94;
    packet_in.coordinate_frame = 161;
    packet_in.type_mask = 19731;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED packet2{};

    mavlink_msg_set_position_target_local_ned_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet_in.type_mask, packet2.type_mask);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.afx, packet2.afx);
    EXPECT_EQ(packet_in.afy, packet2.afy);
    EXPECT_EQ(packet_in.afz, packet2.afz);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.yaw_rate, packet2.yaw_rate);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, POSITION_TARGET_LOCAL_NED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::POSITION_TARGET_LOCAL_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.coordinate_frame = 27;
    packet_in.type_mask = 19731;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::POSITION_TARGET_LOCAL_NED packet1{};
    mavlink::common::msg::POSITION_TARGET_LOCAL_NED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet1.type_mask, packet2.type_mask);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.afx, packet2.afx);
    EXPECT_EQ(packet1.afy, packet2.afy);
    EXPECT_EQ(packet1.afz, packet2.afz);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.yaw_rate, packet2.yaw_rate);
}

#ifdef TEST_INTEROP
TEST(common_interop, POSITION_TARGET_LOCAL_NED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_position_target_local_ned_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 19731, 27
    };

    mavlink::common::msg::POSITION_TARGET_LOCAL_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.coordinate_frame = 27;
    packet_in.type_mask = 19731;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::POSITION_TARGET_LOCAL_NED packet2{};

    mavlink_msg_position_target_local_ned_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet_in.type_mask, packet2.type_mask);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.afx, packet2.afx);
    EXPECT_EQ(packet_in.afy, packet2.afy);
    EXPECT_EQ(packet_in.afz, packet2.afz);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.yaw_rate, packet2.yaw_rate);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_POSITION_TARGET_GLOBAL_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 27;
    packet_in.target_component = 94;
    packet_in.coordinate_frame = 161;
    packet_in.type_mask = 19731;
    packet_in.lat_int = 963497672;
    packet_in.lon_int = 963497880;
    packet_in.alt = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT packet1{};
    mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet1.type_mask, packet2.type_mask);
    EXPECT_EQ(packet1.lat_int, packet2.lat_int);
    EXPECT_EQ(packet1.lon_int, packet2.lon_int);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.afx, packet2.afx);
    EXPECT_EQ(packet1.afy, packet2.afy);
    EXPECT_EQ(packet1.afz, packet2.afz);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.yaw_rate, packet2.yaw_rate);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_POSITION_TARGET_GLOBAL_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_position_target_global_int_t packet_c {
         963497464, 963497672, 963497880, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 19731, 27, 94, 161
    };

    mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 27;
    packet_in.target_component = 94;
    packet_in.coordinate_frame = 161;
    packet_in.type_mask = 19731;
    packet_in.lat_int = 963497672;
    packet_in.lon_int = 963497880;
    packet_in.alt = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT packet2{};

    mavlink_msg_set_position_target_global_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet_in.type_mask, packet2.type_mask);
    EXPECT_EQ(packet_in.lat_int, packet2.lat_int);
    EXPECT_EQ(packet_in.lon_int, packet2.lon_int);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.afx, packet2.afx);
    EXPECT_EQ(packet_in.afy, packet2.afy);
    EXPECT_EQ(packet_in.afz, packet2.afz);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.yaw_rate, packet2.yaw_rate);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, POSITION_TARGET_GLOBAL_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::POSITION_TARGET_GLOBAL_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.coordinate_frame = 27;
    packet_in.type_mask = 19731;
    packet_in.lat_int = 963497672;
    packet_in.lon_int = 963497880;
    packet_in.alt = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::POSITION_TARGET_GLOBAL_INT packet1{};
    mavlink::common::msg::POSITION_TARGET_GLOBAL_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet1.type_mask, packet2.type_mask);
    EXPECT_EQ(packet1.lat_int, packet2.lat_int);
    EXPECT_EQ(packet1.lon_int, packet2.lon_int);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.afx, packet2.afx);
    EXPECT_EQ(packet1.afy, packet2.afy);
    EXPECT_EQ(packet1.afz, packet2.afz);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.yaw_rate, packet2.yaw_rate);
}

#ifdef TEST_INTEROP
TEST(common_interop, POSITION_TARGET_GLOBAL_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_position_target_global_int_t packet_c {
         963497464, 963497672, 963497880, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 19731, 27
    };

    mavlink::common::msg::POSITION_TARGET_GLOBAL_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.coordinate_frame = 27;
    packet_in.type_mask = 19731;
    packet_in.lat_int = 963497672;
    packet_in.lon_int = 963497880;
    packet_in.alt = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;
    packet_in.afx = 213.0;
    packet_in.afy = 241.0;
    packet_in.afz = 269.0;
    packet_in.yaw = 297.0;
    packet_in.yaw_rate = 325.0;

    mavlink::common::msg::POSITION_TARGET_GLOBAL_INT packet2{};

    mavlink_msg_position_target_global_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet_in.type_mask, packet2.type_mask);
    EXPECT_EQ(packet_in.lat_int, packet2.lat_int);
    EXPECT_EQ(packet_in.lon_int, packet2.lon_int);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.afx, packet2.afx);
    EXPECT_EQ(packet_in.afy, packet2.afy);
    EXPECT_EQ(packet_in.afz, packet2.afz);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.yaw_rate, packet2.yaw_rate);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;

    mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET packet1{};
    mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_local_position_ned_system_global_offset_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;

    mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET packet2{};

    mavlink_msg_local_position_ned_system_global_offset_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_STATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_STATE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.roll = 73.0;
    packet_in.pitch = 101.0;
    packet_in.yaw = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;
    packet_in.lat = 963499128;
    packet_in.lon = 963499336;
    packet_in.alt = 963499544;
    packet_in.vx = 19523;
    packet_in.vy = 19627;
    packet_in.vz = 19731;
    packet_in.xacc = 19835;
    packet_in.yacc = 19939;
    packet_in.zacc = 20043;

    mavlink::common::msg::HIL_STATE packet1{};
    mavlink::common::msg::HIL_STATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_STATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_state_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 963499128, 963499336, 963499544, 19523, 19627, 19731, 19835, 19939, 20043
    };

    mavlink::common::msg::HIL_STATE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.roll = 73.0;
    packet_in.pitch = 101.0;
    packet_in.yaw = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;
    packet_in.lat = 963499128;
    packet_in.lon = 963499336;
    packet_in.alt = 963499544;
    packet_in.vx = 19523;
    packet_in.vy = 19627;
    packet_in.vz = 19731;
    packet_in.xacc = 19835;
    packet_in.yacc = 19939;
    packet_in.zacc = 20043;

    mavlink::common::msg::HIL_STATE packet2{};

    mavlink_msg_hil_state_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_CONTROLS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_CONTROLS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.roll_ailerons = 73.0;
    packet_in.pitch_elevator = 101.0;
    packet_in.yaw_rudder = 129.0;
    packet_in.throttle = 157.0;
    packet_in.aux1 = 185.0;
    packet_in.aux2 = 213.0;
    packet_in.aux3 = 241.0;
    packet_in.aux4 = 269.0;
    packet_in.mode = 125;
    packet_in.nav_mode = 192;

    mavlink::common::msg::HIL_CONTROLS packet1{};
    mavlink::common::msg::HIL_CONTROLS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.roll_ailerons, packet2.roll_ailerons);
    EXPECT_EQ(packet1.pitch_elevator, packet2.pitch_elevator);
    EXPECT_EQ(packet1.yaw_rudder, packet2.yaw_rudder);
    EXPECT_EQ(packet1.throttle, packet2.throttle);
    EXPECT_EQ(packet1.aux1, packet2.aux1);
    EXPECT_EQ(packet1.aux2, packet2.aux2);
    EXPECT_EQ(packet1.aux3, packet2.aux3);
    EXPECT_EQ(packet1.aux4, packet2.aux4);
    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.nav_mode, packet2.nav_mode);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_CONTROLS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_controls_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 125, 192
    };

    mavlink::common::msg::HIL_CONTROLS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.roll_ailerons = 73.0;
    packet_in.pitch_elevator = 101.0;
    packet_in.yaw_rudder = 129.0;
    packet_in.throttle = 157.0;
    packet_in.aux1 = 185.0;
    packet_in.aux2 = 213.0;
    packet_in.aux3 = 241.0;
    packet_in.aux4 = 269.0;
    packet_in.mode = 125;
    packet_in.nav_mode = 192;

    mavlink::common::msg::HIL_CONTROLS packet2{};

    mavlink_msg_hil_controls_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.roll_ailerons, packet2.roll_ailerons);
    EXPECT_EQ(packet_in.pitch_elevator, packet2.pitch_elevator);
    EXPECT_EQ(packet_in.yaw_rudder, packet2.yaw_rudder);
    EXPECT_EQ(packet_in.throttle, packet2.throttle);
    EXPECT_EQ(packet_in.aux1, packet2.aux1);
    EXPECT_EQ(packet_in.aux2, packet2.aux2);
    EXPECT_EQ(packet_in.aux3, packet2.aux3);
    EXPECT_EQ(packet_in.aux4, packet2.aux4);
    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.nav_mode, packet2.nav_mode);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_RC_INPUTS_RAW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_RC_INPUTS_RAW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.chan1_raw = 17651;
    packet_in.chan2_raw = 17755;
    packet_in.chan3_raw = 17859;
    packet_in.chan4_raw = 17963;
    packet_in.chan5_raw = 18067;
    packet_in.chan6_raw = 18171;
    packet_in.chan7_raw = 18275;
    packet_in.chan8_raw = 18379;
    packet_in.chan9_raw = 18483;
    packet_in.chan10_raw = 18587;
    packet_in.chan11_raw = 18691;
    packet_in.chan12_raw = 18795;
    packet_in.rssi = 101;

    mavlink::common::msg::HIL_RC_INPUTS_RAW packet1{};
    mavlink::common::msg::HIL_RC_INPUTS_RAW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet1.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet1.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet1.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet1.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet1.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet1.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet1.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet1.chan9_raw, packet2.chan9_raw);
    EXPECT_EQ(packet1.chan10_raw, packet2.chan10_raw);
    EXPECT_EQ(packet1.chan11_raw, packet2.chan11_raw);
    EXPECT_EQ(packet1.chan12_raw, packet2.chan12_raw);
    EXPECT_EQ(packet1.rssi, packet2.rssi);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_RC_INPUTS_RAW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_rc_inputs_raw_t packet_c {
         93372036854775807ULL, 17651, 17755, 17859, 17963, 18067, 18171, 18275, 18379, 18483, 18587, 18691, 18795, 101
    };

    mavlink::common::msg::HIL_RC_INPUTS_RAW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.chan1_raw = 17651;
    packet_in.chan2_raw = 17755;
    packet_in.chan3_raw = 17859;
    packet_in.chan4_raw = 17963;
    packet_in.chan5_raw = 18067;
    packet_in.chan6_raw = 18171;
    packet_in.chan7_raw = 18275;
    packet_in.chan8_raw = 18379;
    packet_in.chan9_raw = 18483;
    packet_in.chan10_raw = 18587;
    packet_in.chan11_raw = 18691;
    packet_in.chan12_raw = 18795;
    packet_in.rssi = 101;

    mavlink::common::msg::HIL_RC_INPUTS_RAW packet2{};

    mavlink_msg_hil_rc_inputs_raw_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.chan1_raw, packet2.chan1_raw);
    EXPECT_EQ(packet_in.chan2_raw, packet2.chan2_raw);
    EXPECT_EQ(packet_in.chan3_raw, packet2.chan3_raw);
    EXPECT_EQ(packet_in.chan4_raw, packet2.chan4_raw);
    EXPECT_EQ(packet_in.chan5_raw, packet2.chan5_raw);
    EXPECT_EQ(packet_in.chan6_raw, packet2.chan6_raw);
    EXPECT_EQ(packet_in.chan7_raw, packet2.chan7_raw);
    EXPECT_EQ(packet_in.chan8_raw, packet2.chan8_raw);
    EXPECT_EQ(packet_in.chan9_raw, packet2.chan9_raw);
    EXPECT_EQ(packet_in.chan10_raw, packet2.chan10_raw);
    EXPECT_EQ(packet_in.chan11_raw, packet2.chan11_raw);
    EXPECT_EQ(packet_in.chan12_raw, packet2.chan12_raw);
    EXPECT_EQ(packet_in.rssi, packet2.rssi);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_ACTUATOR_CONTROLS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_ACTUATOR_CONTROLS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.controls = {{ 129.0, 130.0, 131.0, 132.0, 133.0, 134.0, 135.0, 136.0, 137.0, 138.0, 139.0, 140.0, 141.0, 142.0, 143.0, 144.0 }};
    packet_in.mode = 245;
    packet_in.flags = 93372036854776311ULL;

    mavlink::common::msg::HIL_ACTUATOR_CONTROLS packet1{};
    mavlink::common::msg::HIL_ACTUATOR_CONTROLS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.controls, packet2.controls);
    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.flags, packet2.flags);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_ACTUATOR_CONTROLS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_actuator_controls_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, { 129.0, 130.0, 131.0, 132.0, 133.0, 134.0, 135.0, 136.0, 137.0, 138.0, 139.0, 140.0, 141.0, 142.0, 143.0, 144.0 }, 245
    };

    mavlink::common::msg::HIL_ACTUATOR_CONTROLS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.controls = {{ 129.0, 130.0, 131.0, 132.0, 133.0, 134.0, 135.0, 136.0, 137.0, 138.0, 139.0, 140.0, 141.0, 142.0, 143.0, 144.0 }};
    packet_in.mode = 245;
    packet_in.flags = 93372036854776311ULL;

    mavlink::common::msg::HIL_ACTUATOR_CONTROLS packet2{};

    mavlink_msg_hil_actuator_controls_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.controls, packet2.controls);
    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.flags, packet2.flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPTICAL_FLOW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPTICAL_FLOW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 77;
    packet_in.flow_x = 18275;
    packet_in.flow_y = 18379;
    packet_in.flow_comp_m_x = 73.0;
    packet_in.flow_comp_m_y = 101.0;
    packet_in.quality = 144;
    packet_in.ground_distance = 129.0;
    packet_in.flow_rate_x = 199.0;
    packet_in.flow_rate_y = 227.0;

    mavlink::common::msg::OPTICAL_FLOW packet1{};
    mavlink::common::msg::OPTICAL_FLOW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet1.flow_x, packet2.flow_x);
    EXPECT_EQ(packet1.flow_y, packet2.flow_y);
    EXPECT_EQ(packet1.flow_comp_m_x, packet2.flow_comp_m_x);
    EXPECT_EQ(packet1.flow_comp_m_y, packet2.flow_comp_m_y);
    EXPECT_EQ(packet1.quality, packet2.quality);
    EXPECT_EQ(packet1.ground_distance, packet2.ground_distance);
    EXPECT_EQ(packet1.flow_rate_x, packet2.flow_rate_x);
    EXPECT_EQ(packet1.flow_rate_y, packet2.flow_rate_y);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPTICAL_FLOW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_optical_flow_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 18275, 18379, 77, 144, 199.0, 227.0
    };

    mavlink::common::msg::OPTICAL_FLOW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 77;
    packet_in.flow_x = 18275;
    packet_in.flow_y = 18379;
    packet_in.flow_comp_m_x = 73.0;
    packet_in.flow_comp_m_y = 101.0;
    packet_in.quality = 144;
    packet_in.ground_distance = 129.0;
    packet_in.flow_rate_x = 199.0;
    packet_in.flow_rate_y = 227.0;

    mavlink::common::msg::OPTICAL_FLOW packet2{};

    mavlink_msg_optical_flow_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet_in.flow_x, packet2.flow_x);
    EXPECT_EQ(packet_in.flow_y, packet2.flow_y);
    EXPECT_EQ(packet_in.flow_comp_m_x, packet2.flow_comp_m_x);
    EXPECT_EQ(packet_in.flow_comp_m_y, packet2.flow_comp_m_y);
    EXPECT_EQ(packet_in.quality, packet2.quality);
    EXPECT_EQ(packet_in.ground_distance, packet2.ground_distance);
    EXPECT_EQ(packet_in.flow_rate_x, packet2.flow_rate_x);
    EXPECT_EQ(packet_in.flow_rate_y, packet2.flow_rate_y);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GLOBAL_VISION_POSITION_ESTIMATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GLOBAL_VISION_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;
    packet_in.covariance = {{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }};
    packet_in.reset_counter = 97;

    mavlink::common::msg::GLOBAL_VISION_POSITION_ESTIMATE packet1{};
    mavlink::common::msg::GLOBAL_VISION_POSITION_ESTIMATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.usec, packet2.usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
    EXPECT_EQ(packet1.reset_counter, packet2.reset_counter);
}

#ifdef TEST_INTEROP
TEST(common_interop, GLOBAL_VISION_POSITION_ESTIMATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_global_vision_position_estimate_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, { 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }, 97
    };

    mavlink::common::msg::GLOBAL_VISION_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;
    packet_in.covariance = {{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }};
    packet_in.reset_counter = 97;

    mavlink::common::msg::GLOBAL_VISION_POSITION_ESTIMATE packet2{};

    mavlink_msg_global_vision_position_estimate_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.usec, packet2.usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);
    EXPECT_EQ(packet_in.reset_counter, packet2.reset_counter);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VISION_POSITION_ESTIMATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VISION_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;
    packet_in.covariance = {{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }};
    packet_in.reset_counter = 97;

    mavlink::common::msg::VISION_POSITION_ESTIMATE packet1{};
    mavlink::common::msg::VISION_POSITION_ESTIMATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.usec, packet2.usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
    EXPECT_EQ(packet1.reset_counter, packet2.reset_counter);
}

#ifdef TEST_INTEROP
TEST(common_interop, VISION_POSITION_ESTIMATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vision_position_estimate_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, { 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }, 97
    };

    mavlink::common::msg::VISION_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;
    packet_in.covariance = {{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }};
    packet_in.reset_counter = 97;

    mavlink::common::msg::VISION_POSITION_ESTIMATE packet2{};

    mavlink_msg_vision_position_estimate_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.usec, packet2.usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);
    EXPECT_EQ(packet_in.reset_counter, packet2.reset_counter);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VISION_SPEED_ESTIMATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VISION_SPEED_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.covariance = {{ 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0 }};
    packet_in.reset_counter = 173;

    mavlink::common::msg::VISION_SPEED_ESTIMATE packet1{};
    mavlink::common::msg::VISION_SPEED_ESTIMATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.usec, packet2.usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
    EXPECT_EQ(packet1.reset_counter, packet2.reset_counter);
}

#ifdef TEST_INTEROP
TEST(common_interop, VISION_SPEED_ESTIMATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vision_speed_estimate_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, { 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0 }, 173
    };

    mavlink::common::msg::VISION_SPEED_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.covariance = {{ 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0 }};
    packet_in.reset_counter = 173;

    mavlink::common::msg::VISION_SPEED_ESTIMATE packet2{};

    mavlink_msg_vision_speed_estimate_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.usec, packet2.usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);
    EXPECT_EQ(packet_in.reset_counter, packet2.reset_counter);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VICON_POSITION_ESTIMATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VICON_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;
    packet_in.covariance = {{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }};

    mavlink::common::msg::VICON_POSITION_ESTIMATE packet1{};
    mavlink::common::msg::VICON_POSITION_ESTIMATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.usec, packet2.usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
}

#ifdef TEST_INTEROP
TEST(common_interop, VICON_POSITION_ESTIMATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vicon_position_estimate_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, { 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }
    };

    mavlink::common::msg::VICON_POSITION_ESTIMATE packet_in{};
    packet_in.usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.roll = 157.0;
    packet_in.pitch = 185.0;
    packet_in.yaw = 213.0;
    packet_in.covariance = {{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }};

    mavlink::common::msg::VICON_POSITION_ESTIMATE packet2{};

    mavlink_msg_vicon_position_estimate_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.usec, packet2.usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIGHRES_IMU)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIGHRES_IMU packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 73.0;
    packet_in.yacc = 101.0;
    packet_in.zacc = 129.0;
    packet_in.xgyro = 157.0;
    packet_in.ygyro = 185.0;
    packet_in.zgyro = 213.0;
    packet_in.xmag = 241.0;
    packet_in.ymag = 269.0;
    packet_in.zmag = 297.0;
    packet_in.abs_pressure = 325.0;
    packet_in.diff_pressure = 353.0;
    packet_in.pressure_alt = 381.0;
    packet_in.temperature = 409.0;
    packet_in.fields_updated = 20355;
    packet_in.id = 63;

    mavlink::common::msg::HIGHRES_IMU packet1{};
    mavlink::common::msg::HIGHRES_IMU packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.xmag, packet2.xmag);
    EXPECT_EQ(packet1.ymag, packet2.ymag);
    EXPECT_EQ(packet1.zmag, packet2.zmag);
    EXPECT_EQ(packet1.abs_pressure, packet2.abs_pressure);
    EXPECT_EQ(packet1.diff_pressure, packet2.diff_pressure);
    EXPECT_EQ(packet1.pressure_alt, packet2.pressure_alt);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.fields_updated, packet2.fields_updated);
    EXPECT_EQ(packet1.id, packet2.id);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIGHRES_IMU)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_highres_imu_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0, 381.0, 409.0, 20355, 63
    };

    mavlink::common::msg::HIGHRES_IMU packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 73.0;
    packet_in.yacc = 101.0;
    packet_in.zacc = 129.0;
    packet_in.xgyro = 157.0;
    packet_in.ygyro = 185.0;
    packet_in.zgyro = 213.0;
    packet_in.xmag = 241.0;
    packet_in.ymag = 269.0;
    packet_in.zmag = 297.0;
    packet_in.abs_pressure = 325.0;
    packet_in.diff_pressure = 353.0;
    packet_in.pressure_alt = 381.0;
    packet_in.temperature = 409.0;
    packet_in.fields_updated = 20355;
    packet_in.id = 63;

    mavlink::common::msg::HIGHRES_IMU packet2{};

    mavlink_msg_highres_imu_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.xmag, packet2.xmag);
    EXPECT_EQ(packet_in.ymag, packet2.ymag);
    EXPECT_EQ(packet_in.zmag, packet2.zmag);
    EXPECT_EQ(packet_in.abs_pressure, packet2.abs_pressure);
    EXPECT_EQ(packet_in.diff_pressure, packet2.diff_pressure);
    EXPECT_EQ(packet_in.pressure_alt, packet2.pressure_alt);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.fields_updated, packet2.fields_updated);
    EXPECT_EQ(packet_in.id, packet2.id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPTICAL_FLOW_RAD)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPTICAL_FLOW_RAD packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 3;
    packet_in.integration_time_us = 963497880;
    packet_in.integrated_x = 101.0;
    packet_in.integrated_y = 129.0;
    packet_in.integrated_xgyro = 157.0;
    packet_in.integrated_ygyro = 185.0;
    packet_in.integrated_zgyro = 213.0;
    packet_in.temperature = 19315;
    packet_in.quality = 70;
    packet_in.time_delta_distance_us = 963499128;
    packet_in.distance = 269.0;

    mavlink::common::msg::OPTICAL_FLOW_RAD packet1{};
    mavlink::common::msg::OPTICAL_FLOW_RAD packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet1.integration_time_us, packet2.integration_time_us);
    EXPECT_EQ(packet1.integrated_x, packet2.integrated_x);
    EXPECT_EQ(packet1.integrated_y, packet2.integrated_y);
    EXPECT_EQ(packet1.integrated_xgyro, packet2.integrated_xgyro);
    EXPECT_EQ(packet1.integrated_ygyro, packet2.integrated_ygyro);
    EXPECT_EQ(packet1.integrated_zgyro, packet2.integrated_zgyro);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.quality, packet2.quality);
    EXPECT_EQ(packet1.time_delta_distance_us, packet2.time_delta_distance_us);
    EXPECT_EQ(packet1.distance, packet2.distance);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPTICAL_FLOW_RAD)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_optical_flow_rad_t packet_c {
         93372036854775807ULL, 963497880, 101.0, 129.0, 157.0, 185.0, 213.0, 963499128, 269.0, 19315, 3, 70
    };

    mavlink::common::msg::OPTICAL_FLOW_RAD packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 3;
    packet_in.integration_time_us = 963497880;
    packet_in.integrated_x = 101.0;
    packet_in.integrated_y = 129.0;
    packet_in.integrated_xgyro = 157.0;
    packet_in.integrated_ygyro = 185.0;
    packet_in.integrated_zgyro = 213.0;
    packet_in.temperature = 19315;
    packet_in.quality = 70;
    packet_in.time_delta_distance_us = 963499128;
    packet_in.distance = 269.0;

    mavlink::common::msg::OPTICAL_FLOW_RAD packet2{};

    mavlink_msg_optical_flow_rad_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet_in.integration_time_us, packet2.integration_time_us);
    EXPECT_EQ(packet_in.integrated_x, packet2.integrated_x);
    EXPECT_EQ(packet_in.integrated_y, packet2.integrated_y);
    EXPECT_EQ(packet_in.integrated_xgyro, packet2.integrated_xgyro);
    EXPECT_EQ(packet_in.integrated_ygyro, packet2.integrated_ygyro);
    EXPECT_EQ(packet_in.integrated_zgyro, packet2.integrated_zgyro);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.quality, packet2.quality);
    EXPECT_EQ(packet_in.time_delta_distance_us, packet2.time_delta_distance_us);
    EXPECT_EQ(packet_in.distance, packet2.distance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_SENSOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_SENSOR packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 73.0;
    packet_in.yacc = 101.0;
    packet_in.zacc = 129.0;
    packet_in.xgyro = 157.0;
    packet_in.ygyro = 185.0;
    packet_in.zgyro = 213.0;
    packet_in.xmag = 241.0;
    packet_in.ymag = 269.0;
    packet_in.zmag = 297.0;
    packet_in.abs_pressure = 325.0;
    packet_in.diff_pressure = 353.0;
    packet_in.pressure_alt = 381.0;
    packet_in.temperature = 409.0;
    packet_in.fields_updated = 963500584;
    packet_in.id = 197;

    mavlink::common::msg::HIL_SENSOR packet1{};
    mavlink::common::msg::HIL_SENSOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.xmag, packet2.xmag);
    EXPECT_EQ(packet1.ymag, packet2.ymag);
    EXPECT_EQ(packet1.zmag, packet2.zmag);
    EXPECT_EQ(packet1.abs_pressure, packet2.abs_pressure);
    EXPECT_EQ(packet1.diff_pressure, packet2.diff_pressure);
    EXPECT_EQ(packet1.pressure_alt, packet2.pressure_alt);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.fields_updated, packet2.fields_updated);
    EXPECT_EQ(packet1.id, packet2.id);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_SENSOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_sensor_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0, 381.0, 409.0, 963500584, 197
    };

    mavlink::common::msg::HIL_SENSOR packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 73.0;
    packet_in.yacc = 101.0;
    packet_in.zacc = 129.0;
    packet_in.xgyro = 157.0;
    packet_in.ygyro = 185.0;
    packet_in.zgyro = 213.0;
    packet_in.xmag = 241.0;
    packet_in.ymag = 269.0;
    packet_in.zmag = 297.0;
    packet_in.abs_pressure = 325.0;
    packet_in.diff_pressure = 353.0;
    packet_in.pressure_alt = 381.0;
    packet_in.temperature = 409.0;
    packet_in.fields_updated = 963500584;
    packet_in.id = 197;

    mavlink::common::msg::HIL_SENSOR packet2{};

    mavlink_msg_hil_sensor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.xmag, packet2.xmag);
    EXPECT_EQ(packet_in.ymag, packet2.ymag);
    EXPECT_EQ(packet_in.zmag, packet2.zmag);
    EXPECT_EQ(packet_in.abs_pressure, packet2.abs_pressure);
    EXPECT_EQ(packet_in.diff_pressure, packet2.diff_pressure);
    EXPECT_EQ(packet_in.pressure_alt, packet2.pressure_alt);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.fields_updated, packet2.fields_updated);
    EXPECT_EQ(packet_in.id, packet2.id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SIM_STATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SIM_STATE packet_in{};
    packet_in.q1 = 17.0;
    packet_in.q2 = 45.0;
    packet_in.q3 = 73.0;
    packet_in.q4 = 101.0;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;
    packet_in.xacc = 213.0;
    packet_in.yacc = 241.0;
    packet_in.zacc = 269.0;
    packet_in.xgyro = 297.0;
    packet_in.ygyro = 325.0;
    packet_in.zgyro = 353.0;
    packet_in.lat = 381.0;
    packet_in.lon = 409.0;
    packet_in.alt = 437.0;
    packet_in.std_dev_horz = 465.0;
    packet_in.std_dev_vert = 493.0;
    packet_in.vn = 521.0;
    packet_in.ve = 549.0;
    packet_in.vd = 577.0;
    packet_in.lat_int = 963501832;
    packet_in.lon_int = 963502040;

    mavlink::common::msg::SIM_STATE packet1{};
    mavlink::common::msg::SIM_STATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.q1, packet2.q1);
    EXPECT_EQ(packet1.q2, packet2.q2);
    EXPECT_EQ(packet1.q3, packet2.q3);
    EXPECT_EQ(packet1.q4, packet2.q4);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.std_dev_horz, packet2.std_dev_horz);
    EXPECT_EQ(packet1.std_dev_vert, packet2.std_dev_vert);
    EXPECT_EQ(packet1.vn, packet2.vn);
    EXPECT_EQ(packet1.ve, packet2.ve);
    EXPECT_EQ(packet1.vd, packet2.vd);
    EXPECT_EQ(packet1.lat_int, packet2.lat_int);
    EXPECT_EQ(packet1.lon_int, packet2.lon_int);
}

#ifdef TEST_INTEROP
TEST(common_interop, SIM_STATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_sim_state_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0, 381.0, 409.0, 437.0, 465.0, 493.0, 521.0, 549.0, 577.0, 963501832, 963502040
    };

    mavlink::common::msg::SIM_STATE packet_in{};
    packet_in.q1 = 17.0;
    packet_in.q2 = 45.0;
    packet_in.q3 = 73.0;
    packet_in.q4 = 101.0;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;
    packet_in.xacc = 213.0;
    packet_in.yacc = 241.0;
    packet_in.zacc = 269.0;
    packet_in.xgyro = 297.0;
    packet_in.ygyro = 325.0;
    packet_in.zgyro = 353.0;
    packet_in.lat = 381.0;
    packet_in.lon = 409.0;
    packet_in.alt = 437.0;
    packet_in.std_dev_horz = 465.0;
    packet_in.std_dev_vert = 493.0;
    packet_in.vn = 521.0;
    packet_in.ve = 549.0;
    packet_in.vd = 577.0;
    packet_in.lat_int = 963501832;
    packet_in.lon_int = 963502040;

    mavlink::common::msg::SIM_STATE packet2{};

    mavlink_msg_sim_state_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.q1, packet2.q1);
    EXPECT_EQ(packet_in.q2, packet2.q2);
    EXPECT_EQ(packet_in.q3, packet2.q3);
    EXPECT_EQ(packet_in.q4, packet2.q4);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.std_dev_horz, packet2.std_dev_horz);
    EXPECT_EQ(packet_in.std_dev_vert, packet2.std_dev_vert);
    EXPECT_EQ(packet_in.vn, packet2.vn);
    EXPECT_EQ(packet_in.ve, packet2.ve);
    EXPECT_EQ(packet_in.vd, packet2.vd);
    EXPECT_EQ(packet_in.lat_int, packet2.lat_int);
    EXPECT_EQ(packet_in.lon_int, packet2.lon_int);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RADIO_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RADIO_STATUS packet_in{};
    packet_in.rssi = 17;
    packet_in.remrssi = 84;
    packet_in.txbuf = 151;
    packet_in.noise = 218;
    packet_in.remnoise = 29;
    packet_in.rxerrors = 17235;
    packet_in.fixed = 17339;

    mavlink::common::msg::RADIO_STATUS packet1{};
    mavlink::common::msg::RADIO_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.rssi, packet2.rssi);
    EXPECT_EQ(packet1.remrssi, packet2.remrssi);
    EXPECT_EQ(packet1.txbuf, packet2.txbuf);
    EXPECT_EQ(packet1.noise, packet2.noise);
    EXPECT_EQ(packet1.remnoise, packet2.remnoise);
    EXPECT_EQ(packet1.rxerrors, packet2.rxerrors);
    EXPECT_EQ(packet1.fixed, packet2.fixed);
}

#ifdef TEST_INTEROP
TEST(common_interop, RADIO_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_radio_status_t packet_c {
         17235, 17339, 17, 84, 151, 218, 29
    };

    mavlink::common::msg::RADIO_STATUS packet_in{};
    packet_in.rssi = 17;
    packet_in.remrssi = 84;
    packet_in.txbuf = 151;
    packet_in.noise = 218;
    packet_in.remnoise = 29;
    packet_in.rxerrors = 17235;
    packet_in.fixed = 17339;

    mavlink::common::msg::RADIO_STATUS packet2{};

    mavlink_msg_radio_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.rssi, packet2.rssi);
    EXPECT_EQ(packet_in.remrssi, packet2.remrssi);
    EXPECT_EQ(packet_in.txbuf, packet2.txbuf);
    EXPECT_EQ(packet_in.noise, packet2.noise);
    EXPECT_EQ(packet_in.remnoise, packet2.remnoise);
    EXPECT_EQ(packet_in.rxerrors, packet2.rxerrors);
    EXPECT_EQ(packet_in.fixed, packet2.fixed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, FILE_TRANSFER_PROTOCOL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::FILE_TRANSFER_PROTOCOL packet_in{};
    packet_in.target_network = 5;
    packet_in.target_system = 72;
    packet_in.target_component = 139;
    packet_in.payload = {{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200 }};

    mavlink::common::msg::FILE_TRANSFER_PROTOCOL packet1{};
    mavlink::common::msg::FILE_TRANSFER_PROTOCOL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_network, packet2.target_network);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.payload, packet2.payload);
}

#ifdef TEST_INTEROP
TEST(common_interop, FILE_TRANSFER_PROTOCOL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_file_transfer_protocol_t packet_c {
         5, 72, 139, { 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200 }
    };

    mavlink::common::msg::FILE_TRANSFER_PROTOCOL packet_in{};
    packet_in.target_network = 5;
    packet_in.target_system = 72;
    packet_in.target_component = 139;
    packet_in.payload = {{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200 }};

    mavlink::common::msg::FILE_TRANSFER_PROTOCOL packet2{};

    mavlink_msg_file_transfer_protocol_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_network, packet2.target_network);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.payload, packet2.payload);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TIMESYNC)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TIMESYNC packet_in{};
    packet_in.tc1 = 93372036854775807LL;
    packet_in.ts1 = 170LL;
    packet_in.target_system = 53;
    packet_in.target_component = 120;

    mavlink::common::msg::TIMESYNC packet1{};
    mavlink::common::msg::TIMESYNC packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.tc1, packet2.tc1);
    EXPECT_EQ(packet1.ts1, packet2.ts1);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, TIMESYNC)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_timesync_t packet_c {
         93372036854775807LL, 170LL, 53, 120
    };

    mavlink::common::msg::TIMESYNC packet_in{};
    packet_in.tc1 = 93372036854775807LL;
    packet_in.ts1 = 170LL;
    packet_in.target_system = 53;
    packet_in.target_component = 120;

    mavlink::common::msg::TIMESYNC packet2{};

    mavlink_msg_timesync_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.tc1, packet2.tc1);
    EXPECT_EQ(packet_in.ts1, packet2.ts1);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_TRIGGER)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_TRIGGER packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;

    mavlink::common::msg::CAMERA_TRIGGER packet1{};
    mavlink::common::msg::CAMERA_TRIGGER packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_TRIGGER)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_trigger_t packet_c {
         93372036854775807ULL, 963497880
    };

    mavlink::common::msg::CAMERA_TRIGGER packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;

    mavlink::common::msg::CAMERA_TRIGGER packet2{};

    mavlink_msg_camera_trigger_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_GPS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_GPS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 235;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18275;
    packet_in.epv = 18379;
    packet_in.vel = 18483;
    packet_in.vn = 18587;
    packet_in.ve = 18691;
    packet_in.vd = 18795;
    packet_in.cog = 18899;
    packet_in.satellites_visible = 46;
    packet_in.id = 113;
    packet_in.yaw = 19159;

    mavlink::common::msg::HIL_GPS packet1{};
    mavlink::common::msg::HIL_GPS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.fix_type, packet2.fix_type);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.eph, packet2.eph);
    EXPECT_EQ(packet1.epv, packet2.epv);
    EXPECT_EQ(packet1.vel, packet2.vel);
    EXPECT_EQ(packet1.vn, packet2.vn);
    EXPECT_EQ(packet1.ve, packet2.ve);
    EXPECT_EQ(packet1.vd, packet2.vd);
    EXPECT_EQ(packet1.cog, packet2.cog);
    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_GPS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_gps_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 18275, 18379, 18483, 18587, 18691, 18795, 18899, 235, 46, 113, 19159
    };

    mavlink::common::msg::HIL_GPS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 235;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18275;
    packet_in.epv = 18379;
    packet_in.vel = 18483;
    packet_in.vn = 18587;
    packet_in.ve = 18691;
    packet_in.vd = 18795;
    packet_in.cog = 18899;
    packet_in.satellites_visible = 46;
    packet_in.id = 113;
    packet_in.yaw = 19159;

    mavlink::common::msg::HIL_GPS packet2{};

    mavlink_msg_hil_gps_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.fix_type, packet2.fix_type);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.eph, packet2.eph);
    EXPECT_EQ(packet_in.epv, packet2.epv);
    EXPECT_EQ(packet_in.vel, packet2.vel);
    EXPECT_EQ(packet_in.vn, packet2.vn);
    EXPECT_EQ(packet_in.ve, packet2.ve);
    EXPECT_EQ(packet_in.vd, packet2.vd);
    EXPECT_EQ(packet_in.cog, packet2.cog);
    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_OPTICAL_FLOW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_OPTICAL_FLOW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 3;
    packet_in.integration_time_us = 963497880;
    packet_in.integrated_x = 101.0;
    packet_in.integrated_y = 129.0;
    packet_in.integrated_xgyro = 157.0;
    packet_in.integrated_ygyro = 185.0;
    packet_in.integrated_zgyro = 213.0;
    packet_in.temperature = 19315;
    packet_in.quality = 70;
    packet_in.time_delta_distance_us = 963499128;
    packet_in.distance = 269.0;

    mavlink::common::msg::HIL_OPTICAL_FLOW packet1{};
    mavlink::common::msg::HIL_OPTICAL_FLOW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet1.integration_time_us, packet2.integration_time_us);
    EXPECT_EQ(packet1.integrated_x, packet2.integrated_x);
    EXPECT_EQ(packet1.integrated_y, packet2.integrated_y);
    EXPECT_EQ(packet1.integrated_xgyro, packet2.integrated_xgyro);
    EXPECT_EQ(packet1.integrated_ygyro, packet2.integrated_ygyro);
    EXPECT_EQ(packet1.integrated_zgyro, packet2.integrated_zgyro);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.quality, packet2.quality);
    EXPECT_EQ(packet1.time_delta_distance_us, packet2.time_delta_distance_us);
    EXPECT_EQ(packet1.distance, packet2.distance);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_OPTICAL_FLOW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_optical_flow_t packet_c {
         93372036854775807ULL, 963497880, 101.0, 129.0, 157.0, 185.0, 213.0, 963499128, 269.0, 19315, 3, 70
    };

    mavlink::common::msg::HIL_OPTICAL_FLOW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_id = 3;
    packet_in.integration_time_us = 963497880;
    packet_in.integrated_x = 101.0;
    packet_in.integrated_y = 129.0;
    packet_in.integrated_xgyro = 157.0;
    packet_in.integrated_ygyro = 185.0;
    packet_in.integrated_zgyro = 213.0;
    packet_in.temperature = 19315;
    packet_in.quality = 70;
    packet_in.time_delta_distance_us = 963499128;
    packet_in.distance = 269.0;

    mavlink::common::msg::HIL_OPTICAL_FLOW packet2{};

    mavlink_msg_hil_optical_flow_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.sensor_id, packet2.sensor_id);
    EXPECT_EQ(packet_in.integration_time_us, packet2.integration_time_us);
    EXPECT_EQ(packet_in.integrated_x, packet2.integrated_x);
    EXPECT_EQ(packet_in.integrated_y, packet2.integrated_y);
    EXPECT_EQ(packet_in.integrated_xgyro, packet2.integrated_xgyro);
    EXPECT_EQ(packet_in.integrated_ygyro, packet2.integrated_ygyro);
    EXPECT_EQ(packet_in.integrated_zgyro, packet2.integrated_zgyro);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.quality, packet2.quality);
    EXPECT_EQ(packet_in.time_delta_distance_us, packet2.time_delta_distance_us);
    EXPECT_EQ(packet_in.distance, packet2.distance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIL_STATE_QUATERNION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIL_STATE_QUATERNION packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.attitude_quaternion = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.rollspeed = 185.0;
    packet_in.pitchspeed = 213.0;
    packet_in.yawspeed = 241.0;
    packet_in.lat = 963499336;
    packet_in.lon = 963499544;
    packet_in.alt = 963499752;
    packet_in.vx = 19731;
    packet_in.vy = 19835;
    packet_in.vz = 19939;
    packet_in.ind_airspeed = 20043;
    packet_in.true_airspeed = 20147;
    packet_in.xacc = 20251;
    packet_in.yacc = 20355;
    packet_in.zacc = 20459;

    mavlink::common::msg::HIL_STATE_QUATERNION packet1{};
    mavlink::common::msg::HIL_STATE_QUATERNION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.attitude_quaternion, packet2.attitude_quaternion);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.ind_airspeed, packet2.ind_airspeed);
    EXPECT_EQ(packet1.true_airspeed, packet2.true_airspeed);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIL_STATE_QUATERNION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hil_state_quaternion_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0 }, 185.0, 213.0, 241.0, 963499336, 963499544, 963499752, 19731, 19835, 19939, 20043, 20147, 20251, 20355, 20459
    };

    mavlink::common::msg::HIL_STATE_QUATERNION packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.attitude_quaternion = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.rollspeed = 185.0;
    packet_in.pitchspeed = 213.0;
    packet_in.yawspeed = 241.0;
    packet_in.lat = 963499336;
    packet_in.lon = 963499544;
    packet_in.alt = 963499752;
    packet_in.vx = 19731;
    packet_in.vy = 19835;
    packet_in.vz = 19939;
    packet_in.ind_airspeed = 20043;
    packet_in.true_airspeed = 20147;
    packet_in.xacc = 20251;
    packet_in.yacc = 20355;
    packet_in.zacc = 20459;

    mavlink::common::msg::HIL_STATE_QUATERNION packet2{};

    mavlink_msg_hil_state_quaternion_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.attitude_quaternion, packet2.attitude_quaternion);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.ind_airspeed, packet2.ind_airspeed);
    EXPECT_EQ(packet_in.true_airspeed, packet2.true_airspeed);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SCALED_IMU2)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SCALED_IMU2 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.xacc = 17443;
    packet_in.yacc = 17547;
    packet_in.zacc = 17651;
    packet_in.xgyro = 17755;
    packet_in.ygyro = 17859;
    packet_in.zgyro = 17963;
    packet_in.xmag = 18067;
    packet_in.ymag = 18171;
    packet_in.zmag = 18275;
    packet_in.temperature = 18379;

    mavlink::common::msg::SCALED_IMU2 packet1{};
    mavlink::common::msg::SCALED_IMU2 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.xmag, packet2.xmag);
    EXPECT_EQ(packet1.ymag, packet2.ymag);
    EXPECT_EQ(packet1.zmag, packet2.zmag);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
}

#ifdef TEST_INTEROP
TEST(common_interop, SCALED_IMU2)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_scaled_imu2_t packet_c {
         963497464, 17443, 17547, 17651, 17755, 17859, 17963, 18067, 18171, 18275, 18379
    };

    mavlink::common::msg::SCALED_IMU2 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.xacc = 17443;
    packet_in.yacc = 17547;
    packet_in.zacc = 17651;
    packet_in.xgyro = 17755;
    packet_in.ygyro = 17859;
    packet_in.zgyro = 17963;
    packet_in.xmag = 18067;
    packet_in.ymag = 18171;
    packet_in.zmag = 18275;
    packet_in.temperature = 18379;

    mavlink::common::msg::SCALED_IMU2 packet2{};

    mavlink_msg_scaled_imu2_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.xmag, packet2.xmag);
    EXPECT_EQ(packet_in.ymag, packet2.ymag);
    EXPECT_EQ(packet_in.zmag, packet2.zmag);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOG_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOG_REQUEST_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start = 17235;
    packet_in.end = 17339;

    mavlink::common::msg::LOG_REQUEST_LIST packet1{};
    mavlink::common::msg::LOG_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.start, packet2.start);
    EXPECT_EQ(packet1.end, packet2.end);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOG_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_log_request_list_t packet_c {
         17235, 17339, 17, 84
    };

    mavlink::common::msg::LOG_REQUEST_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start = 17235;
    packet_in.end = 17339;

    mavlink::common::msg::LOG_REQUEST_LIST packet2{};

    mavlink_msg_log_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.start, packet2.start);
    EXPECT_EQ(packet_in.end, packet2.end);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOG_ENTRY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOG_ENTRY packet_in{};
    packet_in.id = 17651;
    packet_in.num_logs = 17755;
    packet_in.last_log_num = 17859;
    packet_in.time_utc = 963497464;
    packet_in.size = 963497672;

    mavlink::common::msg::LOG_ENTRY packet1{};
    mavlink::common::msg::LOG_ENTRY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.num_logs, packet2.num_logs);
    EXPECT_EQ(packet1.last_log_num, packet2.last_log_num);
    EXPECT_EQ(packet1.time_utc, packet2.time_utc);
    EXPECT_EQ(packet1.size, packet2.size);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOG_ENTRY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_log_entry_t packet_c {
         963497464, 963497672, 17651, 17755, 17859
    };

    mavlink::common::msg::LOG_ENTRY packet_in{};
    packet_in.id = 17651;
    packet_in.num_logs = 17755;
    packet_in.last_log_num = 17859;
    packet_in.time_utc = 963497464;
    packet_in.size = 963497672;

    mavlink::common::msg::LOG_ENTRY packet2{};

    mavlink_msg_log_entry_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.num_logs, packet2.num_logs);
    EXPECT_EQ(packet_in.last_log_num, packet2.last_log_num);
    EXPECT_EQ(packet_in.time_utc, packet2.time_utc);
    EXPECT_EQ(packet_in.size, packet2.size);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOG_REQUEST_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOG_REQUEST_DATA packet_in{};
    packet_in.target_system = 163;
    packet_in.target_component = 230;
    packet_in.id = 17651;
    packet_in.ofs = 963497464;
    packet_in.count = 963497672;

    mavlink::common::msg::LOG_REQUEST_DATA packet1{};
    mavlink::common::msg::LOG_REQUEST_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.ofs, packet2.ofs);
    EXPECT_EQ(packet1.count, packet2.count);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOG_REQUEST_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_log_request_data_t packet_c {
         963497464, 963497672, 17651, 163, 230
    };

    mavlink::common::msg::LOG_REQUEST_DATA packet_in{};
    packet_in.target_system = 163;
    packet_in.target_component = 230;
    packet_in.id = 17651;
    packet_in.ofs = 963497464;
    packet_in.count = 963497672;

    mavlink::common::msg::LOG_REQUEST_DATA packet2{};

    mavlink_msg_log_request_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.ofs, packet2.ofs);
    EXPECT_EQ(packet_in.count, packet2.count);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOG_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOG_DATA packet_in{};
    packet_in.id = 17443;
    packet_in.ofs = 963497464;
    packet_in.count = 151;
    packet_in.data = {{ 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51 }};

    mavlink::common::msg::LOG_DATA packet1{};
    mavlink::common::msg::LOG_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.ofs, packet2.ofs);
    EXPECT_EQ(packet1.count, packet2.count);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOG_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_log_data_t packet_c {
         963497464, 17443, 151, { 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51 }
    };

    mavlink::common::msg::LOG_DATA packet_in{};
    packet_in.id = 17443;
    packet_in.ofs = 963497464;
    packet_in.count = 151;
    packet_in.data = {{ 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51 }};

    mavlink::common::msg::LOG_DATA packet2{};

    mavlink_msg_log_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.ofs, packet2.ofs);
    EXPECT_EQ(packet_in.count, packet2.count);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOG_ERASE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOG_ERASE packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::LOG_ERASE packet1{};
    mavlink::common::msg::LOG_ERASE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOG_ERASE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_log_erase_t packet_c {
         5, 72
    };

    mavlink::common::msg::LOG_ERASE packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::LOG_ERASE packet2{};

    mavlink_msg_log_erase_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOG_REQUEST_END)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOG_REQUEST_END packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::LOG_REQUEST_END packet1{};
    mavlink::common::msg::LOG_REQUEST_END packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOG_REQUEST_END)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_log_request_end_t packet_c {
         5, 72
    };

    mavlink::common::msg::LOG_REQUEST_END packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::LOG_REQUEST_END packet2{};

    mavlink_msg_log_request_end_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_INJECT_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_INJECT_DATA packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.len = 139;
    packet_in.data = {{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 }};

    mavlink::common::msg::GPS_INJECT_DATA packet1{};
    mavlink::common::msg::GPS_INJECT_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.len, packet2.len);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_INJECT_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_inject_data_t packet_c {
         5, 72, 139, { 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 }
    };

    mavlink::common::msg::GPS_INJECT_DATA packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.len = 139;
    packet_in.data = {{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 }};

    mavlink::common::msg::GPS_INJECT_DATA packet2{};

    mavlink_msg_gps_inject_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.len, packet2.len);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS2_RAW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS2_RAW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 101;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18483;
    packet_in.epv = 18587;
    packet_in.vel = 18691;
    packet_in.cog = 18795;
    packet_in.satellites_visible = 168;
    packet_in.dgps_numch = 235;
    packet_in.dgps_age = 963498504;
    packet_in.yaw = 19055;
    packet_in.alt_ellipsoid = 963499388;
    packet_in.h_acc = 963499596;
    packet_in.v_acc = 963499804;
    packet_in.vel_acc = 963500012;
    packet_in.hdg_acc = 963500220;

    mavlink::common::msg::GPS2_RAW packet1{};
    mavlink::common::msg::GPS2_RAW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.fix_type, packet2.fix_type);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.eph, packet2.eph);
    EXPECT_EQ(packet1.epv, packet2.epv);
    EXPECT_EQ(packet1.vel, packet2.vel);
    EXPECT_EQ(packet1.cog, packet2.cog);
    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet1.dgps_numch, packet2.dgps_numch);
    EXPECT_EQ(packet1.dgps_age, packet2.dgps_age);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.alt_ellipsoid, packet2.alt_ellipsoid);
    EXPECT_EQ(packet1.h_acc, packet2.h_acc);
    EXPECT_EQ(packet1.v_acc, packet2.v_acc);
    EXPECT_EQ(packet1.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet1.hdg_acc, packet2.hdg_acc);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS2_RAW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps2_raw_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 963498504, 18483, 18587, 18691, 18795, 101, 168, 235, 19055, 963499388, 963499596, 963499804, 963500012, 963500220
    };

    mavlink::common::msg::GPS2_RAW packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 101;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18483;
    packet_in.epv = 18587;
    packet_in.vel = 18691;
    packet_in.cog = 18795;
    packet_in.satellites_visible = 168;
    packet_in.dgps_numch = 235;
    packet_in.dgps_age = 963498504;
    packet_in.yaw = 19055;
    packet_in.alt_ellipsoid = 963499388;
    packet_in.h_acc = 963499596;
    packet_in.v_acc = 963499804;
    packet_in.vel_acc = 963500012;
    packet_in.hdg_acc = 963500220;

    mavlink::common::msg::GPS2_RAW packet2{};

    mavlink_msg_gps2_raw_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.fix_type, packet2.fix_type);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.eph, packet2.eph);
    EXPECT_EQ(packet_in.epv, packet2.epv);
    EXPECT_EQ(packet_in.vel, packet2.vel);
    EXPECT_EQ(packet_in.cog, packet2.cog);
    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet_in.dgps_numch, packet2.dgps_numch);
    EXPECT_EQ(packet_in.dgps_age, packet2.dgps_age);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.alt_ellipsoid, packet2.alt_ellipsoid);
    EXPECT_EQ(packet_in.h_acc, packet2.h_acc);
    EXPECT_EQ(packet_in.v_acc, packet2.v_acc);
    EXPECT_EQ(packet_in.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet_in.hdg_acc, packet2.hdg_acc);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, POWER_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::POWER_STATUS packet_in{};
    packet_in.Vcc = 17235;
    packet_in.Vservo = 17339;
    packet_in.flags = 17443;

    mavlink::common::msg::POWER_STATUS packet1{};
    mavlink::common::msg::POWER_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.Vcc, packet2.Vcc);
    EXPECT_EQ(packet1.Vservo, packet2.Vservo);
    EXPECT_EQ(packet1.flags, packet2.flags);
}

#ifdef TEST_INTEROP
TEST(common_interop, POWER_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_power_status_t packet_c {
         17235, 17339, 17443
    };

    mavlink::common::msg::POWER_STATUS packet_in{};
    packet_in.Vcc = 17235;
    packet_in.Vservo = 17339;
    packet_in.flags = 17443;

    mavlink::common::msg::POWER_STATUS packet2{};

    mavlink_msg_power_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.Vcc, packet2.Vcc);
    EXPECT_EQ(packet_in.Vservo, packet2.Vservo);
    EXPECT_EQ(packet_in.flags, packet2.flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SERIAL_CONTROL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SERIAL_CONTROL packet_in{};
    packet_in.device = 151;
    packet_in.flags = 218;
    packet_in.timeout = 17443;
    packet_in.baudrate = 963497464;
    packet_in.count = 29;
    packet_in.data = {{ 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165 }};
    packet_in.target_system = 178;
    packet_in.target_component = 245;

    mavlink::common::msg::SERIAL_CONTROL packet1{};
    mavlink::common::msg::SERIAL_CONTROL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.device, packet2.device);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.timeout, packet2.timeout);
    EXPECT_EQ(packet1.baudrate, packet2.baudrate);
    EXPECT_EQ(packet1.count, packet2.count);
    EXPECT_EQ(packet1.data, packet2.data);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, SERIAL_CONTROL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_serial_control_t packet_c {
         963497464, 17443, 151, 218, 29, { 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165 }, 178, 245
    };

    mavlink::common::msg::SERIAL_CONTROL packet_in{};
    packet_in.device = 151;
    packet_in.flags = 218;
    packet_in.timeout = 17443;
    packet_in.baudrate = 963497464;
    packet_in.count = 29;
    packet_in.data = {{ 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165 }};
    packet_in.target_system = 178;
    packet_in.target_component = 245;

    mavlink::common::msg::SERIAL_CONTROL packet2{};

    mavlink_msg_serial_control_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.device, packet2.device);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.timeout, packet2.timeout);
    EXPECT_EQ(packet_in.baudrate, packet2.baudrate);
    EXPECT_EQ(packet_in.count, packet2.count);
    EXPECT_EQ(packet_in.data, packet2.data);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_RTK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_RTK packet_in{};
    packet_in.time_last_baseline_ms = 963497464;
    packet_in.rtk_receiver_id = 223;
    packet_in.wn = 18691;
    packet_in.tow = 963497672;
    packet_in.rtk_health = 34;
    packet_in.rtk_rate = 101;
    packet_in.nsats = 168;
    packet_in.baseline_coords_type = 235;
    packet_in.baseline_a_mm = 963497880;
    packet_in.baseline_b_mm = 963498088;
    packet_in.baseline_c_mm = 963498296;
    packet_in.accuracy = 963498504;
    packet_in.iar_num_hypotheses = 963498712;

    mavlink::common::msg::GPS_RTK packet1{};
    mavlink::common::msg::GPS_RTK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_last_baseline_ms, packet2.time_last_baseline_ms);
    EXPECT_EQ(packet1.rtk_receiver_id, packet2.rtk_receiver_id);
    EXPECT_EQ(packet1.wn, packet2.wn);
    EXPECT_EQ(packet1.tow, packet2.tow);
    EXPECT_EQ(packet1.rtk_health, packet2.rtk_health);
    EXPECT_EQ(packet1.rtk_rate, packet2.rtk_rate);
    EXPECT_EQ(packet1.nsats, packet2.nsats);
    EXPECT_EQ(packet1.baseline_coords_type, packet2.baseline_coords_type);
    EXPECT_EQ(packet1.baseline_a_mm, packet2.baseline_a_mm);
    EXPECT_EQ(packet1.baseline_b_mm, packet2.baseline_b_mm);
    EXPECT_EQ(packet1.baseline_c_mm, packet2.baseline_c_mm);
    EXPECT_EQ(packet1.accuracy, packet2.accuracy);
    EXPECT_EQ(packet1.iar_num_hypotheses, packet2.iar_num_hypotheses);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_RTK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_rtk_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296, 963498504, 963498712, 18691, 223, 34, 101, 168, 235
    };

    mavlink::common::msg::GPS_RTK packet_in{};
    packet_in.time_last_baseline_ms = 963497464;
    packet_in.rtk_receiver_id = 223;
    packet_in.wn = 18691;
    packet_in.tow = 963497672;
    packet_in.rtk_health = 34;
    packet_in.rtk_rate = 101;
    packet_in.nsats = 168;
    packet_in.baseline_coords_type = 235;
    packet_in.baseline_a_mm = 963497880;
    packet_in.baseline_b_mm = 963498088;
    packet_in.baseline_c_mm = 963498296;
    packet_in.accuracy = 963498504;
    packet_in.iar_num_hypotheses = 963498712;

    mavlink::common::msg::GPS_RTK packet2{};

    mavlink_msg_gps_rtk_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_last_baseline_ms, packet2.time_last_baseline_ms);
    EXPECT_EQ(packet_in.rtk_receiver_id, packet2.rtk_receiver_id);
    EXPECT_EQ(packet_in.wn, packet2.wn);
    EXPECT_EQ(packet_in.tow, packet2.tow);
    EXPECT_EQ(packet_in.rtk_health, packet2.rtk_health);
    EXPECT_EQ(packet_in.rtk_rate, packet2.rtk_rate);
    EXPECT_EQ(packet_in.nsats, packet2.nsats);
    EXPECT_EQ(packet_in.baseline_coords_type, packet2.baseline_coords_type);
    EXPECT_EQ(packet_in.baseline_a_mm, packet2.baseline_a_mm);
    EXPECT_EQ(packet_in.baseline_b_mm, packet2.baseline_b_mm);
    EXPECT_EQ(packet_in.baseline_c_mm, packet2.baseline_c_mm);
    EXPECT_EQ(packet_in.accuracy, packet2.accuracy);
    EXPECT_EQ(packet_in.iar_num_hypotheses, packet2.iar_num_hypotheses);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS2_RTK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS2_RTK packet_in{};
    packet_in.time_last_baseline_ms = 963497464;
    packet_in.rtk_receiver_id = 223;
    packet_in.wn = 18691;
    packet_in.tow = 963497672;
    packet_in.rtk_health = 34;
    packet_in.rtk_rate = 101;
    packet_in.nsats = 168;
    packet_in.baseline_coords_type = 235;
    packet_in.baseline_a_mm = 963497880;
    packet_in.baseline_b_mm = 963498088;
    packet_in.baseline_c_mm = 963498296;
    packet_in.accuracy = 963498504;
    packet_in.iar_num_hypotheses = 963498712;

    mavlink::common::msg::GPS2_RTK packet1{};
    mavlink::common::msg::GPS2_RTK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_last_baseline_ms, packet2.time_last_baseline_ms);
    EXPECT_EQ(packet1.rtk_receiver_id, packet2.rtk_receiver_id);
    EXPECT_EQ(packet1.wn, packet2.wn);
    EXPECT_EQ(packet1.tow, packet2.tow);
    EXPECT_EQ(packet1.rtk_health, packet2.rtk_health);
    EXPECT_EQ(packet1.rtk_rate, packet2.rtk_rate);
    EXPECT_EQ(packet1.nsats, packet2.nsats);
    EXPECT_EQ(packet1.baseline_coords_type, packet2.baseline_coords_type);
    EXPECT_EQ(packet1.baseline_a_mm, packet2.baseline_a_mm);
    EXPECT_EQ(packet1.baseline_b_mm, packet2.baseline_b_mm);
    EXPECT_EQ(packet1.baseline_c_mm, packet2.baseline_c_mm);
    EXPECT_EQ(packet1.accuracy, packet2.accuracy);
    EXPECT_EQ(packet1.iar_num_hypotheses, packet2.iar_num_hypotheses);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS2_RTK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps2_rtk_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296, 963498504, 963498712, 18691, 223, 34, 101, 168, 235
    };

    mavlink::common::msg::GPS2_RTK packet_in{};
    packet_in.time_last_baseline_ms = 963497464;
    packet_in.rtk_receiver_id = 223;
    packet_in.wn = 18691;
    packet_in.tow = 963497672;
    packet_in.rtk_health = 34;
    packet_in.rtk_rate = 101;
    packet_in.nsats = 168;
    packet_in.baseline_coords_type = 235;
    packet_in.baseline_a_mm = 963497880;
    packet_in.baseline_b_mm = 963498088;
    packet_in.baseline_c_mm = 963498296;
    packet_in.accuracy = 963498504;
    packet_in.iar_num_hypotheses = 963498712;

    mavlink::common::msg::GPS2_RTK packet2{};

    mavlink_msg_gps2_rtk_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_last_baseline_ms, packet2.time_last_baseline_ms);
    EXPECT_EQ(packet_in.rtk_receiver_id, packet2.rtk_receiver_id);
    EXPECT_EQ(packet_in.wn, packet2.wn);
    EXPECT_EQ(packet_in.tow, packet2.tow);
    EXPECT_EQ(packet_in.rtk_health, packet2.rtk_health);
    EXPECT_EQ(packet_in.rtk_rate, packet2.rtk_rate);
    EXPECT_EQ(packet_in.nsats, packet2.nsats);
    EXPECT_EQ(packet_in.baseline_coords_type, packet2.baseline_coords_type);
    EXPECT_EQ(packet_in.baseline_a_mm, packet2.baseline_a_mm);
    EXPECT_EQ(packet_in.baseline_b_mm, packet2.baseline_b_mm);
    EXPECT_EQ(packet_in.baseline_c_mm, packet2.baseline_c_mm);
    EXPECT_EQ(packet_in.accuracy, packet2.accuracy);
    EXPECT_EQ(packet_in.iar_num_hypotheses, packet2.iar_num_hypotheses);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SCALED_IMU3)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SCALED_IMU3 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.xacc = 17443;
    packet_in.yacc = 17547;
    packet_in.zacc = 17651;
    packet_in.xgyro = 17755;
    packet_in.ygyro = 17859;
    packet_in.zgyro = 17963;
    packet_in.xmag = 18067;
    packet_in.ymag = 18171;
    packet_in.zmag = 18275;
    packet_in.temperature = 18379;

    mavlink::common::msg::SCALED_IMU3 packet1{};
    mavlink::common::msg::SCALED_IMU3 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.xmag, packet2.xmag);
    EXPECT_EQ(packet1.ymag, packet2.ymag);
    EXPECT_EQ(packet1.zmag, packet2.zmag);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
}

#ifdef TEST_INTEROP
TEST(common_interop, SCALED_IMU3)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_scaled_imu3_t packet_c {
         963497464, 17443, 17547, 17651, 17755, 17859, 17963, 18067, 18171, 18275, 18379
    };

    mavlink::common::msg::SCALED_IMU3 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.xacc = 17443;
    packet_in.yacc = 17547;
    packet_in.zacc = 17651;
    packet_in.xgyro = 17755;
    packet_in.ygyro = 17859;
    packet_in.zgyro = 17963;
    packet_in.xmag = 18067;
    packet_in.ymag = 18171;
    packet_in.zmag = 18275;
    packet_in.temperature = 18379;

    mavlink::common::msg::SCALED_IMU3 packet2{};

    mavlink_msg_scaled_imu3_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.xmag, packet2.xmag);
    EXPECT_EQ(packet_in.ymag, packet2.ymag);
    EXPECT_EQ(packet_in.zmag, packet2.zmag);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, DATA_TRANSMISSION_HANDSHAKE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::DATA_TRANSMISSION_HANDSHAKE packet_in{};
    packet_in.type = 163;
    packet_in.size = 963497464;
    packet_in.width = 17443;
    packet_in.height = 17547;
    packet_in.packets = 17651;
    packet_in.payload = 230;
    packet_in.jpg_quality = 41;

    mavlink::common::msg::DATA_TRANSMISSION_HANDSHAKE packet1{};
    mavlink::common::msg::DATA_TRANSMISSION_HANDSHAKE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.size, packet2.size);
    EXPECT_EQ(packet1.width, packet2.width);
    EXPECT_EQ(packet1.height, packet2.height);
    EXPECT_EQ(packet1.packets, packet2.packets);
    EXPECT_EQ(packet1.payload, packet2.payload);
    EXPECT_EQ(packet1.jpg_quality, packet2.jpg_quality);
}

#ifdef TEST_INTEROP
TEST(common_interop, DATA_TRANSMISSION_HANDSHAKE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_data_transmission_handshake_t packet_c {
         963497464, 17443, 17547, 17651, 163, 230, 41
    };

    mavlink::common::msg::DATA_TRANSMISSION_HANDSHAKE packet_in{};
    packet_in.type = 163;
    packet_in.size = 963497464;
    packet_in.width = 17443;
    packet_in.height = 17547;
    packet_in.packets = 17651;
    packet_in.payload = 230;
    packet_in.jpg_quality = 41;

    mavlink::common::msg::DATA_TRANSMISSION_HANDSHAKE packet2{};

    mavlink_msg_data_transmission_handshake_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.size, packet2.size);
    EXPECT_EQ(packet_in.width, packet2.width);
    EXPECT_EQ(packet_in.height, packet2.height);
    EXPECT_EQ(packet_in.packets, packet2.packets);
    EXPECT_EQ(packet_in.payload, packet2.payload);
    EXPECT_EQ(packet_in.jpg_quality, packet2.jpg_quality);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ENCAPSULATED_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ENCAPSULATED_DATA packet_in{};
    packet_in.seqnr = 17235;
    packet_in.data = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135 }};

    mavlink::common::msg::ENCAPSULATED_DATA packet1{};
    mavlink::common::msg::ENCAPSULATED_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.seqnr, packet2.seqnr);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, ENCAPSULATED_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_encapsulated_data_t packet_c {
         17235, { 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135 }
    };

    mavlink::common::msg::ENCAPSULATED_DATA packet_in{};
    packet_in.seqnr = 17235;
    packet_in.data = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135 }};

    mavlink::common::msg::ENCAPSULATED_DATA packet2{};

    mavlink_msg_encapsulated_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.seqnr, packet2.seqnr);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, DISTANCE_SENSOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::DISTANCE_SENSOR packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.min_distance = 17443;
    packet_in.max_distance = 17547;
    packet_in.current_distance = 17651;
    packet_in.type = 163;
    packet_in.id = 230;
    packet_in.orientation = 41;
    packet_in.covariance = 108;
    packet_in.horizontal_fov = 115.0;
    packet_in.vertical_fov = 143.0;
    packet_in.quaternion = {{ 171.0, 172.0, 173.0, 174.0 }};
    packet_in.signal_quality = 247;

    mavlink::common::msg::DISTANCE_SENSOR packet1{};
    mavlink::common::msg::DISTANCE_SENSOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.min_distance, packet2.min_distance);
    EXPECT_EQ(packet1.max_distance, packet2.max_distance);
    EXPECT_EQ(packet1.current_distance, packet2.current_distance);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.orientation, packet2.orientation);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
    EXPECT_EQ(packet1.horizontal_fov, packet2.horizontal_fov);
    EXPECT_EQ(packet1.vertical_fov, packet2.vertical_fov);
    EXPECT_EQ(packet1.quaternion, packet2.quaternion);
    EXPECT_EQ(packet1.signal_quality, packet2.signal_quality);
}

#ifdef TEST_INTEROP
TEST(common_interop, DISTANCE_SENSOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_distance_sensor_t packet_c {
         963497464, 17443, 17547, 17651, 163, 230, 41, 108, 115.0, 143.0, { 171.0, 172.0, 173.0, 174.0 }, 247
    };

    mavlink::common::msg::DISTANCE_SENSOR packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.min_distance = 17443;
    packet_in.max_distance = 17547;
    packet_in.current_distance = 17651;
    packet_in.type = 163;
    packet_in.id = 230;
    packet_in.orientation = 41;
    packet_in.covariance = 108;
    packet_in.horizontal_fov = 115.0;
    packet_in.vertical_fov = 143.0;
    packet_in.quaternion = {{ 171.0, 172.0, 173.0, 174.0 }};
    packet_in.signal_quality = 247;

    mavlink::common::msg::DISTANCE_SENSOR packet2{};

    mavlink_msg_distance_sensor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.min_distance, packet2.min_distance);
    EXPECT_EQ(packet_in.max_distance, packet2.max_distance);
    EXPECT_EQ(packet_in.current_distance, packet2.current_distance);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.orientation, packet2.orientation);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);
    EXPECT_EQ(packet_in.horizontal_fov, packet2.horizontal_fov);
    EXPECT_EQ(packet_in.vertical_fov, packet2.vertical_fov);
    EXPECT_EQ(packet_in.quaternion, packet2.quaternion);
    EXPECT_EQ(packet_in.signal_quality, packet2.signal_quality);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TERRAIN_REQUEST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TERRAIN_REQUEST packet_in{};
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.grid_spacing = 18067;
    packet_in.mask = 93372036854775807ULL;

    mavlink::common::msg::TERRAIN_REQUEST packet1{};
    mavlink::common::msg::TERRAIN_REQUEST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.grid_spacing, packet2.grid_spacing);
    EXPECT_EQ(packet1.mask, packet2.mask);
}

#ifdef TEST_INTEROP
TEST(common_interop, TERRAIN_REQUEST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_terrain_request_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 18067
    };

    mavlink::common::msg::TERRAIN_REQUEST packet_in{};
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.grid_spacing = 18067;
    packet_in.mask = 93372036854775807ULL;

    mavlink::common::msg::TERRAIN_REQUEST packet2{};

    mavlink_msg_terrain_request_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.grid_spacing, packet2.grid_spacing);
    EXPECT_EQ(packet_in.mask, packet2.mask);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TERRAIN_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TERRAIN_DATA packet_in{};
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;
    packet_in.grid_spacing = 17651;
    packet_in.gridbit = 3;
    packet_in.data = {{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764, 17765, 17766, 17767, 17768, 17769, 17770 }};

    mavlink::common::msg::TERRAIN_DATA packet1{};
    mavlink::common::msg::TERRAIN_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.grid_spacing, packet2.grid_spacing);
    EXPECT_EQ(packet1.gridbit, packet2.gridbit);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, TERRAIN_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_terrain_data_t packet_c {
         963497464, 963497672, 17651, { 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764, 17765, 17766, 17767, 17768, 17769, 17770 }, 3
    };

    mavlink::common::msg::TERRAIN_DATA packet_in{};
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;
    packet_in.grid_spacing = 17651;
    packet_in.gridbit = 3;
    packet_in.data = {{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764, 17765, 17766, 17767, 17768, 17769, 17770 }};

    mavlink::common::msg::TERRAIN_DATA packet2{};

    mavlink_msg_terrain_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.grid_spacing, packet2.grid_spacing);
    EXPECT_EQ(packet_in.gridbit, packet2.gridbit);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TERRAIN_CHECK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TERRAIN_CHECK packet_in{};
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;

    mavlink::common::msg::TERRAIN_CHECK packet1{};
    mavlink::common::msg::TERRAIN_CHECK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
}

#ifdef TEST_INTEROP
TEST(common_interop, TERRAIN_CHECK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_terrain_check_t packet_c {
         963497464, 963497672
    };

    mavlink::common::msg::TERRAIN_CHECK packet_in{};
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;

    mavlink::common::msg::TERRAIN_CHECK packet2{};

    mavlink_msg_terrain_check_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TERRAIN_REPORT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TERRAIN_REPORT packet_in{};
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;
    packet_in.spacing = 18067;
    packet_in.terrain_height = 73.0;
    packet_in.current_height = 101.0;
    packet_in.pending = 18171;
    packet_in.loaded = 18275;

    mavlink::common::msg::TERRAIN_REPORT packet1{};
    mavlink::common::msg::TERRAIN_REPORT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.spacing, packet2.spacing);
    EXPECT_EQ(packet1.terrain_height, packet2.terrain_height);
    EXPECT_EQ(packet1.current_height, packet2.current_height);
    EXPECT_EQ(packet1.pending, packet2.pending);
    EXPECT_EQ(packet1.loaded, packet2.loaded);
}

#ifdef TEST_INTEROP
TEST(common_interop, TERRAIN_REPORT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_terrain_report_t packet_c {
         963497464, 963497672, 73.0, 101.0, 18067, 18171, 18275
    };

    mavlink::common::msg::TERRAIN_REPORT packet_in{};
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;
    packet_in.spacing = 18067;
    packet_in.terrain_height = 73.0;
    packet_in.current_height = 101.0;
    packet_in.pending = 18171;
    packet_in.loaded = 18275;

    mavlink::common::msg::TERRAIN_REPORT packet2{};

    mavlink_msg_terrain_report_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.spacing, packet2.spacing);
    EXPECT_EQ(packet_in.terrain_height, packet2.terrain_height);
    EXPECT_EQ(packet_in.current_height, packet2.current_height);
    EXPECT_EQ(packet_in.pending, packet2.pending);
    EXPECT_EQ(packet_in.loaded, packet2.loaded);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SCALED_PRESSURE2)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SCALED_PRESSURE2 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;
    packet_in.temperature_press_diff = 17963;

    mavlink::common::msg::SCALED_PRESSURE2 packet1{};
    mavlink::common::msg::SCALED_PRESSURE2 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.press_abs, packet2.press_abs);
    EXPECT_EQ(packet1.press_diff, packet2.press_diff);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.temperature_press_diff, packet2.temperature_press_diff);
}

#ifdef TEST_INTEROP
TEST(common_interop, SCALED_PRESSURE2)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_scaled_pressure2_t packet_c {
         963497464, 45.0, 73.0, 17859, 17963
    };

    mavlink::common::msg::SCALED_PRESSURE2 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;
    packet_in.temperature_press_diff = 17963;

    mavlink::common::msg::SCALED_PRESSURE2 packet2{};

    mavlink_msg_scaled_pressure2_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.press_abs, packet2.press_abs);
    EXPECT_EQ(packet_in.press_diff, packet2.press_diff);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.temperature_press_diff, packet2.temperature_press_diff);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATT_POS_MOCAP)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATT_POS_MOCAP packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.q = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.x = 185.0;
    packet_in.y = 213.0;
    packet_in.z = 241.0;
    packet_in.covariance = {{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0 }};

    mavlink::common::msg::ATT_POS_MOCAP packet1{};
    mavlink::common::msg::ATT_POS_MOCAP packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATT_POS_MOCAP)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_att_pos_mocap_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0 }, 185.0, 213.0, 241.0, { 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0 }
    };

    mavlink::common::msg::ATT_POS_MOCAP packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.q = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.x = 185.0;
    packet_in.y = 213.0;
    packet_in.z = 241.0;
    packet_in.covariance = {{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0 }};

    mavlink::common::msg::ATT_POS_MOCAP packet2{};

    mavlink_msg_att_pos_mocap_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_ACTUATOR_CONTROL_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.group_mlx = 125;
    packet_in.target_system = 192;
    packet_in.target_component = 3;
    packet_in.controls = {{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 }};

    mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET packet1{};
    mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.group_mlx, packet2.group_mlx);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.controls, packet2.controls);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_ACTUATOR_CONTROL_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_actuator_control_target_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 }, 125, 192, 3
    };

    mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.group_mlx = 125;
    packet_in.target_system = 192;
    packet_in.target_component = 3;
    packet_in.controls = {{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 }};

    mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET packet2{};

    mavlink_msg_set_actuator_control_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.group_mlx, packet2.group_mlx);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.controls, packet2.controls);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ACTUATOR_CONTROL_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ACTUATOR_CONTROL_TARGET packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.group_mlx = 125;
    packet_in.controls = {{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 }};

    mavlink::common::msg::ACTUATOR_CONTROL_TARGET packet1{};
    mavlink::common::msg::ACTUATOR_CONTROL_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.group_mlx, packet2.group_mlx);
    EXPECT_EQ(packet1.controls, packet2.controls);
}

#ifdef TEST_INTEROP
TEST(common_interop, ACTUATOR_CONTROL_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_actuator_control_target_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 }, 125
    };

    mavlink::common::msg::ACTUATOR_CONTROL_TARGET packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.group_mlx = 125;
    packet_in.controls = {{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 }};

    mavlink::common::msg::ACTUATOR_CONTROL_TARGET packet2{};

    mavlink_msg_actuator_control_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.group_mlx, packet2.group_mlx);
    EXPECT_EQ(packet_in.controls, packet2.controls);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ALTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ALTITUDE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.altitude_monotonic = 73.0;
    packet_in.altitude_amsl = 101.0;
    packet_in.altitude_local = 129.0;
    packet_in.altitude_relative = 157.0;
    packet_in.altitude_terrain = 185.0;
    packet_in.bottom_clearance = 213.0;

    mavlink::common::msg::ALTITUDE packet1{};
    mavlink::common::msg::ALTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.altitude_monotonic, packet2.altitude_monotonic);
    EXPECT_EQ(packet1.altitude_amsl, packet2.altitude_amsl);
    EXPECT_EQ(packet1.altitude_local, packet2.altitude_local);
    EXPECT_EQ(packet1.altitude_relative, packet2.altitude_relative);
    EXPECT_EQ(packet1.altitude_terrain, packet2.altitude_terrain);
    EXPECT_EQ(packet1.bottom_clearance, packet2.bottom_clearance);
}

#ifdef TEST_INTEROP
TEST(common_interop, ALTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_altitude_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::common::msg::ALTITUDE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.altitude_monotonic = 73.0;
    packet_in.altitude_amsl = 101.0;
    packet_in.altitude_local = 129.0;
    packet_in.altitude_relative = 157.0;
    packet_in.altitude_terrain = 185.0;
    packet_in.bottom_clearance = 213.0;

    mavlink::common::msg::ALTITUDE packet2{};

    mavlink_msg_altitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.altitude_monotonic, packet2.altitude_monotonic);
    EXPECT_EQ(packet_in.altitude_amsl, packet2.altitude_amsl);
    EXPECT_EQ(packet_in.altitude_local, packet2.altitude_local);
    EXPECT_EQ(packet_in.altitude_relative, packet2.altitude_relative);
    EXPECT_EQ(packet_in.altitude_terrain, packet2.altitude_terrain);
    EXPECT_EQ(packet_in.bottom_clearance, packet2.bottom_clearance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RESOURCE_REQUEST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RESOURCE_REQUEST packet_in{};
    packet_in.request_id = 5;
    packet_in.uri_type = 72;
    packet_in.uri = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2 }};
    packet_in.transfer_type = 243;
    packet_in.storage = {{ 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173 }};

    mavlink::common::msg::RESOURCE_REQUEST packet1{};
    mavlink::common::msg::RESOURCE_REQUEST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.request_id, packet2.request_id);
    EXPECT_EQ(packet1.uri_type, packet2.uri_type);
    EXPECT_EQ(packet1.uri, packet2.uri);
    EXPECT_EQ(packet1.transfer_type, packet2.transfer_type);
    EXPECT_EQ(packet1.storage, packet2.storage);
}

#ifdef TEST_INTEROP
TEST(common_interop, RESOURCE_REQUEST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_resource_request_t packet_c {
         5, 72, { 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2 }, 243, { 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173 }
    };

    mavlink::common::msg::RESOURCE_REQUEST packet_in{};
    packet_in.request_id = 5;
    packet_in.uri_type = 72;
    packet_in.uri = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2 }};
    packet_in.transfer_type = 243;
    packet_in.storage = {{ 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173 }};

    mavlink::common::msg::RESOURCE_REQUEST packet2{};

    mavlink_msg_resource_request_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.request_id, packet2.request_id);
    EXPECT_EQ(packet_in.uri_type, packet2.uri_type);
    EXPECT_EQ(packet_in.uri, packet2.uri);
    EXPECT_EQ(packet_in.transfer_type, packet2.transfer_type);
    EXPECT_EQ(packet_in.storage, packet2.storage);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SCALED_PRESSURE3)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SCALED_PRESSURE3 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;
    packet_in.temperature_press_diff = 17963;

    mavlink::common::msg::SCALED_PRESSURE3 packet1{};
    mavlink::common::msg::SCALED_PRESSURE3 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.press_abs, packet2.press_abs);
    EXPECT_EQ(packet1.press_diff, packet2.press_diff);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.temperature_press_diff, packet2.temperature_press_diff);
}

#ifdef TEST_INTEROP
TEST(common_interop, SCALED_PRESSURE3)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_scaled_pressure3_t packet_c {
         963497464, 45.0, 73.0, 17859, 17963
    };

    mavlink::common::msg::SCALED_PRESSURE3 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;
    packet_in.temperature_press_diff = 17963;

    mavlink::common::msg::SCALED_PRESSURE3 packet2{};

    mavlink_msg_scaled_pressure3_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.press_abs, packet2.press_abs);
    EXPECT_EQ(packet_in.press_diff, packet2.press_diff);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.temperature_press_diff, packet2.temperature_press_diff);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, FOLLOW_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::FOLLOW_TARGET packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.est_capabilities = 25;
    packet_in.lat = 963498296;
    packet_in.lon = 963498504;
    packet_in.alt = 185.0;
    packet_in.vel = {{ 213.0, 214.0, 215.0 }};
    packet_in.acc = {{ 297.0, 298.0, 299.0 }};
    packet_in.attitude_q = {{ 381.0, 382.0, 383.0, 384.0 }};
    packet_in.rates = {{ 493.0, 494.0, 495.0 }};
    packet_in.position_cov = {{ 577.0, 578.0, 579.0 }};
    packet_in.custom_state = 93372036854776311ULL;

    mavlink::common::msg::FOLLOW_TARGET packet1{};
    mavlink::common::msg::FOLLOW_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.est_capabilities, packet2.est_capabilities);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.vel, packet2.vel);
    EXPECT_EQ(packet1.acc, packet2.acc);
    EXPECT_EQ(packet1.attitude_q, packet2.attitude_q);
    EXPECT_EQ(packet1.rates, packet2.rates);
    EXPECT_EQ(packet1.position_cov, packet2.position_cov);
    EXPECT_EQ(packet1.custom_state, packet2.custom_state);
}

#ifdef TEST_INTEROP
TEST(common_interop, FOLLOW_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_follow_target_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 963498296, 963498504, 185.0, { 213.0, 214.0, 215.0 }, { 297.0, 298.0, 299.0 }, { 381.0, 382.0, 383.0, 384.0 }, { 493.0, 494.0, 495.0 }, { 577.0, 578.0, 579.0 }, 25
    };

    mavlink::common::msg::FOLLOW_TARGET packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.est_capabilities = 25;
    packet_in.lat = 963498296;
    packet_in.lon = 963498504;
    packet_in.alt = 185.0;
    packet_in.vel = {{ 213.0, 214.0, 215.0 }};
    packet_in.acc = {{ 297.0, 298.0, 299.0 }};
    packet_in.attitude_q = {{ 381.0, 382.0, 383.0, 384.0 }};
    packet_in.rates = {{ 493.0, 494.0, 495.0 }};
    packet_in.position_cov = {{ 577.0, 578.0, 579.0 }};
    packet_in.custom_state = 93372036854776311ULL;

    mavlink::common::msg::FOLLOW_TARGET packet2{};

    mavlink_msg_follow_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.est_capabilities, packet2.est_capabilities);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.vel, packet2.vel);
    EXPECT_EQ(packet_in.acc, packet2.acc);
    EXPECT_EQ(packet_in.attitude_q, packet2.attitude_q);
    EXPECT_EQ(packet_in.rates, packet2.rates);
    EXPECT_EQ(packet_in.position_cov, packet2.position_cov);
    EXPECT_EQ(packet_in.custom_state, packet2.custom_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CONTROL_SYSTEM_STATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CONTROL_SYSTEM_STATE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.x_acc = 73.0;
    packet_in.y_acc = 101.0;
    packet_in.z_acc = 129.0;
    packet_in.x_vel = 157.0;
    packet_in.y_vel = 185.0;
    packet_in.z_vel = 213.0;
    packet_in.x_pos = 241.0;
    packet_in.y_pos = 269.0;
    packet_in.z_pos = 297.0;
    packet_in.airspeed = 325.0;
    packet_in.vel_variance = {{ 353.0, 354.0, 355.0 }};
    packet_in.pos_variance = {{ 437.0, 438.0, 439.0 }};
    packet_in.q = {{ 521.0, 522.0, 523.0, 524.0 }};
    packet_in.roll_rate = 633.0;
    packet_in.pitch_rate = 661.0;
    packet_in.yaw_rate = 689.0;

    mavlink::common::msg::CONTROL_SYSTEM_STATE packet1{};
    mavlink::common::msg::CONTROL_SYSTEM_STATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.x_acc, packet2.x_acc);
    EXPECT_EQ(packet1.y_acc, packet2.y_acc);
    EXPECT_EQ(packet1.z_acc, packet2.z_acc);
    EXPECT_EQ(packet1.x_vel, packet2.x_vel);
    EXPECT_EQ(packet1.y_vel, packet2.y_vel);
    EXPECT_EQ(packet1.z_vel, packet2.z_vel);
    EXPECT_EQ(packet1.x_pos, packet2.x_pos);
    EXPECT_EQ(packet1.y_pos, packet2.y_pos);
    EXPECT_EQ(packet1.z_pos, packet2.z_pos);
    EXPECT_EQ(packet1.airspeed, packet2.airspeed);
    EXPECT_EQ(packet1.vel_variance, packet2.vel_variance);
    EXPECT_EQ(packet1.pos_variance, packet2.pos_variance);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.roll_rate, packet2.roll_rate);
    EXPECT_EQ(packet1.pitch_rate, packet2.pitch_rate);
    EXPECT_EQ(packet1.yaw_rate, packet2.yaw_rate);
}

#ifdef TEST_INTEROP
TEST(common_interop, CONTROL_SYSTEM_STATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_control_system_state_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, { 353.0, 354.0, 355.0 }, { 437.0, 438.0, 439.0 }, { 521.0, 522.0, 523.0, 524.0 }, 633.0, 661.0, 689.0
    };

    mavlink::common::msg::CONTROL_SYSTEM_STATE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.x_acc = 73.0;
    packet_in.y_acc = 101.0;
    packet_in.z_acc = 129.0;
    packet_in.x_vel = 157.0;
    packet_in.y_vel = 185.0;
    packet_in.z_vel = 213.0;
    packet_in.x_pos = 241.0;
    packet_in.y_pos = 269.0;
    packet_in.z_pos = 297.0;
    packet_in.airspeed = 325.0;
    packet_in.vel_variance = {{ 353.0, 354.0, 355.0 }};
    packet_in.pos_variance = {{ 437.0, 438.0, 439.0 }};
    packet_in.q = {{ 521.0, 522.0, 523.0, 524.0 }};
    packet_in.roll_rate = 633.0;
    packet_in.pitch_rate = 661.0;
    packet_in.yaw_rate = 689.0;

    mavlink::common::msg::CONTROL_SYSTEM_STATE packet2{};

    mavlink_msg_control_system_state_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.x_acc, packet2.x_acc);
    EXPECT_EQ(packet_in.y_acc, packet2.y_acc);
    EXPECT_EQ(packet_in.z_acc, packet2.z_acc);
    EXPECT_EQ(packet_in.x_vel, packet2.x_vel);
    EXPECT_EQ(packet_in.y_vel, packet2.y_vel);
    EXPECT_EQ(packet_in.z_vel, packet2.z_vel);
    EXPECT_EQ(packet_in.x_pos, packet2.x_pos);
    EXPECT_EQ(packet_in.y_pos, packet2.y_pos);
    EXPECT_EQ(packet_in.z_pos, packet2.z_pos);
    EXPECT_EQ(packet_in.airspeed, packet2.airspeed);
    EXPECT_EQ(packet_in.vel_variance, packet2.vel_variance);
    EXPECT_EQ(packet_in.pos_variance, packet2.pos_variance);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.roll_rate, packet2.roll_rate);
    EXPECT_EQ(packet_in.pitch_rate, packet2.pitch_rate);
    EXPECT_EQ(packet_in.yaw_rate, packet2.yaw_rate);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, BATTERY_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::BATTERY_STATUS packet_in{};
    packet_in.id = 101;
    packet_in.battery_function = 168;
    packet_in.type = 235;
    packet_in.temperature = 17651;
    packet_in.voltages = {{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764 }};
    packet_in.current_battery = 18795;
    packet_in.current_consumed = 963497464;
    packet_in.energy_consumed = 963497672;
    packet_in.battery_remaining = 46;
    packet_in.time_remaining = 963499336;
    packet_in.charge_state = 125;
    packet_in.voltages_ext = {{ 19367, 19368, 19369, 19370 }};
    packet_in.mode = 216;
    packet_in.fault_bitmask = 963500064;

    mavlink::common::msg::BATTERY_STATUS packet1{};
    mavlink::common::msg::BATTERY_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.battery_function, packet2.battery_function);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.voltages, packet2.voltages);
    EXPECT_EQ(packet1.current_battery, packet2.current_battery);
    EXPECT_EQ(packet1.current_consumed, packet2.current_consumed);
    EXPECT_EQ(packet1.energy_consumed, packet2.energy_consumed);
    EXPECT_EQ(packet1.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet1.time_remaining, packet2.time_remaining);
    EXPECT_EQ(packet1.charge_state, packet2.charge_state);
    EXPECT_EQ(packet1.voltages_ext, packet2.voltages_ext);
    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.fault_bitmask, packet2.fault_bitmask);
}

#ifdef TEST_INTEROP
TEST(common_interop, BATTERY_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_battery_status_t packet_c {
         963497464, 963497672, 17651, { 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764 }, 18795, 101, 168, 235, 46, 963499336, 125, { 19367, 19368, 19369, 19370 }, 216, 963500064
    };

    mavlink::common::msg::BATTERY_STATUS packet_in{};
    packet_in.id = 101;
    packet_in.battery_function = 168;
    packet_in.type = 235;
    packet_in.temperature = 17651;
    packet_in.voltages = {{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764 }};
    packet_in.current_battery = 18795;
    packet_in.current_consumed = 963497464;
    packet_in.energy_consumed = 963497672;
    packet_in.battery_remaining = 46;
    packet_in.time_remaining = 963499336;
    packet_in.charge_state = 125;
    packet_in.voltages_ext = {{ 19367, 19368, 19369, 19370 }};
    packet_in.mode = 216;
    packet_in.fault_bitmask = 963500064;

    mavlink::common::msg::BATTERY_STATUS packet2{};

    mavlink_msg_battery_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.battery_function, packet2.battery_function);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.voltages, packet2.voltages);
    EXPECT_EQ(packet_in.current_battery, packet2.current_battery);
    EXPECT_EQ(packet_in.current_consumed, packet2.current_consumed);
    EXPECT_EQ(packet_in.energy_consumed, packet2.energy_consumed);
    EXPECT_EQ(packet_in.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet_in.time_remaining, packet2.time_remaining);
    EXPECT_EQ(packet_in.charge_state, packet2.charge_state);
    EXPECT_EQ(packet_in.voltages_ext, packet2.voltages_ext);
    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.fault_bitmask, packet2.fault_bitmask);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, AUTOPILOT_VERSION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::AUTOPILOT_VERSION packet_in{};
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.flight_sw_version = 963498296;
    packet_in.middleware_sw_version = 963498504;
    packet_in.os_sw_version = 963498712;
    packet_in.board_version = 963498920;
    packet_in.flight_custom_version = {{ 113, 114, 115, 116, 117, 118, 119, 120 }};
    packet_in.middleware_custom_version = {{ 137, 138, 139, 140, 141, 142, 143, 144 }};
    packet_in.os_custom_version = {{ 161, 162, 163, 164, 165, 166, 167, 168 }};
    packet_in.vendor_id = 18899;
    packet_in.product_id = 19003;
    packet_in.uid = 93372036854776311ULL;
    packet_in.uid2 = {{ 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }};

    mavlink::common::msg::AUTOPILOT_VERSION packet1{};
    mavlink::common::msg::AUTOPILOT_VERSION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.capabilities, packet2.capabilities);
    EXPECT_EQ(packet1.flight_sw_version, packet2.flight_sw_version);
    EXPECT_EQ(packet1.middleware_sw_version, packet2.middleware_sw_version);
    EXPECT_EQ(packet1.os_sw_version, packet2.os_sw_version);
    EXPECT_EQ(packet1.board_version, packet2.board_version);
    EXPECT_EQ(packet1.flight_custom_version, packet2.flight_custom_version);
    EXPECT_EQ(packet1.middleware_custom_version, packet2.middleware_custom_version);
    EXPECT_EQ(packet1.os_custom_version, packet2.os_custom_version);
    EXPECT_EQ(packet1.vendor_id, packet2.vendor_id);
    EXPECT_EQ(packet1.product_id, packet2.product_id);
    EXPECT_EQ(packet1.uid, packet2.uid);
    EXPECT_EQ(packet1.uid2, packet2.uid2);
}

#ifdef TEST_INTEROP
TEST(common_interop, AUTOPILOT_VERSION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_autopilot_version_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 963498296, 963498504, 963498712, 963498920, 18899, 19003, { 113, 114, 115, 116, 117, 118, 119, 120 }, { 137, 138, 139, 140, 141, 142, 143, 144 }, { 161, 162, 163, 164, 165, 166, 167, 168 }, { 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }
    };

    mavlink::common::msg::AUTOPILOT_VERSION packet_in{};
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.flight_sw_version = 963498296;
    packet_in.middleware_sw_version = 963498504;
    packet_in.os_sw_version = 963498712;
    packet_in.board_version = 963498920;
    packet_in.flight_custom_version = {{ 113, 114, 115, 116, 117, 118, 119, 120 }};
    packet_in.middleware_custom_version = {{ 137, 138, 139, 140, 141, 142, 143, 144 }};
    packet_in.os_custom_version = {{ 161, 162, 163, 164, 165, 166, 167, 168 }};
    packet_in.vendor_id = 18899;
    packet_in.product_id = 19003;
    packet_in.uid = 93372036854776311ULL;
    packet_in.uid2 = {{ 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }};

    mavlink::common::msg::AUTOPILOT_VERSION packet2{};

    mavlink_msg_autopilot_version_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.capabilities, packet2.capabilities);
    EXPECT_EQ(packet_in.flight_sw_version, packet2.flight_sw_version);
    EXPECT_EQ(packet_in.middleware_sw_version, packet2.middleware_sw_version);
    EXPECT_EQ(packet_in.os_sw_version, packet2.os_sw_version);
    EXPECT_EQ(packet_in.board_version, packet2.board_version);
    EXPECT_EQ(packet_in.flight_custom_version, packet2.flight_custom_version);
    EXPECT_EQ(packet_in.middleware_custom_version, packet2.middleware_custom_version);
    EXPECT_EQ(packet_in.os_custom_version, packet2.os_custom_version);
    EXPECT_EQ(packet_in.vendor_id, packet2.vendor_id);
    EXPECT_EQ(packet_in.product_id, packet2.product_id);
    EXPECT_EQ(packet_in.uid, packet2.uid);
    EXPECT_EQ(packet_in.uid2, packet2.uid2);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LANDING_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LANDING_TARGET packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.target_num = 89;
    packet_in.frame = 156;
    packet_in.angle_x = 73.0;
    packet_in.angle_y = 101.0;
    packet_in.distance = 129.0;
    packet_in.size_x = 157.0;
    packet_in.size_y = 185.0;
    packet_in.x = 227.0;
    packet_in.y = 255.0;
    packet_in.z = 283.0;
    packet_in.q = {{ 311.0, 312.0, 313.0, 314.0 }};
    packet_in.type = 51;
    packet_in.position_valid = 118;

    mavlink::common::msg::LANDING_TARGET packet1{};
    mavlink::common::msg::LANDING_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.target_num, packet2.target_num);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.angle_x, packet2.angle_x);
    EXPECT_EQ(packet1.angle_y, packet2.angle_y);
    EXPECT_EQ(packet1.distance, packet2.distance);
    EXPECT_EQ(packet1.size_x, packet2.size_x);
    EXPECT_EQ(packet1.size_y, packet2.size_y);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.position_valid, packet2.position_valid);
}

#ifdef TEST_INTEROP
TEST(common_interop, LANDING_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_landing_target_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 89, 156, 227.0, 255.0, 283.0, { 311.0, 312.0, 313.0, 314.0 }, 51, 118
    };

    mavlink::common::msg::LANDING_TARGET packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.target_num = 89;
    packet_in.frame = 156;
    packet_in.angle_x = 73.0;
    packet_in.angle_y = 101.0;
    packet_in.distance = 129.0;
    packet_in.size_x = 157.0;
    packet_in.size_y = 185.0;
    packet_in.x = 227.0;
    packet_in.y = 255.0;
    packet_in.z = 283.0;
    packet_in.q = {{ 311.0, 312.0, 313.0, 314.0 }};
    packet_in.type = 51;
    packet_in.position_valid = 118;

    mavlink::common::msg::LANDING_TARGET packet2{};

    mavlink_msg_landing_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.target_num, packet2.target_num);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.angle_x, packet2.angle_x);
    EXPECT_EQ(packet_in.angle_y, packet2.angle_y);
    EXPECT_EQ(packet_in.distance, packet2.distance);
    EXPECT_EQ(packet_in.size_x, packet2.size_x);
    EXPECT_EQ(packet_in.size_y, packet2.size_y);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.position_valid, packet2.position_valid);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, FENCE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::FENCE_STATUS packet_in{};
    packet_in.breach_status = 151;
    packet_in.breach_count = 17443;
    packet_in.breach_type = 218;
    packet_in.breach_time = 963497464;
    packet_in.breach_mitigation = 29;

    mavlink::common::msg::FENCE_STATUS packet1{};
    mavlink::common::msg::FENCE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.breach_status, packet2.breach_status);
    EXPECT_EQ(packet1.breach_count, packet2.breach_count);
    EXPECT_EQ(packet1.breach_type, packet2.breach_type);
    EXPECT_EQ(packet1.breach_time, packet2.breach_time);
    EXPECT_EQ(packet1.breach_mitigation, packet2.breach_mitigation);
}

#ifdef TEST_INTEROP
TEST(common_interop, FENCE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_fence_status_t packet_c {
         963497464, 17443, 151, 218, 29
    };

    mavlink::common::msg::FENCE_STATUS packet_in{};
    packet_in.breach_status = 151;
    packet_in.breach_count = 17443;
    packet_in.breach_type = 218;
    packet_in.breach_time = 963497464;
    packet_in.breach_mitigation = 29;

    mavlink::common::msg::FENCE_STATUS packet2{};

    mavlink_msg_fence_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.breach_status, packet2.breach_status);
    EXPECT_EQ(packet_in.breach_count, packet2.breach_count);
    EXPECT_EQ(packet_in.breach_type, packet2.breach_type);
    EXPECT_EQ(packet_in.breach_time, packet2.breach_time);
    EXPECT_EQ(packet_in.breach_mitigation, packet2.breach_mitigation);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MAG_CAL_REPORT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MAG_CAL_REPORT packet_in{};
    packet_in.compass_id = 125;
    packet_in.cal_mask = 192;
    packet_in.cal_status = 3;
    packet_in.autosaved = 70;
    packet_in.fitness = 17.0;
    packet_in.ofs_x = 45.0;
    packet_in.ofs_y = 73.0;
    packet_in.ofs_z = 101.0;
    packet_in.diag_x = 129.0;
    packet_in.diag_y = 157.0;
    packet_in.diag_z = 185.0;
    packet_in.offdiag_x = 213.0;
    packet_in.offdiag_y = 241.0;
    packet_in.offdiag_z = 269.0;
    packet_in.orientation_confidence = 325.0;
    packet_in.old_orientation = 149;
    packet_in.new_orientation = 216;
    packet_in.scale_factor = 367.0;

    mavlink::common::msg::MAG_CAL_REPORT packet1{};
    mavlink::common::msg::MAG_CAL_REPORT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.compass_id, packet2.compass_id);
    EXPECT_EQ(packet1.cal_mask, packet2.cal_mask);
    EXPECT_EQ(packet1.cal_status, packet2.cal_status);
    EXPECT_EQ(packet1.autosaved, packet2.autosaved);
    EXPECT_EQ(packet1.fitness, packet2.fitness);
    EXPECT_EQ(packet1.ofs_x, packet2.ofs_x);
    EXPECT_EQ(packet1.ofs_y, packet2.ofs_y);
    EXPECT_EQ(packet1.ofs_z, packet2.ofs_z);
    EXPECT_EQ(packet1.diag_x, packet2.diag_x);
    EXPECT_EQ(packet1.diag_y, packet2.diag_y);
    EXPECT_EQ(packet1.diag_z, packet2.diag_z);
    EXPECT_EQ(packet1.offdiag_x, packet2.offdiag_x);
    EXPECT_EQ(packet1.offdiag_y, packet2.offdiag_y);
    EXPECT_EQ(packet1.offdiag_z, packet2.offdiag_z);
    EXPECT_EQ(packet1.orientation_confidence, packet2.orientation_confidence);
    EXPECT_EQ(packet1.old_orientation, packet2.old_orientation);
    EXPECT_EQ(packet1.new_orientation, packet2.new_orientation);
    EXPECT_EQ(packet1.scale_factor, packet2.scale_factor);
}

#ifdef TEST_INTEROP
TEST(common_interop, MAG_CAL_REPORT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mag_cal_report_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 125, 192, 3, 70, 325.0, 149, 216, 367.0
    };

    mavlink::common::msg::MAG_CAL_REPORT packet_in{};
    packet_in.compass_id = 125;
    packet_in.cal_mask = 192;
    packet_in.cal_status = 3;
    packet_in.autosaved = 70;
    packet_in.fitness = 17.0;
    packet_in.ofs_x = 45.0;
    packet_in.ofs_y = 73.0;
    packet_in.ofs_z = 101.0;
    packet_in.diag_x = 129.0;
    packet_in.diag_y = 157.0;
    packet_in.diag_z = 185.0;
    packet_in.offdiag_x = 213.0;
    packet_in.offdiag_y = 241.0;
    packet_in.offdiag_z = 269.0;
    packet_in.orientation_confidence = 325.0;
    packet_in.old_orientation = 149;
    packet_in.new_orientation = 216;
    packet_in.scale_factor = 367.0;

    mavlink::common::msg::MAG_CAL_REPORT packet2{};

    mavlink_msg_mag_cal_report_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.compass_id, packet2.compass_id);
    EXPECT_EQ(packet_in.cal_mask, packet2.cal_mask);
    EXPECT_EQ(packet_in.cal_status, packet2.cal_status);
    EXPECT_EQ(packet_in.autosaved, packet2.autosaved);
    EXPECT_EQ(packet_in.fitness, packet2.fitness);
    EXPECT_EQ(packet_in.ofs_x, packet2.ofs_x);
    EXPECT_EQ(packet_in.ofs_y, packet2.ofs_y);
    EXPECT_EQ(packet_in.ofs_z, packet2.ofs_z);
    EXPECT_EQ(packet_in.diag_x, packet2.diag_x);
    EXPECT_EQ(packet_in.diag_y, packet2.diag_y);
    EXPECT_EQ(packet_in.diag_z, packet2.diag_z);
    EXPECT_EQ(packet_in.offdiag_x, packet2.offdiag_x);
    EXPECT_EQ(packet_in.offdiag_y, packet2.offdiag_y);
    EXPECT_EQ(packet_in.offdiag_z, packet2.offdiag_z);
    EXPECT_EQ(packet_in.orientation_confidence, packet2.orientation_confidence);
    EXPECT_EQ(packet_in.old_orientation, packet2.old_orientation);
    EXPECT_EQ(packet_in.new_orientation, packet2.new_orientation);
    EXPECT_EQ(packet_in.scale_factor, packet2.scale_factor);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, EFI_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::EFI_STATUS packet_in{};
    packet_in.health = 197;
    packet_in.ecu_index = 17.0;
    packet_in.rpm = 45.0;
    packet_in.fuel_consumed = 73.0;
    packet_in.fuel_flow = 101.0;
    packet_in.engine_load = 129.0;
    packet_in.throttle_position = 157.0;
    packet_in.spark_dwell_time = 185.0;
    packet_in.barometric_pressure = 213.0;
    packet_in.intake_manifold_pressure = 241.0;
    packet_in.intake_manifold_temperature = 269.0;
    packet_in.cylinder_head_temperature = 297.0;
    packet_in.ignition_timing = 325.0;
    packet_in.injection_time = 353.0;
    packet_in.exhaust_gas_temperature = 381.0;
    packet_in.throttle_out = 409.0;
    packet_in.pt_compensation = 437.0;
    packet_in.ignition_voltage = 472.0;
    packet_in.fuel_pressure = 500.0;

    mavlink::common::msg::EFI_STATUS packet1{};
    mavlink::common::msg::EFI_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.health, packet2.health);
    EXPECT_EQ(packet1.ecu_index, packet2.ecu_index);
    EXPECT_EQ(packet1.rpm, packet2.rpm);
    EXPECT_EQ(packet1.fuel_consumed, packet2.fuel_consumed);
    EXPECT_EQ(packet1.fuel_flow, packet2.fuel_flow);
    EXPECT_EQ(packet1.engine_load, packet2.engine_load);
    EXPECT_EQ(packet1.throttle_position, packet2.throttle_position);
    EXPECT_EQ(packet1.spark_dwell_time, packet2.spark_dwell_time);
    EXPECT_EQ(packet1.barometric_pressure, packet2.barometric_pressure);
    EXPECT_EQ(packet1.intake_manifold_pressure, packet2.intake_manifold_pressure);
    EXPECT_EQ(packet1.intake_manifold_temperature, packet2.intake_manifold_temperature);
    EXPECT_EQ(packet1.cylinder_head_temperature, packet2.cylinder_head_temperature);
    EXPECT_EQ(packet1.ignition_timing, packet2.ignition_timing);
    EXPECT_EQ(packet1.injection_time, packet2.injection_time);
    EXPECT_EQ(packet1.exhaust_gas_temperature, packet2.exhaust_gas_temperature);
    EXPECT_EQ(packet1.throttle_out, packet2.throttle_out);
    EXPECT_EQ(packet1.pt_compensation, packet2.pt_compensation);
    EXPECT_EQ(packet1.ignition_voltage, packet2.ignition_voltage);
    EXPECT_EQ(packet1.fuel_pressure, packet2.fuel_pressure);
}

#ifdef TEST_INTEROP
TEST(common_interop, EFI_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_efi_status_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0, 381.0, 409.0, 437.0, 197, 472.0, 500.0
    };

    mavlink::common::msg::EFI_STATUS packet_in{};
    packet_in.health = 197;
    packet_in.ecu_index = 17.0;
    packet_in.rpm = 45.0;
    packet_in.fuel_consumed = 73.0;
    packet_in.fuel_flow = 101.0;
    packet_in.engine_load = 129.0;
    packet_in.throttle_position = 157.0;
    packet_in.spark_dwell_time = 185.0;
    packet_in.barometric_pressure = 213.0;
    packet_in.intake_manifold_pressure = 241.0;
    packet_in.intake_manifold_temperature = 269.0;
    packet_in.cylinder_head_temperature = 297.0;
    packet_in.ignition_timing = 325.0;
    packet_in.injection_time = 353.0;
    packet_in.exhaust_gas_temperature = 381.0;
    packet_in.throttle_out = 409.0;
    packet_in.pt_compensation = 437.0;
    packet_in.ignition_voltage = 472.0;
    packet_in.fuel_pressure = 500.0;

    mavlink::common::msg::EFI_STATUS packet2{};

    mavlink_msg_efi_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.health, packet2.health);
    EXPECT_EQ(packet_in.ecu_index, packet2.ecu_index);
    EXPECT_EQ(packet_in.rpm, packet2.rpm);
    EXPECT_EQ(packet_in.fuel_consumed, packet2.fuel_consumed);
    EXPECT_EQ(packet_in.fuel_flow, packet2.fuel_flow);
    EXPECT_EQ(packet_in.engine_load, packet2.engine_load);
    EXPECT_EQ(packet_in.throttle_position, packet2.throttle_position);
    EXPECT_EQ(packet_in.spark_dwell_time, packet2.spark_dwell_time);
    EXPECT_EQ(packet_in.barometric_pressure, packet2.barometric_pressure);
    EXPECT_EQ(packet_in.intake_manifold_pressure, packet2.intake_manifold_pressure);
    EXPECT_EQ(packet_in.intake_manifold_temperature, packet2.intake_manifold_temperature);
    EXPECT_EQ(packet_in.cylinder_head_temperature, packet2.cylinder_head_temperature);
    EXPECT_EQ(packet_in.ignition_timing, packet2.ignition_timing);
    EXPECT_EQ(packet_in.injection_time, packet2.injection_time);
    EXPECT_EQ(packet_in.exhaust_gas_temperature, packet2.exhaust_gas_temperature);
    EXPECT_EQ(packet_in.throttle_out, packet2.throttle_out);
    EXPECT_EQ(packet_in.pt_compensation, packet2.pt_compensation);
    EXPECT_EQ(packet_in.ignition_voltage, packet2.ignition_voltage);
    EXPECT_EQ(packet_in.fuel_pressure, packet2.fuel_pressure);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ESTIMATOR_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ESTIMATOR_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.flags = 19315;
    packet_in.vel_ratio = 73.0;
    packet_in.pos_horiz_ratio = 101.0;
    packet_in.pos_vert_ratio = 129.0;
    packet_in.mag_ratio = 157.0;
    packet_in.hagl_ratio = 185.0;
    packet_in.tas_ratio = 213.0;
    packet_in.pos_horiz_accuracy = 241.0;
    packet_in.pos_vert_accuracy = 269.0;

    mavlink::common::msg::ESTIMATOR_STATUS packet1{};
    mavlink::common::msg::ESTIMATOR_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.vel_ratio, packet2.vel_ratio);
    EXPECT_EQ(packet1.pos_horiz_ratio, packet2.pos_horiz_ratio);
    EXPECT_EQ(packet1.pos_vert_ratio, packet2.pos_vert_ratio);
    EXPECT_EQ(packet1.mag_ratio, packet2.mag_ratio);
    EXPECT_EQ(packet1.hagl_ratio, packet2.hagl_ratio);
    EXPECT_EQ(packet1.tas_ratio, packet2.tas_ratio);
    EXPECT_EQ(packet1.pos_horiz_accuracy, packet2.pos_horiz_accuracy);
    EXPECT_EQ(packet1.pos_vert_accuracy, packet2.pos_vert_accuracy);
}

#ifdef TEST_INTEROP
TEST(common_interop, ESTIMATOR_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_estimator_status_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 19315
    };

    mavlink::common::msg::ESTIMATOR_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.flags = 19315;
    packet_in.vel_ratio = 73.0;
    packet_in.pos_horiz_ratio = 101.0;
    packet_in.pos_vert_ratio = 129.0;
    packet_in.mag_ratio = 157.0;
    packet_in.hagl_ratio = 185.0;
    packet_in.tas_ratio = 213.0;
    packet_in.pos_horiz_accuracy = 241.0;
    packet_in.pos_vert_accuracy = 269.0;

    mavlink::common::msg::ESTIMATOR_STATUS packet2{};

    mavlink_msg_estimator_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.vel_ratio, packet2.vel_ratio);
    EXPECT_EQ(packet_in.pos_horiz_ratio, packet2.pos_horiz_ratio);
    EXPECT_EQ(packet_in.pos_vert_ratio, packet2.pos_vert_ratio);
    EXPECT_EQ(packet_in.mag_ratio, packet2.mag_ratio);
    EXPECT_EQ(packet_in.hagl_ratio, packet2.hagl_ratio);
    EXPECT_EQ(packet_in.tas_ratio, packet2.tas_ratio);
    EXPECT_EQ(packet_in.pos_horiz_accuracy, packet2.pos_horiz_accuracy);
    EXPECT_EQ(packet_in.pos_vert_accuracy, packet2.pos_vert_accuracy);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, WIND_COV)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::WIND_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.wind_x = 73.0;
    packet_in.wind_y = 101.0;
    packet_in.wind_z = 129.0;
    packet_in.var_horiz = 157.0;
    packet_in.var_vert = 185.0;
    packet_in.wind_alt = 213.0;
    packet_in.horiz_accuracy = 241.0;
    packet_in.vert_accuracy = 269.0;

    mavlink::common::msg::WIND_COV packet1{};
    mavlink::common::msg::WIND_COV packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.wind_x, packet2.wind_x);
    EXPECT_EQ(packet1.wind_y, packet2.wind_y);
    EXPECT_EQ(packet1.wind_z, packet2.wind_z);
    EXPECT_EQ(packet1.var_horiz, packet2.var_horiz);
    EXPECT_EQ(packet1.var_vert, packet2.var_vert);
    EXPECT_EQ(packet1.wind_alt, packet2.wind_alt);
    EXPECT_EQ(packet1.horiz_accuracy, packet2.horiz_accuracy);
    EXPECT_EQ(packet1.vert_accuracy, packet2.vert_accuracy);
}

#ifdef TEST_INTEROP
TEST(common_interop, WIND_COV)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_wind_cov_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0
    };

    mavlink::common::msg::WIND_COV packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.wind_x = 73.0;
    packet_in.wind_y = 101.0;
    packet_in.wind_z = 129.0;
    packet_in.var_horiz = 157.0;
    packet_in.var_vert = 185.0;
    packet_in.wind_alt = 213.0;
    packet_in.horiz_accuracy = 241.0;
    packet_in.vert_accuracy = 269.0;

    mavlink::common::msg::WIND_COV packet2{};

    mavlink_msg_wind_cov_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.wind_x, packet2.wind_x);
    EXPECT_EQ(packet_in.wind_y, packet2.wind_y);
    EXPECT_EQ(packet_in.wind_z, packet2.wind_z);
    EXPECT_EQ(packet_in.var_horiz, packet2.var_horiz);
    EXPECT_EQ(packet_in.var_vert, packet2.var_vert);
    EXPECT_EQ(packet_in.wind_alt, packet2.wind_alt);
    EXPECT_EQ(packet_in.horiz_accuracy, packet2.horiz_accuracy);
    EXPECT_EQ(packet_in.vert_accuracy, packet2.vert_accuracy);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_INPUT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_INPUT packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.gps_id = 185;
    packet_in.ignore_flags = 20147;
    packet_in.time_week_ms = 963497880;
    packet_in.time_week = 20251;
    packet_in.fix_type = 252;
    packet_in.lat = 963498088;
    packet_in.lon = 963498296;
    packet_in.alt = 157.0;
    packet_in.hdop = 185.0;
    packet_in.vdop = 213.0;
    packet_in.vn = 241.0;
    packet_in.ve = 269.0;
    packet_in.vd = 297.0;
    packet_in.speed_accuracy = 325.0;
    packet_in.horiz_accuracy = 353.0;
    packet_in.vert_accuracy = 381.0;
    packet_in.satellites_visible = 63;
    packet_in.yaw = 20511;

    mavlink::common::msg::GPS_INPUT packet1{};
    mavlink::common::msg::GPS_INPUT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.gps_id, packet2.gps_id);
    EXPECT_EQ(packet1.ignore_flags, packet2.ignore_flags);
    EXPECT_EQ(packet1.time_week_ms, packet2.time_week_ms);
    EXPECT_EQ(packet1.time_week, packet2.time_week);
    EXPECT_EQ(packet1.fix_type, packet2.fix_type);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.hdop, packet2.hdop);
    EXPECT_EQ(packet1.vdop, packet2.vdop);
    EXPECT_EQ(packet1.vn, packet2.vn);
    EXPECT_EQ(packet1.ve, packet2.ve);
    EXPECT_EQ(packet1.vd, packet2.vd);
    EXPECT_EQ(packet1.speed_accuracy, packet2.speed_accuracy);
    EXPECT_EQ(packet1.horiz_accuracy, packet2.horiz_accuracy);
    EXPECT_EQ(packet1.vert_accuracy, packet2.vert_accuracy);
    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_INPUT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_input_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0, 381.0, 20147, 20251, 185, 252, 63, 20511
    };

    mavlink::common::msg::GPS_INPUT packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.gps_id = 185;
    packet_in.ignore_flags = 20147;
    packet_in.time_week_ms = 963497880;
    packet_in.time_week = 20251;
    packet_in.fix_type = 252;
    packet_in.lat = 963498088;
    packet_in.lon = 963498296;
    packet_in.alt = 157.0;
    packet_in.hdop = 185.0;
    packet_in.vdop = 213.0;
    packet_in.vn = 241.0;
    packet_in.ve = 269.0;
    packet_in.vd = 297.0;
    packet_in.speed_accuracy = 325.0;
    packet_in.horiz_accuracy = 353.0;
    packet_in.vert_accuracy = 381.0;
    packet_in.satellites_visible = 63;
    packet_in.yaw = 20511;

    mavlink::common::msg::GPS_INPUT packet2{};

    mavlink_msg_gps_input_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.gps_id, packet2.gps_id);
    EXPECT_EQ(packet_in.ignore_flags, packet2.ignore_flags);
    EXPECT_EQ(packet_in.time_week_ms, packet2.time_week_ms);
    EXPECT_EQ(packet_in.time_week, packet2.time_week);
    EXPECT_EQ(packet_in.fix_type, packet2.fix_type);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.hdop, packet2.hdop);
    EXPECT_EQ(packet_in.vdop, packet2.vdop);
    EXPECT_EQ(packet_in.vn, packet2.vn);
    EXPECT_EQ(packet_in.ve, packet2.ve);
    EXPECT_EQ(packet_in.vd, packet2.vd);
    EXPECT_EQ(packet_in.speed_accuracy, packet2.speed_accuracy);
    EXPECT_EQ(packet_in.horiz_accuracy, packet2.horiz_accuracy);
    EXPECT_EQ(packet_in.vert_accuracy, packet2.vert_accuracy);
    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_RTCM_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_RTCM_DATA packet_in{};
    packet_in.flags = 5;
    packet_in.len = 72;
    packet_in.data = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62 }};

    mavlink::common::msg::GPS_RTCM_DATA packet1{};
    mavlink::common::msg::GPS_RTCM_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.len, packet2.len);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_RTCM_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_rtcm_data_t packet_c {
         5, 72, { 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62 }
    };

    mavlink::common::msg::GPS_RTCM_DATA packet_in{};
    packet_in.flags = 5;
    packet_in.len = 72;
    packet_in.data = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62 }};

    mavlink::common::msg::GPS_RTCM_DATA packet2{};

    mavlink_msg_gps_rtcm_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.len, packet2.len);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIGH_LATENCY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIGH_LATENCY packet_in{};
    packet_in.base_mode = 211;
    packet_in.custom_mode = 963497464;
    packet_in.landed_state = 22;
    packet_in.roll = 17859;
    packet_in.pitch = 17963;
    packet_in.heading = 18067;
    packet_in.throttle = 89;
    packet_in.heading_sp = 18171;
    packet_in.latitude = 963497672;
    packet_in.longitude = 963497880;
    packet_in.altitude_amsl = 18275;
    packet_in.altitude_sp = 18379;
    packet_in.airspeed = 156;
    packet_in.airspeed_sp = 223;
    packet_in.groundspeed = 34;
    packet_in.climb_rate = 101;
    packet_in.gps_nsat = 168;
    packet_in.gps_fix_type = 235;
    packet_in.battery_remaining = 46;
    packet_in.temperature = 113;
    packet_in.temperature_air = -76;
    packet_in.failsafe = 247;
    packet_in.wp_num = 58;
    packet_in.wp_distance = 18483;

    mavlink::common::msg::HIGH_LATENCY packet1{};
    mavlink::common::msg::HIGH_LATENCY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.base_mode, packet2.base_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.landed_state, packet2.landed_state);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.heading, packet2.heading);
    EXPECT_EQ(packet1.throttle, packet2.throttle);
    EXPECT_EQ(packet1.heading_sp, packet2.heading_sp);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude_amsl, packet2.altitude_amsl);
    EXPECT_EQ(packet1.altitude_sp, packet2.altitude_sp);
    EXPECT_EQ(packet1.airspeed, packet2.airspeed);
    EXPECT_EQ(packet1.airspeed_sp, packet2.airspeed_sp);
    EXPECT_EQ(packet1.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet1.climb_rate, packet2.climb_rate);
    EXPECT_EQ(packet1.gps_nsat, packet2.gps_nsat);
    EXPECT_EQ(packet1.gps_fix_type, packet2.gps_fix_type);
    EXPECT_EQ(packet1.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.temperature_air, packet2.temperature_air);
    EXPECT_EQ(packet1.failsafe, packet2.failsafe);
    EXPECT_EQ(packet1.wp_num, packet2.wp_num);
    EXPECT_EQ(packet1.wp_distance, packet2.wp_distance);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIGH_LATENCY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_high_latency_t packet_c {
         963497464, 963497672, 963497880, 17859, 17963, 18067, 18171, 18275, 18379, 18483, 211, 22, 89, 156, 223, 34, 101, 168, 235, 46, 113, -76, 247, 58
    };

    mavlink::common::msg::HIGH_LATENCY packet_in{};
    packet_in.base_mode = 211;
    packet_in.custom_mode = 963497464;
    packet_in.landed_state = 22;
    packet_in.roll = 17859;
    packet_in.pitch = 17963;
    packet_in.heading = 18067;
    packet_in.throttle = 89;
    packet_in.heading_sp = 18171;
    packet_in.latitude = 963497672;
    packet_in.longitude = 963497880;
    packet_in.altitude_amsl = 18275;
    packet_in.altitude_sp = 18379;
    packet_in.airspeed = 156;
    packet_in.airspeed_sp = 223;
    packet_in.groundspeed = 34;
    packet_in.climb_rate = 101;
    packet_in.gps_nsat = 168;
    packet_in.gps_fix_type = 235;
    packet_in.battery_remaining = 46;
    packet_in.temperature = 113;
    packet_in.temperature_air = -76;
    packet_in.failsafe = 247;
    packet_in.wp_num = 58;
    packet_in.wp_distance = 18483;

    mavlink::common::msg::HIGH_LATENCY packet2{};

    mavlink_msg_high_latency_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.base_mode, packet2.base_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.landed_state, packet2.landed_state);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.heading, packet2.heading);
    EXPECT_EQ(packet_in.throttle, packet2.throttle);
    EXPECT_EQ(packet_in.heading_sp, packet2.heading_sp);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude_amsl, packet2.altitude_amsl);
    EXPECT_EQ(packet_in.altitude_sp, packet2.altitude_sp);
    EXPECT_EQ(packet_in.airspeed, packet2.airspeed);
    EXPECT_EQ(packet_in.airspeed_sp, packet2.airspeed_sp);
    EXPECT_EQ(packet_in.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet_in.climb_rate, packet2.climb_rate);
    EXPECT_EQ(packet_in.gps_nsat, packet2.gps_nsat);
    EXPECT_EQ(packet_in.gps_fix_type, packet2.gps_fix_type);
    EXPECT_EQ(packet_in.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.temperature_air, packet2.temperature_air);
    EXPECT_EQ(packet_in.failsafe, packet2.failsafe);
    EXPECT_EQ(packet_in.wp_num, packet2.wp_num);
    EXPECT_EQ(packet_in.wp_distance, packet2.wp_distance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HIGH_LATENCY2)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HIGH_LATENCY2 packet_in{};
    packet_in.timestamp = 963497464;
    packet_in.type = 77;
    packet_in.autopilot = 144;
    packet_in.custom_mode = 17859;
    packet_in.latitude = 963497672;
    packet_in.longitude = 963497880;
    packet_in.altitude = 17963;
    packet_in.target_altitude = 18067;
    packet_in.heading = 211;
    packet_in.target_heading = 22;
    packet_in.target_distance = 18171;
    packet_in.throttle = 89;
    packet_in.airspeed = 156;
    packet_in.airspeed_sp = 223;
    packet_in.groundspeed = 34;
    packet_in.windspeed = 101;
    packet_in.wind_heading = 168;
    packet_in.eph = 235;
    packet_in.epv = 46;
    packet_in.temperature_air = 113;
    packet_in.climb_rate = -76;
    packet_in.battery = -9;
    packet_in.wp_num = 18275;
    packet_in.failure_flags = 18379;
    packet_in.custom0 = 58;
    packet_in.custom1 = 125;
    packet_in.custom2 = -64;

    mavlink::common::msg::HIGH_LATENCY2 packet1{};
    mavlink::common::msg::HIGH_LATENCY2 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.autopilot, packet2.autopilot);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.target_altitude, packet2.target_altitude);
    EXPECT_EQ(packet1.heading, packet2.heading);
    EXPECT_EQ(packet1.target_heading, packet2.target_heading);
    EXPECT_EQ(packet1.target_distance, packet2.target_distance);
    EXPECT_EQ(packet1.throttle, packet2.throttle);
    EXPECT_EQ(packet1.airspeed, packet2.airspeed);
    EXPECT_EQ(packet1.airspeed_sp, packet2.airspeed_sp);
    EXPECT_EQ(packet1.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet1.windspeed, packet2.windspeed);
    EXPECT_EQ(packet1.wind_heading, packet2.wind_heading);
    EXPECT_EQ(packet1.eph, packet2.eph);
    EXPECT_EQ(packet1.epv, packet2.epv);
    EXPECT_EQ(packet1.temperature_air, packet2.temperature_air);
    EXPECT_EQ(packet1.climb_rate, packet2.climb_rate);
    EXPECT_EQ(packet1.battery, packet2.battery);
    EXPECT_EQ(packet1.wp_num, packet2.wp_num);
    EXPECT_EQ(packet1.failure_flags, packet2.failure_flags);
    EXPECT_EQ(packet1.custom0, packet2.custom0);
    EXPECT_EQ(packet1.custom1, packet2.custom1);
    EXPECT_EQ(packet1.custom2, packet2.custom2);
}

#ifdef TEST_INTEROP
TEST(common_interop, HIGH_LATENCY2)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_high_latency2_t packet_c {
         963497464, 963497672, 963497880, 17859, 17963, 18067, 18171, 18275, 18379, 77, 144, 211, 22, 89, 156, 223, 34, 101, 168, 235, 46, 113, -76, -9, 58, 125, -64
    };

    mavlink::common::msg::HIGH_LATENCY2 packet_in{};
    packet_in.timestamp = 963497464;
    packet_in.type = 77;
    packet_in.autopilot = 144;
    packet_in.custom_mode = 17859;
    packet_in.latitude = 963497672;
    packet_in.longitude = 963497880;
    packet_in.altitude = 17963;
    packet_in.target_altitude = 18067;
    packet_in.heading = 211;
    packet_in.target_heading = 22;
    packet_in.target_distance = 18171;
    packet_in.throttle = 89;
    packet_in.airspeed = 156;
    packet_in.airspeed_sp = 223;
    packet_in.groundspeed = 34;
    packet_in.windspeed = 101;
    packet_in.wind_heading = 168;
    packet_in.eph = 235;
    packet_in.epv = 46;
    packet_in.temperature_air = 113;
    packet_in.climb_rate = -76;
    packet_in.battery = -9;
    packet_in.wp_num = 18275;
    packet_in.failure_flags = 18379;
    packet_in.custom0 = 58;
    packet_in.custom1 = 125;
    packet_in.custom2 = -64;

    mavlink::common::msg::HIGH_LATENCY2 packet2{};

    mavlink_msg_high_latency2_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.autopilot, packet2.autopilot);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.target_altitude, packet2.target_altitude);
    EXPECT_EQ(packet_in.heading, packet2.heading);
    EXPECT_EQ(packet_in.target_heading, packet2.target_heading);
    EXPECT_EQ(packet_in.target_distance, packet2.target_distance);
    EXPECT_EQ(packet_in.throttle, packet2.throttle);
    EXPECT_EQ(packet_in.airspeed, packet2.airspeed);
    EXPECT_EQ(packet_in.airspeed_sp, packet2.airspeed_sp);
    EXPECT_EQ(packet_in.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet_in.windspeed, packet2.windspeed);
    EXPECT_EQ(packet_in.wind_heading, packet2.wind_heading);
    EXPECT_EQ(packet_in.eph, packet2.eph);
    EXPECT_EQ(packet_in.epv, packet2.epv);
    EXPECT_EQ(packet_in.temperature_air, packet2.temperature_air);
    EXPECT_EQ(packet_in.climb_rate, packet2.climb_rate);
    EXPECT_EQ(packet_in.battery, packet2.battery);
    EXPECT_EQ(packet_in.wp_num, packet2.wp_num);
    EXPECT_EQ(packet_in.failure_flags, packet2.failure_flags);
    EXPECT_EQ(packet_in.custom0, packet2.custom0);
    EXPECT_EQ(packet_in.custom1, packet2.custom1);
    EXPECT_EQ(packet_in.custom2, packet2.custom2);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VIBRATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VIBRATION packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.vibration_x = 73.0;
    packet_in.vibration_y = 101.0;
    packet_in.vibration_z = 129.0;
    packet_in.clipping_0 = 963498504;
    packet_in.clipping_1 = 963498712;
    packet_in.clipping_2 = 963498920;

    mavlink::common::msg::VIBRATION packet1{};
    mavlink::common::msg::VIBRATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.vibration_x, packet2.vibration_x);
    EXPECT_EQ(packet1.vibration_y, packet2.vibration_y);
    EXPECT_EQ(packet1.vibration_z, packet2.vibration_z);
    EXPECT_EQ(packet1.clipping_0, packet2.clipping_0);
    EXPECT_EQ(packet1.clipping_1, packet2.clipping_1);
    EXPECT_EQ(packet1.clipping_2, packet2.clipping_2);
}

#ifdef TEST_INTEROP
TEST(common_interop, VIBRATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vibration_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 963498504, 963498712, 963498920
    };

    mavlink::common::msg::VIBRATION packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.vibration_x = 73.0;
    packet_in.vibration_y = 101.0;
    packet_in.vibration_z = 129.0;
    packet_in.clipping_0 = 963498504;
    packet_in.clipping_1 = 963498712;
    packet_in.clipping_2 = 963498920;

    mavlink::common::msg::VIBRATION packet2{};

    mavlink_msg_vibration_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.vibration_x, packet2.vibration_x);
    EXPECT_EQ(packet_in.vibration_y, packet2.vibration_y);
    EXPECT_EQ(packet_in.vibration_z, packet2.vibration_z);
    EXPECT_EQ(packet_in.clipping_0, packet2.clipping_0);
    EXPECT_EQ(packet_in.clipping_1, packet2.clipping_1);
    EXPECT_EQ(packet_in.clipping_2, packet2.clipping_2);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HOME_POSITION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HOME_POSITION packet_in{};
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;
    packet_in.time_usec = 93372036854779083ULL;

    mavlink::common::msg::HOME_POSITION packet1{};
    mavlink::common::msg::HOME_POSITION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.approach_x, packet2.approach_x);
    EXPECT_EQ(packet1.approach_y, packet2.approach_y);
    EXPECT_EQ(packet1.approach_z, packet2.approach_z);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
}

#ifdef TEST_INTEROP
TEST(common_interop, HOME_POSITION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_home_position_t packet_c {
         963497464, 963497672, 963497880, 101.0, 129.0, 157.0, { 185.0, 186.0, 187.0, 188.0 }, 297.0, 325.0, 353.0, 93372036854779083ULL
    };

    mavlink::common::msg::HOME_POSITION packet_in{};
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;
    packet_in.time_usec = 93372036854779083ULL;

    mavlink::common::msg::HOME_POSITION packet2{};

    mavlink_msg_home_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.approach_x, packet2.approach_x);
    EXPECT_EQ(packet_in.approach_y, packet2.approach_y);
    EXPECT_EQ(packet_in.approach_z, packet2.approach_z);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_HOME_POSITION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_HOME_POSITION packet_in{};
    packet_in.target_system = 161;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;
    packet_in.time_usec = 93372036854779146ULL;

    mavlink::common::msg::SET_HOME_POSITION packet1{};
    mavlink::common::msg::SET_HOME_POSITION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.approach_x, packet2.approach_x);
    EXPECT_EQ(packet1.approach_y, packet2.approach_y);
    EXPECT_EQ(packet1.approach_z, packet2.approach_z);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_HOME_POSITION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_home_position_t packet_c {
         963497464, 963497672, 963497880, 101.0, 129.0, 157.0, { 185.0, 186.0, 187.0, 188.0 }, 297.0, 325.0, 353.0, 161, 93372036854779146ULL
    };

    mavlink::common::msg::SET_HOME_POSITION packet_in{};
    packet_in.target_system = 161;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;
    packet_in.time_usec = 93372036854779146ULL;

    mavlink::common::msg::SET_HOME_POSITION packet2{};

    mavlink_msg_set_home_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.approach_x, packet2.approach_x);
    EXPECT_EQ(packet_in.approach_y, packet2.approach_y);
    EXPECT_EQ(packet_in.approach_z, packet2.approach_z);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MESSAGE_INTERVAL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MESSAGE_INTERVAL packet_in{};
    packet_in.message_id = 17443;
    packet_in.interval_us = 963497464;

    mavlink::common::msg::MESSAGE_INTERVAL packet1{};
    mavlink::common::msg::MESSAGE_INTERVAL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.message_id, packet2.message_id);
    EXPECT_EQ(packet1.interval_us, packet2.interval_us);
}

#ifdef TEST_INTEROP
TEST(common_interop, MESSAGE_INTERVAL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_message_interval_t packet_c {
         963497464, 17443
    };

    mavlink::common::msg::MESSAGE_INTERVAL packet_in{};
    packet_in.message_id = 17443;
    packet_in.interval_us = 963497464;

    mavlink::common::msg::MESSAGE_INTERVAL packet2{};

    mavlink_msg_message_interval_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.message_id, packet2.message_id);
    EXPECT_EQ(packet_in.interval_us, packet2.interval_us);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, EXTENDED_SYS_STATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::EXTENDED_SYS_STATE packet_in{};
    packet_in.vtol_state = 5;
    packet_in.landed_state = 72;

    mavlink::common::msg::EXTENDED_SYS_STATE packet1{};
    mavlink::common::msg::EXTENDED_SYS_STATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.vtol_state, packet2.vtol_state);
    EXPECT_EQ(packet1.landed_state, packet2.landed_state);
}

#ifdef TEST_INTEROP
TEST(common_interop, EXTENDED_SYS_STATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_extended_sys_state_t packet_c {
         5, 72
    };

    mavlink::common::msg::EXTENDED_SYS_STATE packet_in{};
    packet_in.vtol_state = 5;
    packet_in.landed_state = 72;

    mavlink::common::msg::EXTENDED_SYS_STATE packet2{};

    mavlink_msg_extended_sys_state_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.vtol_state, packet2.vtol_state);
    EXPECT_EQ(packet_in.landed_state, packet2.landed_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ADSB_VEHICLE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ADSB_VEHICLE packet_in{};
    packet_in.ICAO_address = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.altitude_type = 211;
    packet_in.altitude = 963498088;
    packet_in.heading = 18067;
    packet_in.hor_velocity = 18171;
    packet_in.ver_velocity = 18275;
    packet_in.callsign = to_char_array("BCDEFGHI");
    packet_in.emitter_type = 113;
    packet_in.tslc = 180;
    packet_in.flags = 18379;
    packet_in.squawk = 18483;

    mavlink::common::msg::ADSB_VEHICLE packet1{};
    mavlink::common::msg::ADSB_VEHICLE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.ICAO_address, packet2.ICAO_address);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.altitude_type, packet2.altitude_type);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.heading, packet2.heading);
    EXPECT_EQ(packet1.hor_velocity, packet2.hor_velocity);
    EXPECT_EQ(packet1.ver_velocity, packet2.ver_velocity);
    EXPECT_EQ(packet1.callsign, packet2.callsign);
    EXPECT_EQ(packet1.emitter_type, packet2.emitter_type);
    EXPECT_EQ(packet1.tslc, packet2.tslc);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.squawk, packet2.squawk);
}

#ifdef TEST_INTEROP
TEST(common_interop, ADSB_VEHICLE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_adsb_vehicle_t packet_c {
         963497464, 963497672, 963497880, 963498088, 18067, 18171, 18275, 18379, 18483, 211, "BCDEFGHI", 113, 180
    };

    mavlink::common::msg::ADSB_VEHICLE packet_in{};
    packet_in.ICAO_address = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.altitude_type = 211;
    packet_in.altitude = 963498088;
    packet_in.heading = 18067;
    packet_in.hor_velocity = 18171;
    packet_in.ver_velocity = 18275;
    packet_in.callsign = to_char_array("BCDEFGHI");
    packet_in.emitter_type = 113;
    packet_in.tslc = 180;
    packet_in.flags = 18379;
    packet_in.squawk = 18483;

    mavlink::common::msg::ADSB_VEHICLE packet2{};

    mavlink_msg_adsb_vehicle_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.ICAO_address, packet2.ICAO_address);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.altitude_type, packet2.altitude_type);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.heading, packet2.heading);
    EXPECT_EQ(packet_in.hor_velocity, packet2.hor_velocity);
    EXPECT_EQ(packet_in.ver_velocity, packet2.ver_velocity);
    EXPECT_EQ(packet_in.callsign, packet2.callsign);
    EXPECT_EQ(packet_in.emitter_type, packet2.emitter_type);
    EXPECT_EQ(packet_in.tslc, packet2.tslc);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.squawk, packet2.squawk);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COLLISION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COLLISION packet_in{};
    packet_in.src = 53;
    packet_in.id = 963497464;
    packet_in.action = 120;
    packet_in.threat_level = 187;
    packet_in.time_to_minimum_delta = 45.0;
    packet_in.altitude_minimum_delta = 73.0;
    packet_in.horizontal_minimum_delta = 101.0;

    mavlink::common::msg::COLLISION packet1{};
    mavlink::common::msg::COLLISION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.src, packet2.src);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.action, packet2.action);
    EXPECT_EQ(packet1.threat_level, packet2.threat_level);
    EXPECT_EQ(packet1.time_to_minimum_delta, packet2.time_to_minimum_delta);
    EXPECT_EQ(packet1.altitude_minimum_delta, packet2.altitude_minimum_delta);
    EXPECT_EQ(packet1.horizontal_minimum_delta, packet2.horizontal_minimum_delta);
}

#ifdef TEST_INTEROP
TEST(common_interop, COLLISION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_collision_t packet_c {
         963497464, 45.0, 73.0, 101.0, 53, 120, 187
    };

    mavlink::common::msg::COLLISION packet_in{};
    packet_in.src = 53;
    packet_in.id = 963497464;
    packet_in.action = 120;
    packet_in.threat_level = 187;
    packet_in.time_to_minimum_delta = 45.0;
    packet_in.altitude_minimum_delta = 73.0;
    packet_in.horizontal_minimum_delta = 101.0;

    mavlink::common::msg::COLLISION packet2{};

    mavlink_msg_collision_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.src, packet2.src);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.action, packet2.action);
    EXPECT_EQ(packet_in.threat_level, packet2.threat_level);
    EXPECT_EQ(packet_in.time_to_minimum_delta, packet2.time_to_minimum_delta);
    EXPECT_EQ(packet_in.altitude_minimum_delta, packet2.altitude_minimum_delta);
    EXPECT_EQ(packet_in.horizontal_minimum_delta, packet2.horizontal_minimum_delta);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, V2_EXTENSION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::V2_EXTENSION packet_in{};
    packet_in.target_network = 139;
    packet_in.target_system = 206;
    packet_in.target_component = 17;
    packet_in.message_type = 17235;
    packet_in.payload = {{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76 }};

    mavlink::common::msg::V2_EXTENSION packet1{};
    mavlink::common::msg::V2_EXTENSION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_network, packet2.target_network);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.message_type, packet2.message_type);
    EXPECT_EQ(packet1.payload, packet2.payload);
}

#ifdef TEST_INTEROP
TEST(common_interop, V2_EXTENSION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_v2_extension_t packet_c {
         17235, 139, 206, 17, { 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76 }
    };

    mavlink::common::msg::V2_EXTENSION packet_in{};
    packet_in.target_network = 139;
    packet_in.target_system = 206;
    packet_in.target_component = 17;
    packet_in.message_type = 17235;
    packet_in.payload = {{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76 }};

    mavlink::common::msg::V2_EXTENSION packet2{};

    mavlink_msg_v2_extension_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_network, packet2.target_network);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.message_type, packet2.message_type);
    EXPECT_EQ(packet_in.payload, packet2.payload);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MEMORY_VECT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MEMORY_VECT packet_in{};
    packet_in.address = 17235;
    packet_in.ver = 139;
    packet_in.type = 206;
    packet_in.value = {{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48 }};

    mavlink::common::msg::MEMORY_VECT packet1{};
    mavlink::common::msg::MEMORY_VECT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.address, packet2.address);
    EXPECT_EQ(packet1.ver, packet2.ver);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.value, packet2.value);
}

#ifdef TEST_INTEROP
TEST(common_interop, MEMORY_VECT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_memory_vect_t packet_c {
         17235, 139, 206, { 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48 }
    };

    mavlink::common::msg::MEMORY_VECT packet_in{};
    packet_in.address = 17235;
    packet_in.ver = 139;
    packet_in.type = 206;
    packet_in.value = {{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48 }};

    mavlink::common::msg::MEMORY_VECT packet2{};

    mavlink_msg_memory_vect_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.address, packet2.address);
    EXPECT_EQ(packet_in.ver, packet2.ver);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.value, packet2.value);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, DEBUG_VECT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::DEBUG_VECT packet_in{};
    packet_in.name = to_char_array("UVWXYZABC");
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;

    mavlink::common::msg::DEBUG_VECT packet1{};
    mavlink::common::msg::DEBUG_VECT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(common_interop, DEBUG_VECT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_debug_vect_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, "UVWXYZABC"
    };

    mavlink::common::msg::DEBUG_VECT packet_in{};
    packet_in.name = to_char_array("UVWXYZABC");
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;

    mavlink::common::msg::DEBUG_VECT packet2{};

    mavlink_msg_debug_vect_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, NAMED_VALUE_FLOAT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::NAMED_VALUE_FLOAT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.name = to_char_array("IJKLMNOPQ");
    packet_in.value = 45.0;

    mavlink::common::msg::NAMED_VALUE_FLOAT packet1{};
    mavlink::common::msg::NAMED_VALUE_FLOAT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.value, packet2.value);
}

#ifdef TEST_INTEROP
TEST(common_interop, NAMED_VALUE_FLOAT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_named_value_float_t packet_c {
         963497464, 45.0, "IJKLMNOPQ"
    };

    mavlink::common::msg::NAMED_VALUE_FLOAT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.name = to_char_array("IJKLMNOPQ");
    packet_in.value = 45.0;

    mavlink::common::msg::NAMED_VALUE_FLOAT packet2{};

    mavlink_msg_named_value_float_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.value, packet2.value);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, NAMED_VALUE_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::NAMED_VALUE_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.name = to_char_array("IJKLMNOPQ");
    packet_in.value = 963497672;

    mavlink::common::msg::NAMED_VALUE_INT packet1{};
    mavlink::common::msg::NAMED_VALUE_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.value, packet2.value);
}

#ifdef TEST_INTEROP
TEST(common_interop, NAMED_VALUE_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_named_value_int_t packet_c {
         963497464, 963497672, "IJKLMNOPQ"
    };

    mavlink::common::msg::NAMED_VALUE_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.name = to_char_array("IJKLMNOPQ");
    packet_in.value = 963497672;

    mavlink::common::msg::NAMED_VALUE_INT packet2{};

    mavlink_msg_named_value_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.value, packet2.value);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, STATUSTEXT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::STATUSTEXT packet_in{};
    packet_in.severity = 5;
    packet_in.text = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX");
    packet_in.id = 19887;
    packet_in.chunk_seq = 228;

    mavlink::common::msg::STATUSTEXT packet1{};
    mavlink::common::msg::STATUSTEXT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.severity, packet2.severity);
    EXPECT_EQ(packet1.text, packet2.text);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.chunk_seq, packet2.chunk_seq);
}

#ifdef TEST_INTEROP
TEST(common_interop, STATUSTEXT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_statustext_t packet_c {
         5, "BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX", 19887, 228
    };

    mavlink::common::msg::STATUSTEXT packet_in{};
    packet_in.severity = 5;
    packet_in.text = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX");
    packet_in.id = 19887;
    packet_in.chunk_seq = 228;

    mavlink::common::msg::STATUSTEXT packet2{};

    mavlink_msg_statustext_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.severity, packet2.severity);
    EXPECT_EQ(packet_in.text, packet2.text);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.chunk_seq, packet2.chunk_seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, DEBUG)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::DEBUG packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.ind = 29;
    packet_in.value = 45.0;

    mavlink::common::msg::DEBUG packet1{};
    mavlink::common::msg::DEBUG packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.ind, packet2.ind);
    EXPECT_EQ(packet1.value, packet2.value);
}

#ifdef TEST_INTEROP
TEST(common_interop, DEBUG)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_debug_t packet_c {
         963497464, 45.0, 29
    };

    mavlink::common::msg::DEBUG packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.ind = 29;
    packet_in.value = 45.0;

    mavlink::common::msg::DEBUG packet2{};

    mavlink_msg_debug_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.ind, packet2.ind);
    EXPECT_EQ(packet_in.value, packet2.value);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SETUP_SIGNING)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SETUP_SIGNING packet_in{};
    packet_in.target_system = 29;
    packet_in.target_component = 96;
    packet_in.secret_key = {{ 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194 }};
    packet_in.initial_timestamp = 93372036854775807ULL;

    mavlink::common::msg::SETUP_SIGNING packet1{};
    mavlink::common::msg::SETUP_SIGNING packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.secret_key, packet2.secret_key);
    EXPECT_EQ(packet1.initial_timestamp, packet2.initial_timestamp);
}

#ifdef TEST_INTEROP
TEST(common_interop, SETUP_SIGNING)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_setup_signing_t packet_c {
         93372036854775807ULL, 29, 96, { 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194 }
    };

    mavlink::common::msg::SETUP_SIGNING packet_in{};
    packet_in.target_system = 29;
    packet_in.target_component = 96;
    packet_in.secret_key = {{ 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194 }};
    packet_in.initial_timestamp = 93372036854775807ULL;

    mavlink::common::msg::SETUP_SIGNING packet2{};

    mavlink_msg_setup_signing_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.secret_key, packet2.secret_key);
    EXPECT_EQ(packet_in.initial_timestamp, packet2.initial_timestamp);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, BUTTON_CHANGE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::BUTTON_CHANGE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.last_change_ms = 963497672;
    packet_in.state = 29;

    mavlink::common::msg::BUTTON_CHANGE packet1{};
    mavlink::common::msg::BUTTON_CHANGE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.last_change_ms, packet2.last_change_ms);
    EXPECT_EQ(packet1.state, packet2.state);
}

#ifdef TEST_INTEROP
TEST(common_interop, BUTTON_CHANGE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_button_change_t packet_c {
         963497464, 963497672, 29
    };

    mavlink::common::msg::BUTTON_CHANGE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.last_change_ms = 963497672;
    packet_in.state = 29;

    mavlink::common::msg::BUTTON_CHANGE packet2{};

    mavlink_msg_button_change_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.last_change_ms, packet2.last_change_ms);
    EXPECT_EQ(packet_in.state, packet2.state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PLAY_TUNE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PLAY_TUNE packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.tune = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDE");
    packet_in.tune2 = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW");

    mavlink::common::msg::PLAY_TUNE packet1{};
    mavlink::common::msg::PLAY_TUNE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.tune, packet2.tune);
    EXPECT_EQ(packet1.tune2, packet2.tune2);
}

#ifdef TEST_INTEROP
TEST(common_interop, PLAY_TUNE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_play_tune_t packet_c {
         5, 72, "CDEFGHIJKLMNOPQRSTUVWXYZABCDE", "GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };

    mavlink::common::msg::PLAY_TUNE packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.tune = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDE");
    packet_in.tune2 = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW");

    mavlink::common::msg::PLAY_TUNE packet2{};

    mavlink_msg_play_tune_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.tune, packet2.tune);
    EXPECT_EQ(packet_in.tune2, packet2.tune2);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.vendor_name = {{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254 }};
    packet_in.model_name = {{ 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94 }};
    packet_in.firmware_version = 963497672;
    packet_in.focal_length = 73.0;
    packet_in.sensor_size_h = 101.0;
    packet_in.sensor_size_v = 129.0;
    packet_in.resolution_h = 18483;
    packet_in.resolution_v = 18587;
    packet_in.lens_id = 159;
    packet_in.flags = 963498504;
    packet_in.cam_definition_version = 18691;
    packet_in.cam_definition_uri = to_char_array("RSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ");
    packet_in.gimbal_device_id = 134;
    packet_in.camera_device_id = 201;

    mavlink::common::msg::CAMERA_INFORMATION packet1{};
    mavlink::common::msg::CAMERA_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet1.model_name, packet2.model_name);
    EXPECT_EQ(packet1.firmware_version, packet2.firmware_version);
    EXPECT_EQ(packet1.focal_length, packet2.focal_length);
    EXPECT_EQ(packet1.sensor_size_h, packet2.sensor_size_h);
    EXPECT_EQ(packet1.sensor_size_v, packet2.sensor_size_v);
    EXPECT_EQ(packet1.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet1.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet1.lens_id, packet2.lens_id);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.cam_definition_version, packet2.cam_definition_version);
    EXPECT_EQ(packet1.cam_definition_uri, packet2.cam_definition_uri);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_information_t packet_c {
         963497464, 963497672, 73.0, 101.0, 129.0, 963498504, 18483, 18587, 18691, { 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254 }, { 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94 }, 159, "RSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ", 134, 201
    };

    mavlink::common::msg::CAMERA_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.vendor_name = {{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254 }};
    packet_in.model_name = {{ 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94 }};
    packet_in.firmware_version = 963497672;
    packet_in.focal_length = 73.0;
    packet_in.sensor_size_h = 101.0;
    packet_in.sensor_size_v = 129.0;
    packet_in.resolution_h = 18483;
    packet_in.resolution_v = 18587;
    packet_in.lens_id = 159;
    packet_in.flags = 963498504;
    packet_in.cam_definition_version = 18691;
    packet_in.cam_definition_uri = to_char_array("RSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ");
    packet_in.gimbal_device_id = 134;
    packet_in.camera_device_id = 201;

    mavlink::common::msg::CAMERA_INFORMATION packet2{};

    mavlink_msg_camera_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet_in.model_name, packet2.model_name);
    EXPECT_EQ(packet_in.firmware_version, packet2.firmware_version);
    EXPECT_EQ(packet_in.focal_length, packet2.focal_length);
    EXPECT_EQ(packet_in.sensor_size_h, packet2.sensor_size_h);
    EXPECT_EQ(packet_in.sensor_size_v, packet2.sensor_size_v);
    EXPECT_EQ(packet_in.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet_in.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet_in.lens_id, packet2.lens_id);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.cam_definition_version, packet2.cam_definition_version);
    EXPECT_EQ(packet_in.cam_definition_uri, packet2.cam_definition_uri);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_SETTINGS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_SETTINGS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.mode_id = 17;
    packet_in.zoomLevel = 52.0;
    packet_in.focusLevel = 80.0;
    packet_in.camera_device_id = 108;

    mavlink::common::msg::CAMERA_SETTINGS packet1{};
    mavlink::common::msg::CAMERA_SETTINGS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.mode_id, packet2.mode_id);
    EXPECT_EQ(packet1.zoomLevel, packet2.zoomLevel);
    EXPECT_EQ(packet1.focusLevel, packet2.focusLevel);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_SETTINGS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_settings_t packet_c {
         963497464, 17, 52.0, 80.0, 108
    };

    mavlink::common::msg::CAMERA_SETTINGS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.mode_id = 17;
    packet_in.zoomLevel = 52.0;
    packet_in.focusLevel = 80.0;
    packet_in.camera_device_id = 108;

    mavlink::common::msg::CAMERA_SETTINGS packet2{};

    mavlink_msg_camera_settings_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.mode_id, packet2.mode_id);
    EXPECT_EQ(packet_in.zoomLevel, packet2.zoomLevel);
    EXPECT_EQ(packet_in.focusLevel, packet2.focusLevel);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, STORAGE_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::STORAGE_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.storage_id = 77;
    packet_in.storage_count = 144;
    packet_in.status = 211;
    packet_in.total_capacity = 45.0;
    packet_in.used_capacity = 73.0;
    packet_in.available_capacity = 101.0;
    packet_in.read_speed = 129.0;
    packet_in.write_speed = 157.0;
    packet_in.type = 22;
    packet_in.name = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.storage_usage = 185;

    mavlink::common::msg::STORAGE_INFORMATION packet1{};
    mavlink::common::msg::STORAGE_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.storage_id, packet2.storage_id);
    EXPECT_EQ(packet1.storage_count, packet2.storage_count);
    EXPECT_EQ(packet1.status, packet2.status);
    EXPECT_EQ(packet1.total_capacity, packet2.total_capacity);
    EXPECT_EQ(packet1.used_capacity, packet2.used_capacity);
    EXPECT_EQ(packet1.available_capacity, packet2.available_capacity);
    EXPECT_EQ(packet1.read_speed, packet2.read_speed);
    EXPECT_EQ(packet1.write_speed, packet2.write_speed);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.storage_usage, packet2.storage_usage);
}

#ifdef TEST_INTEROP
TEST(common_interop, STORAGE_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_storage_information_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 77, 144, 211, 22, "CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG", 185
    };

    mavlink::common::msg::STORAGE_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.storage_id = 77;
    packet_in.storage_count = 144;
    packet_in.status = 211;
    packet_in.total_capacity = 45.0;
    packet_in.used_capacity = 73.0;
    packet_in.available_capacity = 101.0;
    packet_in.read_speed = 129.0;
    packet_in.write_speed = 157.0;
    packet_in.type = 22;
    packet_in.name = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.storage_usage = 185;

    mavlink::common::msg::STORAGE_INFORMATION packet2{};

    mavlink_msg_storage_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.storage_id, packet2.storage_id);
    EXPECT_EQ(packet_in.storage_count, packet2.storage_count);
    EXPECT_EQ(packet_in.status, packet2.status);
    EXPECT_EQ(packet_in.total_capacity, packet2.total_capacity);
    EXPECT_EQ(packet_in.used_capacity, packet2.used_capacity);
    EXPECT_EQ(packet_in.available_capacity, packet2.available_capacity);
    EXPECT_EQ(packet_in.read_speed, packet2.read_speed);
    EXPECT_EQ(packet_in.write_speed, packet2.write_speed);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.storage_usage, packet2.storage_usage);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_CAPTURE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.image_status = 53;
    packet_in.video_status = 120;
    packet_in.image_interval = 45.0;
    packet_in.recording_time_ms = 963497880;
    packet_in.available_capacity = 101.0;
    packet_in.image_count = 963498400;
    packet_in.camera_device_id = 199;

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet1{};
    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.image_status, packet2.image_status);
    EXPECT_EQ(packet1.video_status, packet2.video_status);
    EXPECT_EQ(packet1.image_interval, packet2.image_interval);
    EXPECT_EQ(packet1.recording_time_ms, packet2.recording_time_ms);
    EXPECT_EQ(packet1.available_capacity, packet2.available_capacity);
    EXPECT_EQ(packet1.image_count, packet2.image_count);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_CAPTURE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_capture_status_t packet_c {
         963497464, 45.0, 963497880, 101.0, 53, 120, 963498400, 199
    };

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.image_status = 53;
    packet_in.video_status = 120;
    packet_in.image_interval = 45.0;
    packet_in.recording_time_ms = 963497880;
    packet_in.available_capacity = 101.0;
    packet_in.image_count = 963498400;
    packet_in.camera_device_id = 199;

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet2{};

    mavlink_msg_camera_capture_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.image_status, packet2.image_status);
    EXPECT_EQ(packet_in.video_status, packet2.video_status);
    EXPECT_EQ(packet_in.image_interval, packet2.image_interval);
    EXPECT_EQ(packet_in.recording_time_ms, packet2.recording_time_ms);
    EXPECT_EQ(packet_in.available_capacity, packet2.available_capacity);
    EXPECT_EQ(packet_in.image_count, packet2.image_count);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_IMAGE_CAPTURED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.time_utc = 93372036854775807ULL;
    packet_in.camera_id = 149;
    packet_in.lat = 963498088;
    packet_in.lon = 963498296;
    packet_in.alt = 963498504;
    packet_in.relative_alt = 963498712;
    packet_in.q = {{ 213.0, 214.0, 215.0, 216.0 }};
    packet_in.image_index = 963499752;
    packet_in.capture_result = -40;
    packet_in.file_url = to_char_array("YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST");

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet1{};
    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.time_utc, packet2.time_utc);
    EXPECT_EQ(packet1.camera_id, packet2.camera_id);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.image_index, packet2.image_index);
    EXPECT_EQ(packet1.capture_result, packet2.capture_result);
    EXPECT_EQ(packet1.file_url, packet2.file_url);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_IMAGE_CAPTURED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_image_captured_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 963498504, 963498712, { 213.0, 214.0, 215.0, 216.0 }, 963499752, 149, -40, "YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST"
    };

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.time_utc = 93372036854775807ULL;
    packet_in.camera_id = 149;
    packet_in.lat = 963498088;
    packet_in.lon = 963498296;
    packet_in.alt = 963498504;
    packet_in.relative_alt = 963498712;
    packet_in.q = {{ 213.0, 214.0, 215.0, 216.0 }};
    packet_in.image_index = 963499752;
    packet_in.capture_result = -40;
    packet_in.file_url = to_char_array("YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST");

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet2{};

    mavlink_msg_camera_image_captured_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.time_utc, packet2.time_utc);
    EXPECT_EQ(packet_in.camera_id, packet2.camera_id);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.image_index, packet2.image_index);
    EXPECT_EQ(packet_in.capture_result, packet2.capture_result);
    EXPECT_EQ(packet_in.file_url, packet2.file_url);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, FLIGHT_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::FLIGHT_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963498712;
    packet_in.arming_time_utc = 93372036854775807ULL;
    packet_in.takeoff_time_utc = 93372036854776311ULL;
    packet_in.flight_uuid = 93372036854776815ULL;
    packet_in.landing_time = 963498920;

    mavlink::common::msg::FLIGHT_INFORMATION packet1{};
    mavlink::common::msg::FLIGHT_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.arming_time_utc, packet2.arming_time_utc);
    EXPECT_EQ(packet1.takeoff_time_utc, packet2.takeoff_time_utc);
    EXPECT_EQ(packet1.flight_uuid, packet2.flight_uuid);
    EXPECT_EQ(packet1.landing_time, packet2.landing_time);
}

#ifdef TEST_INTEROP
TEST(common_interop, FLIGHT_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_flight_information_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 93372036854776815ULL, 963498712, 963498920
    };

    mavlink::common::msg::FLIGHT_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963498712;
    packet_in.arming_time_utc = 93372036854775807ULL;
    packet_in.takeoff_time_utc = 93372036854776311ULL;
    packet_in.flight_uuid = 93372036854776815ULL;
    packet_in.landing_time = 963498920;

    mavlink::common::msg::FLIGHT_INFORMATION packet2{};

    mavlink_msg_flight_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.arming_time_utc, packet2.arming_time_utc);
    EXPECT_EQ(packet_in.takeoff_time_utc, packet2.takeoff_time_utc);
    EXPECT_EQ(packet_in.flight_uuid, packet2.flight_uuid);
    EXPECT_EQ(packet_in.landing_time, packet2.landing_time);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MOUNT_ORIENTATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MOUNT_ORIENTATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.yaw_absolute = 129.0;

    mavlink::common::msg::MOUNT_ORIENTATION packet1{};
    mavlink::common::msg::MOUNT_ORIENTATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.yaw_absolute, packet2.yaw_absolute);
}

#ifdef TEST_INTEROP
TEST(common_interop, MOUNT_ORIENTATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mount_orientation_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0
    };

    mavlink::common::msg::MOUNT_ORIENTATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.yaw_absolute = 129.0;

    mavlink::common::msg::MOUNT_ORIENTATION packet2{};

    mavlink_msg_mount_orientation_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.yaw_absolute, packet2.yaw_absolute);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOGGING_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOGGING_DATA packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.sequence = 17235;
    packet_in.length = 17;
    packet_in.first_message_offset = 84;
    packet_in.data = {{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }};

    mavlink::common::msg::LOGGING_DATA packet1{};
    mavlink::common::msg::LOGGING_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.sequence, packet2.sequence);
    EXPECT_EQ(packet1.length, packet2.length);
    EXPECT_EQ(packet1.first_message_offset, packet2.first_message_offset);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOGGING_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_logging_data_t packet_c {
         17235, 139, 206, 17, 84, { 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }
    };

    mavlink::common::msg::LOGGING_DATA packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.sequence = 17235;
    packet_in.length = 17;
    packet_in.first_message_offset = 84;
    packet_in.data = {{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }};

    mavlink::common::msg::LOGGING_DATA packet2{};

    mavlink_msg_logging_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.sequence, packet2.sequence);
    EXPECT_EQ(packet_in.length, packet2.length);
    EXPECT_EQ(packet_in.first_message_offset, packet2.first_message_offset);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOGGING_DATA_ACKED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOGGING_DATA_ACKED packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.sequence = 17235;
    packet_in.length = 17;
    packet_in.first_message_offset = 84;
    packet_in.data = {{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }};

    mavlink::common::msg::LOGGING_DATA_ACKED packet1{};
    mavlink::common::msg::LOGGING_DATA_ACKED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.sequence, packet2.sequence);
    EXPECT_EQ(packet1.length, packet2.length);
    EXPECT_EQ(packet1.first_message_offset, packet2.first_message_offset);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOGGING_DATA_ACKED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_logging_data_acked_t packet_c {
         17235, 139, 206, 17, 84, { 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }
    };

    mavlink::common::msg::LOGGING_DATA_ACKED packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.sequence = 17235;
    packet_in.length = 17;
    packet_in.first_message_offset = 84;
    packet_in.data = {{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }};

    mavlink::common::msg::LOGGING_DATA_ACKED packet2{};

    mavlink_msg_logging_data_acked_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.sequence, packet2.sequence);
    EXPECT_EQ(packet_in.length, packet2.length);
    EXPECT_EQ(packet_in.first_message_offset, packet2.first_message_offset);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOGGING_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOGGING_ACK packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.sequence = 17235;

    mavlink::common::msg::LOGGING_ACK packet1{};
    mavlink::common::msg::LOGGING_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.sequence, packet2.sequence);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOGGING_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_logging_ack_t packet_c {
         17235, 139, 206
    };

    mavlink::common::msg::LOGGING_ACK packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.sequence = 17235;

    mavlink::common::msg::LOGGING_ACK packet2{};

    mavlink_msg_logging_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.sequence, packet2.sequence);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VIDEO_STREAM_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VIDEO_STREAM_INFORMATION packet_in{};
    packet_in.stream_id = 187;
    packet_in.count = 254;
    packet_in.type = 65;
    packet_in.flags = 17651;
    packet_in.framerate = 17.0;
    packet_in.resolution_h = 17755;
    packet_in.resolution_v = 17859;
    packet_in.bitrate = 963497672;
    packet_in.rotation = 17963;
    packet_in.hfov = 18067;
    packet_in.name = to_char_array("VWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ");
    packet_in.uri = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCD");
    packet_in.encoding = 196;
    packet_in.camera_device_id = 7;

    mavlink::common::msg::VIDEO_STREAM_INFORMATION packet1{};
    mavlink::common::msg::VIDEO_STREAM_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.stream_id, packet2.stream_id);
    EXPECT_EQ(packet1.count, packet2.count);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.framerate, packet2.framerate);
    EXPECT_EQ(packet1.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet1.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet1.bitrate, packet2.bitrate);
    EXPECT_EQ(packet1.rotation, packet2.rotation);
    EXPECT_EQ(packet1.hfov, packet2.hfov);
    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.uri, packet2.uri);
    EXPECT_EQ(packet1.encoding, packet2.encoding);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, VIDEO_STREAM_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_video_stream_information_t packet_c {
         17.0, 963497672, 17651, 17755, 17859, 17963, 18067, 187, 254, 65, "VWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ", "BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCD", 196, 7
    };

    mavlink::common::msg::VIDEO_STREAM_INFORMATION packet_in{};
    packet_in.stream_id = 187;
    packet_in.count = 254;
    packet_in.type = 65;
    packet_in.flags = 17651;
    packet_in.framerate = 17.0;
    packet_in.resolution_h = 17755;
    packet_in.resolution_v = 17859;
    packet_in.bitrate = 963497672;
    packet_in.rotation = 17963;
    packet_in.hfov = 18067;
    packet_in.name = to_char_array("VWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ");
    packet_in.uri = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCD");
    packet_in.encoding = 196;
    packet_in.camera_device_id = 7;

    mavlink::common::msg::VIDEO_STREAM_INFORMATION packet2{};

    mavlink_msg_video_stream_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.stream_id, packet2.stream_id);
    EXPECT_EQ(packet_in.count, packet2.count);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.framerate, packet2.framerate);
    EXPECT_EQ(packet_in.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet_in.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet_in.bitrate, packet2.bitrate);
    EXPECT_EQ(packet_in.rotation, packet2.rotation);
    EXPECT_EQ(packet_in.hfov, packet2.hfov);
    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.uri, packet2.uri);
    EXPECT_EQ(packet_in.encoding, packet2.encoding);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VIDEO_STREAM_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VIDEO_STREAM_STATUS packet_in{};
    packet_in.stream_id = 187;
    packet_in.flags = 17651;
    packet_in.framerate = 17.0;
    packet_in.resolution_h = 17755;
    packet_in.resolution_v = 17859;
    packet_in.bitrate = 963497672;
    packet_in.rotation = 17963;
    packet_in.hfov = 18067;
    packet_in.camera_device_id = 254;

    mavlink::common::msg::VIDEO_STREAM_STATUS packet1{};
    mavlink::common::msg::VIDEO_STREAM_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.stream_id, packet2.stream_id);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.framerate, packet2.framerate);
    EXPECT_EQ(packet1.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet1.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet1.bitrate, packet2.bitrate);
    EXPECT_EQ(packet1.rotation, packet2.rotation);
    EXPECT_EQ(packet1.hfov, packet2.hfov);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, VIDEO_STREAM_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_video_stream_status_t packet_c {
         17.0, 963497672, 17651, 17755, 17859, 17963, 18067, 187, 254
    };

    mavlink::common::msg::VIDEO_STREAM_STATUS packet_in{};
    packet_in.stream_id = 187;
    packet_in.flags = 17651;
    packet_in.framerate = 17.0;
    packet_in.resolution_h = 17755;
    packet_in.resolution_v = 17859;
    packet_in.bitrate = 963497672;
    packet_in.rotation = 17963;
    packet_in.hfov = 18067;
    packet_in.camera_device_id = 254;

    mavlink::common::msg::VIDEO_STREAM_STATUS packet2{};

    mavlink_msg_video_stream_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.stream_id, packet2.stream_id);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.framerate, packet2.framerate);
    EXPECT_EQ(packet_in.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet_in.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet_in.bitrate, packet2.bitrate);
    EXPECT_EQ(packet_in.rotation, packet2.rotation);
    EXPECT_EQ(packet_in.hfov, packet2.hfov);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_FOV_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_FOV_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat_camera = 963497672;
    packet_in.lon_camera = 963497880;
    packet_in.alt_camera = 963498088;
    packet_in.lat_image = 963498296;
    packet_in.lon_image = 963498504;
    packet_in.alt_image = 963498712;
    packet_in.q = {{ 213.0, 214.0, 215.0, 216.0 }};
    packet_in.hfov = 325.0;
    packet_in.vfov = 353.0;
    packet_in.camera_device_id = 161;

    mavlink::common::msg::CAMERA_FOV_STATUS packet1{};
    mavlink::common::msg::CAMERA_FOV_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.lat_camera, packet2.lat_camera);
    EXPECT_EQ(packet1.lon_camera, packet2.lon_camera);
    EXPECT_EQ(packet1.alt_camera, packet2.alt_camera);
    EXPECT_EQ(packet1.lat_image, packet2.lat_image);
    EXPECT_EQ(packet1.lon_image, packet2.lon_image);
    EXPECT_EQ(packet1.alt_image, packet2.alt_image);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.hfov, packet2.hfov);
    EXPECT_EQ(packet1.vfov, packet2.vfov);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_FOV_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_fov_status_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296, 963498504, 963498712, { 213.0, 214.0, 215.0, 216.0 }, 325.0, 353.0, 161
    };

    mavlink::common::msg::CAMERA_FOV_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat_camera = 963497672;
    packet_in.lon_camera = 963497880;
    packet_in.alt_camera = 963498088;
    packet_in.lat_image = 963498296;
    packet_in.lon_image = 963498504;
    packet_in.alt_image = 963498712;
    packet_in.q = {{ 213.0, 214.0, 215.0, 216.0 }};
    packet_in.hfov = 325.0;
    packet_in.vfov = 353.0;
    packet_in.camera_device_id = 161;

    mavlink::common::msg::CAMERA_FOV_STATUS packet2{};

    mavlink_msg_camera_fov_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.lat_camera, packet2.lat_camera);
    EXPECT_EQ(packet_in.lon_camera, packet2.lon_camera);
    EXPECT_EQ(packet_in.alt_camera, packet2.alt_camera);
    EXPECT_EQ(packet_in.lat_image, packet2.lat_image);
    EXPECT_EQ(packet_in.lon_image, packet2.lon_image);
    EXPECT_EQ(packet_in.alt_image, packet2.alt_image);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.hfov, packet2.hfov);
    EXPECT_EQ(packet_in.vfov, packet2.vfov);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_TRACKING_IMAGE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_TRACKING_IMAGE_STATUS packet_in{};
    packet_in.tracking_status = 89;
    packet_in.tracking_mode = 156;
    packet_in.target_data = 223;
    packet_in.point_x = 17.0;
    packet_in.point_y = 45.0;
    packet_in.radius = 73.0;
    packet_in.rec_top_x = 101.0;
    packet_in.rec_top_y = 129.0;
    packet_in.rec_bottom_x = 157.0;
    packet_in.rec_bottom_y = 185.0;
    packet_in.camera_device_id = 34;

    mavlink::common::msg::CAMERA_TRACKING_IMAGE_STATUS packet1{};
    mavlink::common::msg::CAMERA_TRACKING_IMAGE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.tracking_status, packet2.tracking_status);
    EXPECT_EQ(packet1.tracking_mode, packet2.tracking_mode);
    EXPECT_EQ(packet1.target_data, packet2.target_data);
    EXPECT_EQ(packet1.point_x, packet2.point_x);
    EXPECT_EQ(packet1.point_y, packet2.point_y);
    EXPECT_EQ(packet1.radius, packet2.radius);
    EXPECT_EQ(packet1.rec_top_x, packet2.rec_top_x);
    EXPECT_EQ(packet1.rec_top_y, packet2.rec_top_y);
    EXPECT_EQ(packet1.rec_bottom_x, packet2.rec_bottom_x);
    EXPECT_EQ(packet1.rec_bottom_y, packet2.rec_bottom_y);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_TRACKING_IMAGE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_tracking_image_status_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 89, 156, 223, 34
    };

    mavlink::common::msg::CAMERA_TRACKING_IMAGE_STATUS packet_in{};
    packet_in.tracking_status = 89;
    packet_in.tracking_mode = 156;
    packet_in.target_data = 223;
    packet_in.point_x = 17.0;
    packet_in.point_y = 45.0;
    packet_in.radius = 73.0;
    packet_in.rec_top_x = 101.0;
    packet_in.rec_top_y = 129.0;
    packet_in.rec_bottom_x = 157.0;
    packet_in.rec_bottom_y = 185.0;
    packet_in.camera_device_id = 34;

    mavlink::common::msg::CAMERA_TRACKING_IMAGE_STATUS packet2{};

    mavlink_msg_camera_tracking_image_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.tracking_status, packet2.tracking_status);
    EXPECT_EQ(packet_in.tracking_mode, packet2.tracking_mode);
    EXPECT_EQ(packet_in.target_data, packet2.target_data);
    EXPECT_EQ(packet_in.point_x, packet2.point_x);
    EXPECT_EQ(packet_in.point_y, packet2.point_y);
    EXPECT_EQ(packet_in.radius, packet2.radius);
    EXPECT_EQ(packet_in.rec_top_x, packet2.rec_top_x);
    EXPECT_EQ(packet_in.rec_top_y, packet2.rec_top_y);
    EXPECT_EQ(packet_in.rec_bottom_x, packet2.rec_bottom_x);
    EXPECT_EQ(packet_in.rec_bottom_y, packet2.rec_bottom_y);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_TRACKING_GEO_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_TRACKING_GEO_STATUS packet_in{};
    packet_in.tracking_status = 149;
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;
    packet_in.alt = 73.0;
    packet_in.h_acc = 101.0;
    packet_in.v_acc = 129.0;
    packet_in.vel_n = 157.0;
    packet_in.vel_e = 185.0;
    packet_in.vel_d = 213.0;
    packet_in.vel_acc = 241.0;
    packet_in.dist = 269.0;
    packet_in.hdg = 297.0;
    packet_in.hdg_acc = 325.0;
    packet_in.camera_device_id = 216;

    mavlink::common::msg::CAMERA_TRACKING_GEO_STATUS packet1{};
    mavlink::common::msg::CAMERA_TRACKING_GEO_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.tracking_status, packet2.tracking_status);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.h_acc, packet2.h_acc);
    EXPECT_EQ(packet1.v_acc, packet2.v_acc);
    EXPECT_EQ(packet1.vel_n, packet2.vel_n);
    EXPECT_EQ(packet1.vel_e, packet2.vel_e);
    EXPECT_EQ(packet1.vel_d, packet2.vel_d);
    EXPECT_EQ(packet1.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet1.dist, packet2.dist);
    EXPECT_EQ(packet1.hdg, packet2.hdg);
    EXPECT_EQ(packet1.hdg_acc, packet2.hdg_acc);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_TRACKING_GEO_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_tracking_geo_status_t packet_c {
         963497464, 963497672, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 149, 216
    };

    mavlink::common::msg::CAMERA_TRACKING_GEO_STATUS packet_in{};
    packet_in.tracking_status = 149;
    packet_in.lat = 963497464;
    packet_in.lon = 963497672;
    packet_in.alt = 73.0;
    packet_in.h_acc = 101.0;
    packet_in.v_acc = 129.0;
    packet_in.vel_n = 157.0;
    packet_in.vel_e = 185.0;
    packet_in.vel_d = 213.0;
    packet_in.vel_acc = 241.0;
    packet_in.dist = 269.0;
    packet_in.hdg = 297.0;
    packet_in.hdg_acc = 325.0;
    packet_in.camera_device_id = 216;

    mavlink::common::msg::CAMERA_TRACKING_GEO_STATUS packet2{};

    mavlink_msg_camera_tracking_geo_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.tracking_status, packet2.tracking_status);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.h_acc, packet2.h_acc);
    EXPECT_EQ(packet_in.v_acc, packet2.v_acc);
    EXPECT_EQ(packet_in.vel_n, packet2.vel_n);
    EXPECT_EQ(packet_in.vel_e, packet2.vel_e);
    EXPECT_EQ(packet_in.vel_d, packet2.vel_d);
    EXPECT_EQ(packet_in.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet_in.dist, packet2.dist);
    EXPECT_EQ(packet_in.hdg, packet2.hdg);
    EXPECT_EQ(packet_in.hdg_acc, packet2.hdg_acc);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_THERMAL_RANGE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_THERMAL_RANGE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.stream_id = 89;
    packet_in.camera_device_id = 156;
    packet_in.max = 45.0;
    packet_in.max_point_x = 73.0;
    packet_in.max_point_y = 101.0;
    packet_in.min = 129.0;
    packet_in.min_point_x = 157.0;
    packet_in.min_point_y = 185.0;

    mavlink::common::msg::CAMERA_THERMAL_RANGE packet1{};
    mavlink::common::msg::CAMERA_THERMAL_RANGE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.stream_id, packet2.stream_id);
    EXPECT_EQ(packet1.camera_device_id, packet2.camera_device_id);
    EXPECT_EQ(packet1.max, packet2.max);
    EXPECT_EQ(packet1.max_point_x, packet2.max_point_x);
    EXPECT_EQ(packet1.max_point_y, packet2.max_point_y);
    EXPECT_EQ(packet1.min, packet2.min);
    EXPECT_EQ(packet1.min_point_x, packet2.min_point_x);
    EXPECT_EQ(packet1.min_point_y, packet2.min_point_y);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_THERMAL_RANGE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_thermal_range_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 89, 156
    };

    mavlink::common::msg::CAMERA_THERMAL_RANGE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.stream_id = 89;
    packet_in.camera_device_id = 156;
    packet_in.max = 45.0;
    packet_in.max_point_x = 73.0;
    packet_in.max_point_y = 101.0;
    packet_in.min = 129.0;
    packet_in.min_point_x = 157.0;
    packet_in.min_point_y = 185.0;

    mavlink::common::msg::CAMERA_THERMAL_RANGE packet2{};

    mavlink_msg_camera_thermal_range_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.stream_id, packet2.stream_id);
    EXPECT_EQ(packet_in.camera_device_id, packet2.camera_device_id);
    EXPECT_EQ(packet_in.max, packet2.max);
    EXPECT_EQ(packet_in.max_point_x, packet2.max_point_x);
    EXPECT_EQ(packet_in.max_point_y, packet2.max_point_y);
    EXPECT_EQ(packet_in.min, packet2.min);
    EXPECT_EQ(packet_in.min_point_x, packet2.min_point_x);
    EXPECT_EQ(packet_in.min_point_y, packet2.min_point_y);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_MANAGER_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_MANAGER_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.cap_flags = 963497672;
    packet_in.gimbal_device_id = 101;
    packet_in.roll_min = 73.0;
    packet_in.roll_max = 101.0;
    packet_in.pitch_min = 129.0;
    packet_in.pitch_max = 157.0;
    packet_in.yaw_min = 185.0;
    packet_in.yaw_max = 213.0;

    mavlink::common::msg::GIMBAL_MANAGER_INFORMATION packet1{};
    mavlink::common::msg::GIMBAL_MANAGER_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.cap_flags, packet2.cap_flags);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet1.roll_min, packet2.roll_min);
    EXPECT_EQ(packet1.roll_max, packet2.roll_max);
    EXPECT_EQ(packet1.pitch_min, packet2.pitch_min);
    EXPECT_EQ(packet1.pitch_max, packet2.pitch_max);
    EXPECT_EQ(packet1.yaw_min, packet2.yaw_min);
    EXPECT_EQ(packet1.yaw_max, packet2.yaw_max);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_MANAGER_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_manager_information_t packet_c {
         963497464, 963497672, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 101
    };

    mavlink::common::msg::GIMBAL_MANAGER_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.cap_flags = 963497672;
    packet_in.gimbal_device_id = 101;
    packet_in.roll_min = 73.0;
    packet_in.roll_max = 101.0;
    packet_in.pitch_min = 129.0;
    packet_in.pitch_max = 157.0;
    packet_in.yaw_min = 185.0;
    packet_in.yaw_max = 213.0;

    mavlink::common::msg::GIMBAL_MANAGER_INFORMATION packet2{};

    mavlink_msg_gimbal_manager_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.cap_flags, packet2.cap_flags);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet_in.roll_min, packet2.roll_min);
    EXPECT_EQ(packet_in.roll_max, packet2.roll_max);
    EXPECT_EQ(packet_in.pitch_min, packet2.pitch_min);
    EXPECT_EQ(packet_in.pitch_max, packet2.pitch_max);
    EXPECT_EQ(packet_in.yaw_min, packet2.yaw_min);
    EXPECT_EQ(packet_in.yaw_max, packet2.yaw_max);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_MANAGER_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_MANAGER_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.flags = 963497672;
    packet_in.gimbal_device_id = 29;
    packet_in.primary_control_sysid = 96;
    packet_in.primary_control_compid = 163;
    packet_in.secondary_control_sysid = 230;
    packet_in.secondary_control_compid = 41;

    mavlink::common::msg::GIMBAL_MANAGER_STATUS packet1{};
    mavlink::common::msg::GIMBAL_MANAGER_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet1.primary_control_sysid, packet2.primary_control_sysid);
    EXPECT_EQ(packet1.primary_control_compid, packet2.primary_control_compid);
    EXPECT_EQ(packet1.secondary_control_sysid, packet2.secondary_control_sysid);
    EXPECT_EQ(packet1.secondary_control_compid, packet2.secondary_control_compid);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_MANAGER_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_manager_status_t packet_c {
         963497464, 963497672, 29, 96, 163, 230, 41
    };

    mavlink::common::msg::GIMBAL_MANAGER_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.flags = 963497672;
    packet_in.gimbal_device_id = 29;
    packet_in.primary_control_sysid = 96;
    packet_in.primary_control_compid = 163;
    packet_in.secondary_control_sysid = 230;
    packet_in.secondary_control_compid = 41;

    mavlink::common::msg::GIMBAL_MANAGER_STATUS packet2{};

    mavlink_msg_gimbal_manager_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet_in.primary_control_sysid, packet2.primary_control_sysid);
    EXPECT_EQ(packet_in.primary_control_compid, packet2.primary_control_compid);
    EXPECT_EQ(packet_in.secondary_control_sysid, packet2.secondary_control_sysid);
    EXPECT_EQ(packet_in.secondary_control_compid, packet2.secondary_control_compid);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_MANAGER_SET_ATTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.flags = 963497464;
    packet_in.gimbal_device_id = 235;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.angular_velocity_x = 157.0;
    packet_in.angular_velocity_y = 185.0;
    packet_in.angular_velocity_z = 213.0;

    mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE packet1{};
    mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.angular_velocity_x, packet2.angular_velocity_x);
    EXPECT_EQ(packet1.angular_velocity_y, packet2.angular_velocity_y);
    EXPECT_EQ(packet1.angular_velocity_z, packet2.angular_velocity_z);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_MANAGER_SET_ATTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_manager_set_attitude_t packet_c {
         963497464, { 45.0, 46.0, 47.0, 48.0 }, 157.0, 185.0, 213.0, 101, 168, 235
    };

    mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.flags = 963497464;
    packet_in.gimbal_device_id = 235;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.angular_velocity_x = 157.0;
    packet_in.angular_velocity_y = 185.0;
    packet_in.angular_velocity_z = 213.0;

    mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE packet2{};

    mavlink_msg_gimbal_manager_set_attitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.angular_velocity_x, packet2.angular_velocity_x);
    EXPECT_EQ(packet_in.angular_velocity_y, packet2.angular_velocity_y);
    EXPECT_EQ(packet_in.angular_velocity_z, packet2.angular_velocity_z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_DEVICE_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_DEVICE_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.vendor_name = to_char_array("WXYZABCDEFGHIJKLMNOPQRSTUVWXYZA");
    packet_in.model_name = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.custom_name = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLM");
    packet_in.firmware_version = 963498088;
    packet_in.hardware_version = 963498296;
    packet_in.uid = 93372036854775807ULL;
    packet_in.cap_flags = 19523;
    packet_in.custom_cap_flags = 19627;
    packet_in.roll_min = 157.0;
    packet_in.roll_max = 185.0;
    packet_in.pitch_min = 213.0;
    packet_in.pitch_max = 241.0;
    packet_in.yaw_min = 269.0;
    packet_in.yaw_max = 297.0;
    packet_in.gimbal_device_id = 181;

    mavlink::common::msg::GIMBAL_DEVICE_INFORMATION packet1{};
    mavlink::common::msg::GIMBAL_DEVICE_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet1.model_name, packet2.model_name);
    EXPECT_EQ(packet1.custom_name, packet2.custom_name);
    EXPECT_EQ(packet1.firmware_version, packet2.firmware_version);
    EXPECT_EQ(packet1.hardware_version, packet2.hardware_version);
    EXPECT_EQ(packet1.uid, packet2.uid);
    EXPECT_EQ(packet1.cap_flags, packet2.cap_flags);
    EXPECT_EQ(packet1.custom_cap_flags, packet2.custom_cap_flags);
    EXPECT_EQ(packet1.roll_min, packet2.roll_min);
    EXPECT_EQ(packet1.roll_max, packet2.roll_max);
    EXPECT_EQ(packet1.pitch_min, packet2.pitch_min);
    EXPECT_EQ(packet1.pitch_max, packet2.pitch_max);
    EXPECT_EQ(packet1.yaw_min, packet2.yaw_min);
    EXPECT_EQ(packet1.yaw_max, packet2.yaw_max);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_DEVICE_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_device_information_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 19523, 19627, "WXYZABCDEFGHIJKLMNOPQRSTUVWXYZA", "CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG", "IJKLMNOPQRSTUVWXYZABCDEFGHIJKLM", 181
    };

    mavlink::common::msg::GIMBAL_DEVICE_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.vendor_name = to_char_array("WXYZABCDEFGHIJKLMNOPQRSTUVWXYZA");
    packet_in.model_name = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.custom_name = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLM");
    packet_in.firmware_version = 963498088;
    packet_in.hardware_version = 963498296;
    packet_in.uid = 93372036854775807ULL;
    packet_in.cap_flags = 19523;
    packet_in.custom_cap_flags = 19627;
    packet_in.roll_min = 157.0;
    packet_in.roll_max = 185.0;
    packet_in.pitch_min = 213.0;
    packet_in.pitch_max = 241.0;
    packet_in.yaw_min = 269.0;
    packet_in.yaw_max = 297.0;
    packet_in.gimbal_device_id = 181;

    mavlink::common::msg::GIMBAL_DEVICE_INFORMATION packet2{};

    mavlink_msg_gimbal_device_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet_in.model_name, packet2.model_name);
    EXPECT_EQ(packet_in.custom_name, packet2.custom_name);
    EXPECT_EQ(packet_in.firmware_version, packet2.firmware_version);
    EXPECT_EQ(packet_in.hardware_version, packet2.hardware_version);
    EXPECT_EQ(packet_in.uid, packet2.uid);
    EXPECT_EQ(packet_in.cap_flags, packet2.cap_flags);
    EXPECT_EQ(packet_in.custom_cap_flags, packet2.custom_cap_flags);
    EXPECT_EQ(packet_in.roll_min, packet2.roll_min);
    EXPECT_EQ(packet_in.roll_max, packet2.roll_max);
    EXPECT_EQ(packet_in.pitch_min, packet2.pitch_min);
    EXPECT_EQ(packet_in.pitch_max, packet2.pitch_max);
    EXPECT_EQ(packet_in.yaw_min, packet2.yaw_min);
    EXPECT_EQ(packet_in.yaw_max, packet2.yaw_max);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_DEVICE_SET_ATTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.flags = 18691;
    packet_in.q = {{ 17.0, 18.0, 19.0, 20.0 }};
    packet_in.angular_velocity_x = 129.0;
    packet_in.angular_velocity_y = 157.0;
    packet_in.angular_velocity_z = 185.0;

    mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE packet1{};
    mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.angular_velocity_x, packet2.angular_velocity_x);
    EXPECT_EQ(packet1.angular_velocity_y, packet2.angular_velocity_y);
    EXPECT_EQ(packet1.angular_velocity_z, packet2.angular_velocity_z);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_DEVICE_SET_ATTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_device_set_attitude_t packet_c {
         { 17.0, 18.0, 19.0, 20.0 }, 129.0, 157.0, 185.0, 18691, 223, 34
    };

    mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.flags = 18691;
    packet_in.q = {{ 17.0, 18.0, 19.0, 20.0 }};
    packet_in.angular_velocity_x = 129.0;
    packet_in.angular_velocity_y = 157.0;
    packet_in.angular_velocity_z = 185.0;

    mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE packet2{};

    mavlink_msg_gimbal_device_set_attitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.angular_velocity_x, packet2.angular_velocity_x);
    EXPECT_EQ(packet_in.angular_velocity_y, packet2.angular_velocity_y);
    EXPECT_EQ(packet_in.angular_velocity_z, packet2.angular_velocity_z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_DEVICE_ATTITUDE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS packet_in{};
    packet_in.target_system = 247;
    packet_in.target_component = 58;
    packet_in.time_boot_ms = 963497464;
    packet_in.flags = 19107;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.angular_velocity_x = 157.0;
    packet_in.angular_velocity_y = 185.0;
    packet_in.angular_velocity_z = 213.0;
    packet_in.failure_flags = 963499128;
    packet_in.delta_yaw = 297.0;
    packet_in.delta_yaw_velocity = 325.0;
    packet_in.gimbal_device_id = 149;

    mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS packet1{};
    mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.angular_velocity_x, packet2.angular_velocity_x);
    EXPECT_EQ(packet1.angular_velocity_y, packet2.angular_velocity_y);
    EXPECT_EQ(packet1.angular_velocity_z, packet2.angular_velocity_z);
    EXPECT_EQ(packet1.failure_flags, packet2.failure_flags);
    EXPECT_EQ(packet1.delta_yaw, packet2.delta_yaw);
    EXPECT_EQ(packet1.delta_yaw_velocity, packet2.delta_yaw_velocity);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_DEVICE_ATTITUDE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_device_attitude_status_t packet_c {
         963497464, { 45.0, 46.0, 47.0, 48.0 }, 157.0, 185.0, 213.0, 963499128, 19107, 247, 58, 297.0, 325.0, 149
    };

    mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS packet_in{};
    packet_in.target_system = 247;
    packet_in.target_component = 58;
    packet_in.time_boot_ms = 963497464;
    packet_in.flags = 19107;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.angular_velocity_x = 157.0;
    packet_in.angular_velocity_y = 185.0;
    packet_in.angular_velocity_z = 213.0;
    packet_in.failure_flags = 963499128;
    packet_in.delta_yaw = 297.0;
    packet_in.delta_yaw_velocity = 325.0;
    packet_in.gimbal_device_id = 149;

    mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS packet2{};

    mavlink_msg_gimbal_device_attitude_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.angular_velocity_x, packet2.angular_velocity_x);
    EXPECT_EQ(packet_in.angular_velocity_y, packet2.angular_velocity_y);
    EXPECT_EQ(packet_in.angular_velocity_z, packet2.angular_velocity_z);
    EXPECT_EQ(packet_in.failure_flags, packet2.failure_flags);
    EXPECT_EQ(packet_in.delta_yaw, packet2.delta_yaw);
    EXPECT_EQ(packet_in.delta_yaw_velocity, packet2.delta_yaw_velocity);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::AUTOPILOT_STATE_FOR_GIMBAL_DEVICE packet_in{};
    packet_in.target_system = 27;
    packet_in.target_component = 94;
    packet_in.time_boot_us = 93372036854775807ULL;
    packet_in.q = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.q_estimated_delay_us = 963498712;
    packet_in.vx = 213.0;
    packet_in.vy = 241.0;
    packet_in.vz = 269.0;
    packet_in.v_estimated_delay_us = 963499544;
    packet_in.feed_forward_angular_velocity_z = 325.0;
    packet_in.estimator_status = 19731;
    packet_in.landed_state = 161;
    packet_in.angular_velocity_z = 388.0;

    mavlink::common::msg::AUTOPILOT_STATE_FOR_GIMBAL_DEVICE packet1{};
    mavlink::common::msg::AUTOPILOT_STATE_FOR_GIMBAL_DEVICE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.time_boot_us, packet2.time_boot_us);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.q_estimated_delay_us, packet2.q_estimated_delay_us);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.v_estimated_delay_us, packet2.v_estimated_delay_us);
    EXPECT_EQ(packet1.feed_forward_angular_velocity_z, packet2.feed_forward_angular_velocity_z);
    EXPECT_EQ(packet1.estimator_status, packet2.estimator_status);
    EXPECT_EQ(packet1.landed_state, packet2.landed_state);
    EXPECT_EQ(packet1.angular_velocity_z, packet2.angular_velocity_z);
}

#ifdef TEST_INTEROP
TEST(common_interop, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_autopilot_state_for_gimbal_device_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0 }, 963498712, 213.0, 241.0, 269.0, 963499544, 325.0, 19731, 27, 94, 161, 388.0
    };

    mavlink::common::msg::AUTOPILOT_STATE_FOR_GIMBAL_DEVICE packet_in{};
    packet_in.target_system = 27;
    packet_in.target_component = 94;
    packet_in.time_boot_us = 93372036854775807ULL;
    packet_in.q = {{ 73.0, 74.0, 75.0, 76.0 }};
    packet_in.q_estimated_delay_us = 963498712;
    packet_in.vx = 213.0;
    packet_in.vy = 241.0;
    packet_in.vz = 269.0;
    packet_in.v_estimated_delay_us = 963499544;
    packet_in.feed_forward_angular_velocity_z = 325.0;
    packet_in.estimator_status = 19731;
    packet_in.landed_state = 161;
    packet_in.angular_velocity_z = 388.0;

    mavlink::common::msg::AUTOPILOT_STATE_FOR_GIMBAL_DEVICE packet2{};

    mavlink_msg_autopilot_state_for_gimbal_device_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.time_boot_us, packet2.time_boot_us);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.q_estimated_delay_us, packet2.q_estimated_delay_us);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.v_estimated_delay_us, packet2.v_estimated_delay_us);
    EXPECT_EQ(packet_in.feed_forward_angular_velocity_z, packet2.feed_forward_angular_velocity_z);
    EXPECT_EQ(packet_in.estimator_status, packet2.estimator_status);
    EXPECT_EQ(packet_in.landed_state, packet2.landed_state);
    EXPECT_EQ(packet_in.angular_velocity_z, packet2.angular_velocity_z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_MANAGER_SET_PITCHYAW)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW packet_in{};
    packet_in.target_system = 65;
    packet_in.target_component = 132;
    packet_in.flags = 963497464;
    packet_in.gimbal_device_id = 199;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;
    packet_in.pitch_rate = 101.0;
    packet_in.yaw_rate = 129.0;

    mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW packet1{};
    mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.pitch_rate, packet2.pitch_rate);
    EXPECT_EQ(packet1.yaw_rate, packet2.yaw_rate);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_MANAGER_SET_PITCHYAW)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_manager_set_pitchyaw_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 65, 132, 199
    };

    mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW packet_in{};
    packet_in.target_system = 65;
    packet_in.target_component = 132;
    packet_in.flags = 963497464;
    packet_in.gimbal_device_id = 199;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;
    packet_in.pitch_rate = 101.0;
    packet_in.yaw_rate = 129.0;

    mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW packet2{};

    mavlink_msg_gimbal_manager_set_pitchyaw_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.pitch_rate, packet2.pitch_rate);
    EXPECT_EQ(packet_in.yaw_rate, packet2.yaw_rate);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GIMBAL_MANAGER_SET_MANUAL_CONTROL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GIMBAL_MANAGER_SET_MANUAL_CONTROL packet_in{};
    packet_in.target_system = 65;
    packet_in.target_component = 132;
    packet_in.flags = 963497464;
    packet_in.gimbal_device_id = 199;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;
    packet_in.pitch_rate = 101.0;
    packet_in.yaw_rate = 129.0;

    mavlink::common::msg::GIMBAL_MANAGER_SET_MANUAL_CONTROL packet1{};
    mavlink::common::msg::GIMBAL_MANAGER_SET_MANUAL_CONTROL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.pitch_rate, packet2.pitch_rate);
    EXPECT_EQ(packet1.yaw_rate, packet2.yaw_rate);
}

#ifdef TEST_INTEROP
TEST(common_interop, GIMBAL_MANAGER_SET_MANUAL_CONTROL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gimbal_manager_set_manual_control_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 65, 132, 199
    };

    mavlink::common::msg::GIMBAL_MANAGER_SET_MANUAL_CONTROL packet_in{};
    packet_in.target_system = 65;
    packet_in.target_component = 132;
    packet_in.flags = 963497464;
    packet_in.gimbal_device_id = 199;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;
    packet_in.pitch_rate = 101.0;
    packet_in.yaw_rate = 129.0;

    mavlink::common::msg::GIMBAL_MANAGER_SET_MANUAL_CONTROL packet2{};

    mavlink_msg_gimbal_manager_set_manual_control_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.gimbal_device_id, packet2.gimbal_device_id);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.pitch_rate, packet2.pitch_rate);
    EXPECT_EQ(packet_in.yaw_rate, packet2.yaw_rate);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ESC_INFO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ESC_INFO packet_in{};
    packet_in.index = 3;
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.counter = 18483;
    packet_in.count = 70;
    packet_in.connection_type = 137;
    packet_in.info = 204;
    packet_in.failure_flags = {{ 18587, 18588, 18589, 18590 }};
    packet_in.error_count = {{ 963497880, 963497881, 963497882, 963497883 }};
    packet_in.temperature = {{ 19003, 19004, 19005, 19006 }};

    mavlink::common::msg::ESC_INFO packet1{};
    mavlink::common::msg::ESC_INFO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.index, packet2.index);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.counter, packet2.counter);
    EXPECT_EQ(packet1.count, packet2.count);
    EXPECT_EQ(packet1.connection_type, packet2.connection_type);
    EXPECT_EQ(packet1.info, packet2.info);
    EXPECT_EQ(packet1.failure_flags, packet2.failure_flags);
    EXPECT_EQ(packet1.error_count, packet2.error_count);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
}

#ifdef TEST_INTEROP
TEST(common_interop, ESC_INFO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_esc_info_t packet_c {
         93372036854775807ULL, { 963497880, 963497881, 963497882, 963497883 }, 18483, { 18587, 18588, 18589, 18590 }, { 19003, 19004, 19005, 19006 }, 3, 70, 137, 204
    };

    mavlink::common::msg::ESC_INFO packet_in{};
    packet_in.index = 3;
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.counter = 18483;
    packet_in.count = 70;
    packet_in.connection_type = 137;
    packet_in.info = 204;
    packet_in.failure_flags = {{ 18587, 18588, 18589, 18590 }};
    packet_in.error_count = {{ 963497880, 963497881, 963497882, 963497883 }};
    packet_in.temperature = {{ 19003, 19004, 19005, 19006 }};

    mavlink::common::msg::ESC_INFO packet2{};

    mavlink_msg_esc_info_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.index, packet2.index);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.counter, packet2.counter);
    EXPECT_EQ(packet_in.count, packet2.count);
    EXPECT_EQ(packet_in.connection_type, packet2.connection_type);
    EXPECT_EQ(packet_in.info, packet2.info);
    EXPECT_EQ(packet_in.failure_flags, packet2.failure_flags);
    EXPECT_EQ(packet_in.error_count, packet2.error_count);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ESC_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ESC_STATUS packet_in{};
    packet_in.index = 173;
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.rpm = {{ 963497880, 963497881, 963497882, 963497883 }};
    packet_in.voltage = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.current = {{ 297.0, 298.0, 299.0, 300.0 }};

    mavlink::common::msg::ESC_STATUS packet1{};
    mavlink::common::msg::ESC_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.index, packet2.index);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.rpm, packet2.rpm);
    EXPECT_EQ(packet1.voltage, packet2.voltage);
    EXPECT_EQ(packet1.current, packet2.current);
}

#ifdef TEST_INTEROP
TEST(common_interop, ESC_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_esc_status_t packet_c {
         93372036854775807ULL, { 963497880, 963497881, 963497882, 963497883 }, { 185.0, 186.0, 187.0, 188.0 }, { 297.0, 298.0, 299.0, 300.0 }, 173
    };

    mavlink::common::msg::ESC_STATUS packet_in{};
    packet_in.index = 173;
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.rpm = {{ 963497880, 963497881, 963497882, 963497883 }};
    packet_in.voltage = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.current = {{ 297.0, 298.0, 299.0, 300.0 }};

    mavlink::common::msg::ESC_STATUS packet2{};

    mavlink_msg_esc_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.index, packet2.index);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.rpm, packet2.rpm);
    EXPECT_EQ(packet_in.voltage, packet2.voltage);
    EXPECT_EQ(packet_in.current, packet2.current);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, WIFI_CONFIG_AP)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::WIFI_CONFIG_AP packet_in{};
    packet_in.ssid = to_char_array("ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");
    packet_in.password = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.mode = 37;
    packet_in.response = 104;

    mavlink::common::msg::WIFI_CONFIG_AP packet1{};
    mavlink::common::msg::WIFI_CONFIG_AP packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.ssid, packet2.ssid);
    EXPECT_EQ(packet1.password, packet2.password);
    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.response, packet2.response);
}

#ifdef TEST_INTEROP
TEST(common_interop, WIFI_CONFIG_AP)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_wifi_config_ap_t packet_c {
         "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE", "GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ", 37, 104
    };

    mavlink::common::msg::WIFI_CONFIG_AP packet_in{};
    packet_in.ssid = to_char_array("ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");
    packet_in.password = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.mode = 37;
    packet_in.response = 104;

    mavlink::common::msg::WIFI_CONFIG_AP packet2{};

    mavlink_msg_wifi_config_ap_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.ssid, packet2.ssid);
    EXPECT_EQ(packet_in.password, packet2.password);
    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.response, packet2.response);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, AIS_VESSEL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::AIS_VESSEL packet_in{};
    packet_in.MMSI = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.COG = 17859;
    packet_in.heading = 17963;
    packet_in.velocity = 18067;
    packet_in.turn_rate = -45;
    packet_in.navigational_status = 22;
    packet_in.type = 89;
    packet_in.dimension_bow = 18171;
    packet_in.dimension_stern = 18275;
    packet_in.dimension_port = 156;
    packet_in.dimension_starboard = 223;
    packet_in.callsign = to_char_array("FGHIJK");
    packet_in.name = to_char_array("MNOPQRSTUVWXYZABCDE");
    packet_in.tslc = 18379;
    packet_in.flags = 18483;

    mavlink::common::msg::AIS_VESSEL packet1{};
    mavlink::common::msg::AIS_VESSEL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.MMSI, packet2.MMSI);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.COG, packet2.COG);
    EXPECT_EQ(packet1.heading, packet2.heading);
    EXPECT_EQ(packet1.velocity, packet2.velocity);
    EXPECT_EQ(packet1.turn_rate, packet2.turn_rate);
    EXPECT_EQ(packet1.navigational_status, packet2.navigational_status);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.dimension_bow, packet2.dimension_bow);
    EXPECT_EQ(packet1.dimension_stern, packet2.dimension_stern);
    EXPECT_EQ(packet1.dimension_port, packet2.dimension_port);
    EXPECT_EQ(packet1.dimension_starboard, packet2.dimension_starboard);
    EXPECT_EQ(packet1.callsign, packet2.callsign);
    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.tslc, packet2.tslc);
    EXPECT_EQ(packet1.flags, packet2.flags);
}

#ifdef TEST_INTEROP
TEST(common_interop, AIS_VESSEL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_ais_vessel_t packet_c {
         963497464, 963497672, 963497880, 17859, 17963, 18067, 18171, 18275, 18379, 18483, -45, 22, 89, 156, 223, "FGHIJK", "MNOPQRSTUVWXYZABCDE"
    };

    mavlink::common::msg::AIS_VESSEL packet_in{};
    packet_in.MMSI = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.COG = 17859;
    packet_in.heading = 17963;
    packet_in.velocity = 18067;
    packet_in.turn_rate = -45;
    packet_in.navigational_status = 22;
    packet_in.type = 89;
    packet_in.dimension_bow = 18171;
    packet_in.dimension_stern = 18275;
    packet_in.dimension_port = 156;
    packet_in.dimension_starboard = 223;
    packet_in.callsign = to_char_array("FGHIJK");
    packet_in.name = to_char_array("MNOPQRSTUVWXYZABCDE");
    packet_in.tslc = 18379;
    packet_in.flags = 18483;

    mavlink::common::msg::AIS_VESSEL packet2{};

    mavlink_msg_ais_vessel_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.MMSI, packet2.MMSI);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.COG, packet2.COG);
    EXPECT_EQ(packet_in.heading, packet2.heading);
    EXPECT_EQ(packet_in.velocity, packet2.velocity);
    EXPECT_EQ(packet_in.turn_rate, packet2.turn_rate);
    EXPECT_EQ(packet_in.navigational_status, packet2.navigational_status);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.dimension_bow, packet2.dimension_bow);
    EXPECT_EQ(packet_in.dimension_stern, packet2.dimension_stern);
    EXPECT_EQ(packet_in.dimension_port, packet2.dimension_port);
    EXPECT_EQ(packet_in.dimension_starboard, packet2.dimension_starboard);
    EXPECT_EQ(packet_in.callsign, packet2.callsign);
    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.tslc, packet2.tslc);
    EXPECT_EQ(packet_in.flags, packet2.flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, UAVCAN_NODE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::UAVCAN_NODE_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.uptime_sec = 963497880;
    packet_in.health = 175;
    packet_in.mode = 242;
    packet_in.sub_mode = 53;
    packet_in.vendor_specific_status_code = 17859;

    mavlink::common::msg::UAVCAN_NODE_STATUS packet1{};
    mavlink::common::msg::UAVCAN_NODE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.uptime_sec, packet2.uptime_sec);
    EXPECT_EQ(packet1.health, packet2.health);
    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.sub_mode, packet2.sub_mode);
    EXPECT_EQ(packet1.vendor_specific_status_code, packet2.vendor_specific_status_code);
}

#ifdef TEST_INTEROP
TEST(common_interop, UAVCAN_NODE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_uavcan_node_status_t packet_c {
         93372036854775807ULL, 963497880, 17859, 175, 242, 53
    };

    mavlink::common::msg::UAVCAN_NODE_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.uptime_sec = 963497880;
    packet_in.health = 175;
    packet_in.mode = 242;
    packet_in.sub_mode = 53;
    packet_in.vendor_specific_status_code = 17859;

    mavlink::common::msg::UAVCAN_NODE_STATUS packet2{};

    mavlink_msg_uavcan_node_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.uptime_sec, packet2.uptime_sec);
    EXPECT_EQ(packet_in.health, packet2.health);
    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.sub_mode, packet2.sub_mode);
    EXPECT_EQ(packet_in.vendor_specific_status_code, packet2.vendor_specific_status_code);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, UAVCAN_NODE_INFO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::UAVCAN_NODE_INFO packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.uptime_sec = 963497880;
    packet_in.name = to_char_array("QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.hw_version_major = 37;
    packet_in.hw_version_minor = 104;
    packet_in.hw_unique_id = {{ 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186 }};
    packet_in.sw_version_major = 219;
    packet_in.sw_version_minor = 30;
    packet_in.sw_vcs_commit = 963498088;

    mavlink::common::msg::UAVCAN_NODE_INFO packet1{};
    mavlink::common::msg::UAVCAN_NODE_INFO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.uptime_sec, packet2.uptime_sec);
    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.hw_version_major, packet2.hw_version_major);
    EXPECT_EQ(packet1.hw_version_minor, packet2.hw_version_minor);
    EXPECT_EQ(packet1.hw_unique_id, packet2.hw_unique_id);
    EXPECT_EQ(packet1.sw_version_major, packet2.sw_version_major);
    EXPECT_EQ(packet1.sw_version_minor, packet2.sw_version_minor);
    EXPECT_EQ(packet1.sw_vcs_commit, packet2.sw_vcs_commit);
}

#ifdef TEST_INTEROP
TEST(common_interop, UAVCAN_NODE_INFO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_uavcan_node_info_t packet_c {
         93372036854775807ULL, 963497880, 963498088, "QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ", 37, 104, { 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186 }, 219, 30
    };

    mavlink::common::msg::UAVCAN_NODE_INFO packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.uptime_sec = 963497880;
    packet_in.name = to_char_array("QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.hw_version_major = 37;
    packet_in.hw_version_minor = 104;
    packet_in.hw_unique_id = {{ 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186 }};
    packet_in.sw_version_major = 219;
    packet_in.sw_version_minor = 30;
    packet_in.sw_vcs_commit = 963498088;

    mavlink::common::msg::UAVCAN_NODE_INFO packet2{};

    mavlink_msg_uavcan_node_info_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.uptime_sec, packet2.uptime_sec);
    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.hw_version_major, packet2.hw_version_major);
    EXPECT_EQ(packet_in.hw_version_minor, packet2.hw_version_minor);
    EXPECT_EQ(packet_in.hw_unique_id, packet2.hw_unique_id);
    EXPECT_EQ(packet_in.sw_version_major, packet2.sw_version_major);
    EXPECT_EQ(packet_in.sw_version_minor, packet2.sw_version_minor);
    EXPECT_EQ(packet_in.sw_vcs_commit, packet2.sw_vcs_commit);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_EXT_REQUEST_READ)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_EXT_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::common::msg::PARAM_EXT_REQUEST_READ packet1{};
    mavlink::common::msg::PARAM_EXT_REQUEST_READ packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_EXT_REQUEST_READ)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_ext_request_read_t packet_c {
         17235, 139, 206, "EFGHIJKLMNOPQRS"
    };

    mavlink::common::msg::PARAM_EXT_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::common::msg::PARAM_EXT_REQUEST_READ packet2{};

    mavlink_msg_param_ext_request_read_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_EXT_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_EXT_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::PARAM_EXT_REQUEST_LIST packet1{};
    mavlink::common::msg::PARAM_EXT_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_EXT_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_ext_request_list_t packet_c {
         5, 72
    };

    mavlink::common::msg::PARAM_EXT_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::PARAM_EXT_REQUEST_LIST packet2{};

    mavlink_msg_param_ext_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_EXT_VALUE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_EXT_VALUE packet_in{};
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_value = to_char_array("UVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.param_type = 193;
    packet_in.param_count = 17235;
    packet_in.param_index = 17339;

    mavlink::common::msg::PARAM_EXT_VALUE packet1{};
    mavlink::common::msg::PARAM_EXT_VALUE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
    EXPECT_EQ(packet1.param_count, packet2.param_count);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_EXT_VALUE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_ext_value_t packet_c {
         17235, 17339, "EFGHIJKLMNOPQRS", "UVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ", 193
    };

    mavlink::common::msg::PARAM_EXT_VALUE packet_in{};
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_value = to_char_array("UVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.param_type = 193;
    packet_in.param_count = 17235;
    packet_in.param_index = 17339;

    mavlink::common::msg::PARAM_EXT_VALUE packet2{};

    mavlink_msg_param_ext_value_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);
    EXPECT_EQ(packet_in.param_count, packet2.param_count);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_EXT_SET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_EXT_SET packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.param_id = to_char_array("CDEFGHIJKLMNOPQ");
    packet_in.param_value = to_char_array("STUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNO");
    packet_in.param_type = 59;

    mavlink::common::msg::PARAM_EXT_SET packet1{};
    mavlink::common::msg::PARAM_EXT_SET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_EXT_SET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_ext_set_t packet_c {
         5, 72, "CDEFGHIJKLMNOPQ", "STUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNO", 59
    };

    mavlink::common::msg::PARAM_EXT_SET packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.param_id = to_char_array("CDEFGHIJKLMNOPQ");
    packet_in.param_value = to_char_array("STUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNO");
    packet_in.param_type = 59;

    mavlink::common::msg::PARAM_EXT_SET packet2{};

    mavlink_msg_param_ext_set_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_EXT_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_EXT_ACK packet_in{};
    packet_in.param_id = to_char_array("ABCDEFGHIJKLMNO");
    packet_in.param_value = to_char_array("QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLM");
    packet_in.param_type = 181;
    packet_in.param_result = 248;

    mavlink::common::msg::PARAM_EXT_ACK packet1{};
    mavlink::common::msg::PARAM_EXT_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
    EXPECT_EQ(packet1.param_result, packet2.param_result);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_EXT_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_ext_ack_t packet_c {
         "ABCDEFGHIJKLMNO", "QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLM", 181, 248
    };

    mavlink::common::msg::PARAM_EXT_ACK packet_in{};
    packet_in.param_id = to_char_array("ABCDEFGHIJKLMNO");
    packet_in.param_value = to_char_array("QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLM");
    packet_in.param_type = 181;
    packet_in.param_result = 248;

    mavlink::common::msg::PARAM_EXT_ACK packet2{};

    mavlink_msg_param_ext_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);
    EXPECT_EQ(packet_in.param_result, packet2.param_result);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OBSTACLE_DISTANCE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OBSTACLE_DISTANCE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_type = 217;
    packet_in.distances = {{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714, 17715, 17716, 17717, 17718, 17719, 17720, 17721, 17722 }};
    packet_in.increment = 28;
    packet_in.min_distance = 25139;
    packet_in.max_distance = 25243;
    packet_in.increment_f = 1123.0;
    packet_in.angle_offset = 1151.0;
    packet_in.frame = 119;

    mavlink::common::msg::OBSTACLE_DISTANCE packet1{};
    mavlink::common::msg::OBSTACLE_DISTANCE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.sensor_type, packet2.sensor_type);
    EXPECT_EQ(packet1.distances, packet2.distances);
    EXPECT_EQ(packet1.increment, packet2.increment);
    EXPECT_EQ(packet1.min_distance, packet2.min_distance);
    EXPECT_EQ(packet1.max_distance, packet2.max_distance);
    EXPECT_EQ(packet1.increment_f, packet2.increment_f);
    EXPECT_EQ(packet1.angle_offset, packet2.angle_offset);
    EXPECT_EQ(packet1.frame, packet2.frame);
}

#ifdef TEST_INTEROP
TEST(common_interop, OBSTACLE_DISTANCE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_obstacle_distance_t packet_c {
         93372036854775807ULL, { 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714, 17715, 17716, 17717, 17718, 17719, 17720, 17721, 17722 }, 25139, 25243, 217, 28, 1123.0, 1151.0, 119
    };

    mavlink::common::msg::OBSTACLE_DISTANCE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.sensor_type = 217;
    packet_in.distances = {{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714, 17715, 17716, 17717, 17718, 17719, 17720, 17721, 17722 }};
    packet_in.increment = 28;
    packet_in.min_distance = 25139;
    packet_in.max_distance = 25243;
    packet_in.increment_f = 1123.0;
    packet_in.angle_offset = 1151.0;
    packet_in.frame = 119;

    mavlink::common::msg::OBSTACLE_DISTANCE packet2{};

    mavlink_msg_obstacle_distance_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.sensor_type, packet2.sensor_type);
    EXPECT_EQ(packet_in.distances, packet2.distances);
    EXPECT_EQ(packet_in.increment, packet2.increment);
    EXPECT_EQ(packet_in.min_distance, packet2.min_distance);
    EXPECT_EQ(packet_in.max_distance, packet2.max_distance);
    EXPECT_EQ(packet_in.increment_f, packet2.increment_f);
    EXPECT_EQ(packet_in.angle_offset, packet2.angle_offset);
    EXPECT_EQ(packet_in.frame, packet2.frame);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ODOMETRY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ODOMETRY packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.frame_id = 177;
    packet_in.child_frame_id = 244;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.q = {{ 157.0, 158.0, 159.0, 160.0 }};
    packet_in.vx = 269.0;
    packet_in.vy = 297.0;
    packet_in.vz = 325.0;
    packet_in.rollspeed = 353.0;
    packet_in.pitchspeed = 381.0;
    packet_in.yawspeed = 409.0;
    packet_in.pose_covariance = {{ 437.0, 438.0, 439.0, 440.0, 441.0, 442.0, 443.0, 444.0, 445.0, 446.0, 447.0, 448.0, 449.0, 450.0, 451.0, 452.0, 453.0, 454.0, 455.0, 456.0, 457.0 }};
    packet_in.velocity_covariance = {{ 1025.0, 1026.0, 1027.0, 1028.0, 1029.0, 1030.0, 1031.0, 1032.0, 1033.0, 1034.0, 1035.0, 1036.0, 1037.0, 1038.0, 1039.0, 1040.0, 1041.0, 1042.0, 1043.0, 1044.0, 1045.0 }};
    packet_in.reset_counter = 55;
    packet_in.estimator_type = 122;
    packet_in.quality = -67;

    mavlink::common::msg::ODOMETRY packet1{};
    mavlink::common::msg::ODOMETRY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.frame_id, packet2.frame_id);
    EXPECT_EQ(packet1.child_frame_id, packet2.child_frame_id);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet1.pose_covariance, packet2.pose_covariance);
    EXPECT_EQ(packet1.velocity_covariance, packet2.velocity_covariance);
    EXPECT_EQ(packet1.reset_counter, packet2.reset_counter);
    EXPECT_EQ(packet1.estimator_type, packet2.estimator_type);
    EXPECT_EQ(packet1.quality, packet2.quality);
}

#ifdef TEST_INTEROP
TEST(common_interop, ODOMETRY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_odometry_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, { 157.0, 158.0, 159.0, 160.0 }, 269.0, 297.0, 325.0, 353.0, 381.0, 409.0, { 437.0, 438.0, 439.0, 440.0, 441.0, 442.0, 443.0, 444.0, 445.0, 446.0, 447.0, 448.0, 449.0, 450.0, 451.0, 452.0, 453.0, 454.0, 455.0, 456.0, 457.0 }, { 1025.0, 1026.0, 1027.0, 1028.0, 1029.0, 1030.0, 1031.0, 1032.0, 1033.0, 1034.0, 1035.0, 1036.0, 1037.0, 1038.0, 1039.0, 1040.0, 1041.0, 1042.0, 1043.0, 1044.0, 1045.0 }, 177, 244, 55, 122, -67
    };

    mavlink::common::msg::ODOMETRY packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.frame_id = 177;
    packet_in.child_frame_id = 244;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.q = {{ 157.0, 158.0, 159.0, 160.0 }};
    packet_in.vx = 269.0;
    packet_in.vy = 297.0;
    packet_in.vz = 325.0;
    packet_in.rollspeed = 353.0;
    packet_in.pitchspeed = 381.0;
    packet_in.yawspeed = 409.0;
    packet_in.pose_covariance = {{ 437.0, 438.0, 439.0, 440.0, 441.0, 442.0, 443.0, 444.0, 445.0, 446.0, 447.0, 448.0, 449.0, 450.0, 451.0, 452.0, 453.0, 454.0, 455.0, 456.0, 457.0 }};
    packet_in.velocity_covariance = {{ 1025.0, 1026.0, 1027.0, 1028.0, 1029.0, 1030.0, 1031.0, 1032.0, 1033.0, 1034.0, 1035.0, 1036.0, 1037.0, 1038.0, 1039.0, 1040.0, 1041.0, 1042.0, 1043.0, 1044.0, 1045.0 }};
    packet_in.reset_counter = 55;
    packet_in.estimator_type = 122;
    packet_in.quality = -67;

    mavlink::common::msg::ODOMETRY packet2{};

    mavlink_msg_odometry_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.frame_id, packet2.frame_id);
    EXPECT_EQ(packet_in.child_frame_id, packet2.child_frame_id);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);
    EXPECT_EQ(packet_in.pose_covariance, packet2.pose_covariance);
    EXPECT_EQ(packet_in.velocity_covariance, packet2.velocity_covariance);
    EXPECT_EQ(packet_in.reset_counter, packet2.reset_counter);
    EXPECT_EQ(packet_in.estimator_type, packet2.estimator_type);
    EXPECT_EQ(packet_in.quality, packet2.quality);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TRAJECTORY_REPRESENTATION_WAYPOINTS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.valid_points = 79;
    packet_in.pos_x = {{ 73.0, 74.0, 75.0, 76.0, 77.0 }};
    packet_in.pos_y = {{ 213.0, 214.0, 215.0, 216.0, 217.0 }};
    packet_in.pos_z = {{ 353.0, 354.0, 355.0, 356.0, 357.0 }};
    packet_in.vel_x = {{ 493.0, 494.0, 495.0, 496.0, 497.0 }};
    packet_in.vel_y = {{ 633.0, 634.0, 635.0, 636.0, 637.0 }};
    packet_in.vel_z = {{ 773.0, 774.0, 775.0, 776.0, 777.0 }};
    packet_in.acc_x = {{ 913.0, 914.0, 915.0, 916.0, 917.0 }};
    packet_in.acc_y = {{ 1053.0, 1054.0, 1055.0, 1056.0, 1057.0 }};
    packet_in.acc_z = {{ 1193.0, 1194.0, 1195.0, 1196.0, 1197.0 }};
    packet_in.pos_yaw = {{ 1333.0, 1334.0, 1335.0, 1336.0, 1337.0 }};
    packet_in.vel_yaw = {{ 1473.0, 1474.0, 1475.0, 1476.0, 1477.0 }};
    packet_in.command = {{ 29091, 29092, 29093, 29094, 29095 }};

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS packet1{};
    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.valid_points, packet2.valid_points);
    EXPECT_EQ(packet1.pos_x, packet2.pos_x);
    EXPECT_EQ(packet1.pos_y, packet2.pos_y);
    EXPECT_EQ(packet1.pos_z, packet2.pos_z);
    EXPECT_EQ(packet1.vel_x, packet2.vel_x);
    EXPECT_EQ(packet1.vel_y, packet2.vel_y);
    EXPECT_EQ(packet1.vel_z, packet2.vel_z);
    EXPECT_EQ(packet1.acc_x, packet2.acc_x);
    EXPECT_EQ(packet1.acc_y, packet2.acc_y);
    EXPECT_EQ(packet1.acc_z, packet2.acc_z);
    EXPECT_EQ(packet1.pos_yaw, packet2.pos_yaw);
    EXPECT_EQ(packet1.vel_yaw, packet2.vel_yaw);
    EXPECT_EQ(packet1.command, packet2.command);
}

#ifdef TEST_INTEROP
TEST(common_interop, TRAJECTORY_REPRESENTATION_WAYPOINTS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_trajectory_representation_waypoints_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0, 77.0 }, { 213.0, 214.0, 215.0, 216.0, 217.0 }, { 353.0, 354.0, 355.0, 356.0, 357.0 }, { 493.0, 494.0, 495.0, 496.0, 497.0 }, { 633.0, 634.0, 635.0, 636.0, 637.0 }, { 773.0, 774.0, 775.0, 776.0, 777.0 }, { 913.0, 914.0, 915.0, 916.0, 917.0 }, { 1053.0, 1054.0, 1055.0, 1056.0, 1057.0 }, { 1193.0, 1194.0, 1195.0, 1196.0, 1197.0 }, { 1333.0, 1334.0, 1335.0, 1336.0, 1337.0 }, { 1473.0, 1474.0, 1475.0, 1476.0, 1477.0 }, { 29091, 29092, 29093, 29094, 29095 }, 79
    };

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.valid_points = 79;
    packet_in.pos_x = {{ 73.0, 74.0, 75.0, 76.0, 77.0 }};
    packet_in.pos_y = {{ 213.0, 214.0, 215.0, 216.0, 217.0 }};
    packet_in.pos_z = {{ 353.0, 354.0, 355.0, 356.0, 357.0 }};
    packet_in.vel_x = {{ 493.0, 494.0, 495.0, 496.0, 497.0 }};
    packet_in.vel_y = {{ 633.0, 634.0, 635.0, 636.0, 637.0 }};
    packet_in.vel_z = {{ 773.0, 774.0, 775.0, 776.0, 777.0 }};
    packet_in.acc_x = {{ 913.0, 914.0, 915.0, 916.0, 917.0 }};
    packet_in.acc_y = {{ 1053.0, 1054.0, 1055.0, 1056.0, 1057.0 }};
    packet_in.acc_z = {{ 1193.0, 1194.0, 1195.0, 1196.0, 1197.0 }};
    packet_in.pos_yaw = {{ 1333.0, 1334.0, 1335.0, 1336.0, 1337.0 }};
    packet_in.vel_yaw = {{ 1473.0, 1474.0, 1475.0, 1476.0, 1477.0 }};
    packet_in.command = {{ 29091, 29092, 29093, 29094, 29095 }};

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS packet2{};

    mavlink_msg_trajectory_representation_waypoints_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.valid_points, packet2.valid_points);
    EXPECT_EQ(packet_in.pos_x, packet2.pos_x);
    EXPECT_EQ(packet_in.pos_y, packet2.pos_y);
    EXPECT_EQ(packet_in.pos_z, packet2.pos_z);
    EXPECT_EQ(packet_in.vel_x, packet2.vel_x);
    EXPECT_EQ(packet_in.vel_y, packet2.vel_y);
    EXPECT_EQ(packet_in.vel_z, packet2.vel_z);
    EXPECT_EQ(packet_in.acc_x, packet2.acc_x);
    EXPECT_EQ(packet_in.acc_y, packet2.acc_y);
    EXPECT_EQ(packet_in.acc_z, packet2.acc_z);
    EXPECT_EQ(packet_in.pos_yaw, packet2.pos_yaw);
    EXPECT_EQ(packet_in.vel_yaw, packet2.vel_yaw);
    EXPECT_EQ(packet_in.command, packet2.command);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TRAJECTORY_REPRESENTATION_BEZIER)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.valid_points = 73;
    packet_in.pos_x = {{ 73.0, 74.0, 75.0, 76.0, 77.0 }};
    packet_in.pos_y = {{ 213.0, 214.0, 215.0, 216.0, 217.0 }};
    packet_in.pos_z = {{ 353.0, 354.0, 355.0, 356.0, 357.0 }};
    packet_in.delta = {{ 493.0, 494.0, 495.0, 496.0, 497.0 }};
    packet_in.pos_yaw = {{ 633.0, 634.0, 635.0, 636.0, 637.0 }};

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER packet1{};
    mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.valid_points, packet2.valid_points);
    EXPECT_EQ(packet1.pos_x, packet2.pos_x);
    EXPECT_EQ(packet1.pos_y, packet2.pos_y);
    EXPECT_EQ(packet1.pos_z, packet2.pos_z);
    EXPECT_EQ(packet1.delta, packet2.delta);
    EXPECT_EQ(packet1.pos_yaw, packet2.pos_yaw);
}

#ifdef TEST_INTEROP
TEST(common_interop, TRAJECTORY_REPRESENTATION_BEZIER)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_trajectory_representation_bezier_t packet_c {
         93372036854775807ULL, { 73.0, 74.0, 75.0, 76.0, 77.0 }, { 213.0, 214.0, 215.0, 216.0, 217.0 }, { 353.0, 354.0, 355.0, 356.0, 357.0 }, { 493.0, 494.0, 495.0, 496.0, 497.0 }, { 633.0, 634.0, 635.0, 636.0, 637.0 }, 73
    };

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.valid_points = 73;
    packet_in.pos_x = {{ 73.0, 74.0, 75.0, 76.0, 77.0 }};
    packet_in.pos_y = {{ 213.0, 214.0, 215.0, 216.0, 217.0 }};
    packet_in.pos_z = {{ 353.0, 354.0, 355.0, 356.0, 357.0 }};
    packet_in.delta = {{ 493.0, 494.0, 495.0, 496.0, 497.0 }};
    packet_in.pos_yaw = {{ 633.0, 634.0, 635.0, 636.0, 637.0 }};

    mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER packet2{};

    mavlink_msg_trajectory_representation_bezier_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.valid_points, packet2.valid_points);
    EXPECT_EQ(packet_in.pos_x, packet2.pos_x);
    EXPECT_EQ(packet_in.pos_y, packet2.pos_y);
    EXPECT_EQ(packet_in.pos_z, packet2.pos_z);
    EXPECT_EQ(packet_in.delta, packet2.delta);
    EXPECT_EQ(packet_in.pos_yaw, packet2.pos_yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CELLULAR_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CELLULAR_STATUS packet_in{};
    packet_in.status = 151;
    packet_in.failure_reason = 218;
    packet_in.type = 29;
    packet_in.quality = 96;
    packet_in.mcc = 17235;
    packet_in.mnc = 17339;
    packet_in.lac = 17443;

    mavlink::common::msg::CELLULAR_STATUS packet1{};
    mavlink::common::msg::CELLULAR_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.status, packet2.status);
    EXPECT_EQ(packet1.failure_reason, packet2.failure_reason);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.quality, packet2.quality);
    EXPECT_EQ(packet1.mcc, packet2.mcc);
    EXPECT_EQ(packet1.mnc, packet2.mnc);
    EXPECT_EQ(packet1.lac, packet2.lac);
}

#ifdef TEST_INTEROP
TEST(common_interop, CELLULAR_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_cellular_status_t packet_c {
         17235, 17339, 17443, 151, 218, 29, 96
    };

    mavlink::common::msg::CELLULAR_STATUS packet_in{};
    packet_in.status = 151;
    packet_in.failure_reason = 218;
    packet_in.type = 29;
    packet_in.quality = 96;
    packet_in.mcc = 17235;
    packet_in.mnc = 17339;
    packet_in.lac = 17443;

    mavlink::common::msg::CELLULAR_STATUS packet2{};

    mavlink_msg_cellular_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.status, packet2.status);
    EXPECT_EQ(packet_in.failure_reason, packet2.failure_reason);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.quality, packet2.quality);
    EXPECT_EQ(packet_in.mcc, packet2.mcc);
    EXPECT_EQ(packet_in.mnc, packet2.mnc);
    EXPECT_EQ(packet_in.lac, packet2.lac);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ISBD_LINK_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ISBD_LINK_STATUS packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.last_heartbeat = 93372036854776311ULL;
    packet_in.failed_sessions = 18067;
    packet_in.successful_sessions = 18171;
    packet_in.signal_quality = 65;
    packet_in.ring_pending = 132;
    packet_in.tx_session_pending = 199;
    packet_in.rx_session_pending = 10;

    mavlink::common::msg::ISBD_LINK_STATUS packet1{};
    mavlink::common::msg::ISBD_LINK_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.last_heartbeat, packet2.last_heartbeat);
    EXPECT_EQ(packet1.failed_sessions, packet2.failed_sessions);
    EXPECT_EQ(packet1.successful_sessions, packet2.successful_sessions);
    EXPECT_EQ(packet1.signal_quality, packet2.signal_quality);
    EXPECT_EQ(packet1.ring_pending, packet2.ring_pending);
    EXPECT_EQ(packet1.tx_session_pending, packet2.tx_session_pending);
    EXPECT_EQ(packet1.rx_session_pending, packet2.rx_session_pending);
}

#ifdef TEST_INTEROP
TEST(common_interop, ISBD_LINK_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_isbd_link_status_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 18067, 18171, 65, 132, 199, 10
    };

    mavlink::common::msg::ISBD_LINK_STATUS packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.last_heartbeat = 93372036854776311ULL;
    packet_in.failed_sessions = 18067;
    packet_in.successful_sessions = 18171;
    packet_in.signal_quality = 65;
    packet_in.ring_pending = 132;
    packet_in.tx_session_pending = 199;
    packet_in.rx_session_pending = 10;

    mavlink::common::msg::ISBD_LINK_STATUS packet2{};

    mavlink_msg_isbd_link_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.last_heartbeat, packet2.last_heartbeat);
    EXPECT_EQ(packet_in.failed_sessions, packet2.failed_sessions);
    EXPECT_EQ(packet_in.successful_sessions, packet2.successful_sessions);
    EXPECT_EQ(packet_in.signal_quality, packet2.signal_quality);
    EXPECT_EQ(packet_in.ring_pending, packet2.ring_pending);
    EXPECT_EQ(packet_in.tx_session_pending, packet2.tx_session_pending);
    EXPECT_EQ(packet_in.rx_session_pending, packet2.rx_session_pending);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CELLULAR_CONFIG)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CELLULAR_CONFIG packet_in{};
    packet_in.enable_lte = 5;
    packet_in.enable_pin = 72;
    packet_in.pin = to_char_array("CDEFGHIJKLMNOPQ");
    packet_in.new_pin = to_char_array("STUVWXYZABCDEFG");
    packet_in.apn = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLM");
    packet_in.puk = to_char_array("OPQRSTUVWXYZABC");
    packet_in.roaming = 123;
    packet_in.response = 190;

    mavlink::common::msg::CELLULAR_CONFIG packet1{};
    mavlink::common::msg::CELLULAR_CONFIG packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.enable_lte, packet2.enable_lte);
    EXPECT_EQ(packet1.enable_pin, packet2.enable_pin);
    EXPECT_EQ(packet1.pin, packet2.pin);
    EXPECT_EQ(packet1.new_pin, packet2.new_pin);
    EXPECT_EQ(packet1.apn, packet2.apn);
    EXPECT_EQ(packet1.puk, packet2.puk);
    EXPECT_EQ(packet1.roaming, packet2.roaming);
    EXPECT_EQ(packet1.response, packet2.response);
}

#ifdef TEST_INTEROP
TEST(common_interop, CELLULAR_CONFIG)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_cellular_config_t packet_c {
         5, 72, "CDEFGHIJKLMNOPQ", "STUVWXYZABCDEFG", "IJKLMNOPQRSTUVWXYZABCDEFGHIJKLM", "OPQRSTUVWXYZABC", 123, 190
    };

    mavlink::common::msg::CELLULAR_CONFIG packet_in{};
    packet_in.enable_lte = 5;
    packet_in.enable_pin = 72;
    packet_in.pin = to_char_array("CDEFGHIJKLMNOPQ");
    packet_in.new_pin = to_char_array("STUVWXYZABCDEFG");
    packet_in.apn = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLM");
    packet_in.puk = to_char_array("OPQRSTUVWXYZABC");
    packet_in.roaming = 123;
    packet_in.response = 190;

    mavlink::common::msg::CELLULAR_CONFIG packet2{};

    mavlink_msg_cellular_config_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.enable_lte, packet2.enable_lte);
    EXPECT_EQ(packet_in.enable_pin, packet2.enable_pin);
    EXPECT_EQ(packet_in.pin, packet2.pin);
    EXPECT_EQ(packet_in.new_pin, packet2.new_pin);
    EXPECT_EQ(packet_in.apn, packet2.apn);
    EXPECT_EQ(packet_in.puk, packet2.puk);
    EXPECT_EQ(packet_in.roaming, packet2.roaming);
    EXPECT_EQ(packet_in.response, packet2.response);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RAW_RPM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RAW_RPM packet_in{};
    packet_in.index = 17;
    packet_in.frequency = 17.0;

    mavlink::common::msg::RAW_RPM packet1{};
    mavlink::common::msg::RAW_RPM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.index, packet2.index);
    EXPECT_EQ(packet1.frequency, packet2.frequency);
}

#ifdef TEST_INTEROP
TEST(common_interop, RAW_RPM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_raw_rpm_t packet_c {
         17.0, 17
    };

    mavlink::common::msg::RAW_RPM packet_in{};
    packet_in.index = 17;
    packet_in.frequency = 17.0;

    mavlink::common::msg::RAW_RPM packet2{};

    mavlink_msg_raw_rpm_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.index, packet2.index);
    EXPECT_EQ(packet_in.frequency, packet2.frequency);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, UTM_GLOBAL_POSITION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::UTM_GLOBAL_POSITION packet_in{};
    packet_in.time = 93372036854775807ULL;
    packet_in.uas_id = {{ 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 }};
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.relative_alt = 963498504;
    packet_in.vx = 19107;
    packet_in.vy = 19211;
    packet_in.vz = 19315;
    packet_in.h_acc = 19419;
    packet_in.v_acc = 19523;
    packet_in.vel_acc = 19627;
    packet_in.next_lat = 963498712;
    packet_in.next_lon = 963498920;
    packet_in.next_alt = 963499128;
    packet_in.update_rate = 19731;
    packet_in.flight_state = 209;
    packet_in.flags = 20;

    mavlink::common::msg::UTM_GLOBAL_POSITION packet1{};
    mavlink::common::msg::UTM_GLOBAL_POSITION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time, packet2.time);
    EXPECT_EQ(packet1.uas_id, packet2.uas_id);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.h_acc, packet2.h_acc);
    EXPECT_EQ(packet1.v_acc, packet2.v_acc);
    EXPECT_EQ(packet1.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet1.next_lat, packet2.next_lat);
    EXPECT_EQ(packet1.next_lon, packet2.next_lon);
    EXPECT_EQ(packet1.next_alt, packet2.next_alt);
    EXPECT_EQ(packet1.update_rate, packet2.update_rate);
    EXPECT_EQ(packet1.flight_state, packet2.flight_state);
    EXPECT_EQ(packet1.flags, packet2.flags);
}

#ifdef TEST_INTEROP
TEST(common_interop, UTM_GLOBAL_POSITION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_utm_global_position_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 963498504, 963498712, 963498920, 963499128, 19107, 19211, 19315, 19419, 19523, 19627, 19731, { 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 }, 209, 20
    };

    mavlink::common::msg::UTM_GLOBAL_POSITION packet_in{};
    packet_in.time = 93372036854775807ULL;
    packet_in.uas_id = {{ 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 }};
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.relative_alt = 963498504;
    packet_in.vx = 19107;
    packet_in.vy = 19211;
    packet_in.vz = 19315;
    packet_in.h_acc = 19419;
    packet_in.v_acc = 19523;
    packet_in.vel_acc = 19627;
    packet_in.next_lat = 963498712;
    packet_in.next_lon = 963498920;
    packet_in.next_alt = 963499128;
    packet_in.update_rate = 19731;
    packet_in.flight_state = 209;
    packet_in.flags = 20;

    mavlink::common::msg::UTM_GLOBAL_POSITION packet2{};

    mavlink_msg_utm_global_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time, packet2.time);
    EXPECT_EQ(packet_in.uas_id, packet2.uas_id);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.h_acc, packet2.h_acc);
    EXPECT_EQ(packet_in.v_acc, packet2.v_acc);
    EXPECT_EQ(packet_in.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet_in.next_lat, packet2.next_lat);
    EXPECT_EQ(packet_in.next_lon, packet2.next_lon);
    EXPECT_EQ(packet_in.next_alt, packet2.next_alt);
    EXPECT_EQ(packet_in.update_rate, packet2.update_rate);
    EXPECT_EQ(packet_in.flight_state, packet2.flight_state);
    EXPECT_EQ(packet_in.flags, packet2.flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, DEBUG_FLOAT_ARRAY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::DEBUG_FLOAT_ARRAY packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.name = to_char_array("KLMNOPQRS");
    packet_in.array_id = 17651;
    packet_in.data = {{ 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0, 166.0, 167.0, 168.0, 169.0, 170.0, 171.0, 172.0, 173.0, 174.0, 175.0, 176.0, 177.0, 178.0, 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0, 195.0, 196.0, 197.0, 198.0, 199.0, 200.0, 201.0, 202.0, 203.0, 204.0, 205.0, 206.0, 207.0, 208.0, 209.0, 210.0, 211.0, 212.0, 213.0, 214.0 }};

    mavlink::common::msg::DEBUG_FLOAT_ARRAY packet1{};
    mavlink::common::msg::DEBUG_FLOAT_ARRAY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.name, packet2.name);
    EXPECT_EQ(packet1.array_id, packet2.array_id);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, DEBUG_FLOAT_ARRAY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_debug_float_array_t packet_c {
         93372036854775807ULL, 17651, "KLMNOPQRS", { 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0, 166.0, 167.0, 168.0, 169.0, 170.0, 171.0, 172.0, 173.0, 174.0, 175.0, 176.0, 177.0, 178.0, 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0, 195.0, 196.0, 197.0, 198.0, 199.0, 200.0, 201.0, 202.0, 203.0, 204.0, 205.0, 206.0, 207.0, 208.0, 209.0, 210.0, 211.0, 212.0, 213.0, 214.0 }
    };

    mavlink::common::msg::DEBUG_FLOAT_ARRAY packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.name = to_char_array("KLMNOPQRS");
    packet_in.array_id = 17651;
    packet_in.data = {{ 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0, 166.0, 167.0, 168.0, 169.0, 170.0, 171.0, 172.0, 173.0, 174.0, 175.0, 176.0, 177.0, 178.0, 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0, 195.0, 196.0, 197.0, 198.0, 199.0, 200.0, 201.0, 202.0, 203.0, 204.0, 205.0, 206.0, 207.0, 208.0, 209.0, 210.0, 211.0, 212.0, 213.0, 214.0 }};

    mavlink::common::msg::DEBUG_FLOAT_ARRAY packet2{};

    mavlink_msg_debug_float_array_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.name, packet2.name);
    EXPECT_EQ(packet_in.array_id, packet2.array_id);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ORBIT_EXECUTION_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ORBIT_EXECUTION_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.radius = 73.0;
    packet_in.frame = 77;
    packet_in.x = 963498088;
    packet_in.y = 963498296;
    packet_in.z = 157.0;

    mavlink::common::msg::ORBIT_EXECUTION_STATUS packet1{};
    mavlink::common::msg::ORBIT_EXECUTION_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.radius, packet2.radius);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(common_interop, ORBIT_EXECUTION_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_orbit_execution_status_t packet_c {
         93372036854775807ULL, 73.0, 963498088, 963498296, 157.0, 77
    };

    mavlink::common::msg::ORBIT_EXECUTION_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.radius = 73.0;
    packet_in.frame = 77;
    packet_in.x = 963498088;
    packet_in.y = 963498296;
    packet_in.z = 157.0;

    mavlink::common::msg::ORBIT_EXECUTION_STATUS packet2{};

    mavlink_msg_orbit_execution_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.radius, packet2.radius);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SMART_BATTERY_INFO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SMART_BATTERY_INFO packet_in{};
    packet_in.id = 187;
    packet_in.battery_function = 254;
    packet_in.type = 65;
    packet_in.capacity_full_specification = 963497464;
    packet_in.capacity_full = 963497672;
    packet_in.cycle_count = 17651;
    packet_in.serial_number = to_char_array("VWXYZABCDEFGHIJ");
    packet_in.device_name = to_char_array("LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGH");
    packet_in.weight = 17755;
    packet_in.discharge_minimum_voltage = 17859;
    packet_in.charging_minimum_voltage = 17963;
    packet_in.resting_minimum_voltage = 18067;
    packet_in.charging_maximum_voltage = 21759;
    packet_in.cells_in_series = 80;
    packet_in.discharge_maximum_current = 963502144;
    packet_in.discharge_maximum_burst_current = 963502352;
    packet_in.manufacture_date = to_char_array("UVWXYZABCD");

    mavlink::common::msg::SMART_BATTERY_INFO packet1{};
    mavlink::common::msg::SMART_BATTERY_INFO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.battery_function, packet2.battery_function);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.capacity_full_specification, packet2.capacity_full_specification);
    EXPECT_EQ(packet1.capacity_full, packet2.capacity_full);
    EXPECT_EQ(packet1.cycle_count, packet2.cycle_count);
    EXPECT_EQ(packet1.serial_number, packet2.serial_number);
    EXPECT_EQ(packet1.device_name, packet2.device_name);
    EXPECT_EQ(packet1.weight, packet2.weight);
    EXPECT_EQ(packet1.discharge_minimum_voltage, packet2.discharge_minimum_voltage);
    EXPECT_EQ(packet1.charging_minimum_voltage, packet2.charging_minimum_voltage);
    EXPECT_EQ(packet1.resting_minimum_voltage, packet2.resting_minimum_voltage);
    EXPECT_EQ(packet1.charging_maximum_voltage, packet2.charging_maximum_voltage);
    EXPECT_EQ(packet1.cells_in_series, packet2.cells_in_series);
    EXPECT_EQ(packet1.discharge_maximum_current, packet2.discharge_maximum_current);
    EXPECT_EQ(packet1.discharge_maximum_burst_current, packet2.discharge_maximum_burst_current);
    EXPECT_EQ(packet1.manufacture_date, packet2.manufacture_date);
}

#ifdef TEST_INTEROP
TEST(common_interop, SMART_BATTERY_INFO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_smart_battery_info_t packet_c {
         963497464, 963497672, 17651, 17755, 17859, 17963, 18067, 187, 254, 65, "VWXYZABCDEFGHIJ", "LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGH", 21759, 80, 963502144, 963502352, "UVWXYZABCD"
    };

    mavlink::common::msg::SMART_BATTERY_INFO packet_in{};
    packet_in.id = 187;
    packet_in.battery_function = 254;
    packet_in.type = 65;
    packet_in.capacity_full_specification = 963497464;
    packet_in.capacity_full = 963497672;
    packet_in.cycle_count = 17651;
    packet_in.serial_number = to_char_array("VWXYZABCDEFGHIJ");
    packet_in.device_name = to_char_array("LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGH");
    packet_in.weight = 17755;
    packet_in.discharge_minimum_voltage = 17859;
    packet_in.charging_minimum_voltage = 17963;
    packet_in.resting_minimum_voltage = 18067;
    packet_in.charging_maximum_voltage = 21759;
    packet_in.cells_in_series = 80;
    packet_in.discharge_maximum_current = 963502144;
    packet_in.discharge_maximum_burst_current = 963502352;
    packet_in.manufacture_date = to_char_array("UVWXYZABCD");

    mavlink::common::msg::SMART_BATTERY_INFO packet2{};

    mavlink_msg_smart_battery_info_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.battery_function, packet2.battery_function);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.capacity_full_specification, packet2.capacity_full_specification);
    EXPECT_EQ(packet_in.capacity_full, packet2.capacity_full);
    EXPECT_EQ(packet_in.cycle_count, packet2.cycle_count);
    EXPECT_EQ(packet_in.serial_number, packet2.serial_number);
    EXPECT_EQ(packet_in.device_name, packet2.device_name);
    EXPECT_EQ(packet_in.weight, packet2.weight);
    EXPECT_EQ(packet_in.discharge_minimum_voltage, packet2.discharge_minimum_voltage);
    EXPECT_EQ(packet_in.charging_minimum_voltage, packet2.charging_minimum_voltage);
    EXPECT_EQ(packet_in.resting_minimum_voltage, packet2.resting_minimum_voltage);
    EXPECT_EQ(packet_in.charging_maximum_voltage, packet2.charging_maximum_voltage);
    EXPECT_EQ(packet_in.cells_in_series, packet2.cells_in_series);
    EXPECT_EQ(packet_in.discharge_maximum_current, packet2.discharge_maximum_current);
    EXPECT_EQ(packet_in.discharge_maximum_burst_current, packet2.discharge_maximum_burst_current);
    EXPECT_EQ(packet_in.manufacture_date, packet2.manufacture_date);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, FUEL_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::FUEL_STATUS packet_in{};
    packet_in.id = 77;
    packet_in.maximum_fuel = 17.0;
    packet_in.consumed_fuel = 45.0;
    packet_in.remaining_fuel = 73.0;
    packet_in.percent_remaining = 144;
    packet_in.flow_rate = 101.0;
    packet_in.temperature = 129.0;
    packet_in.fuel_type = 963498504;

    mavlink::common::msg::FUEL_STATUS packet1{};
    mavlink::common::msg::FUEL_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.maximum_fuel, packet2.maximum_fuel);
    EXPECT_EQ(packet1.consumed_fuel, packet2.consumed_fuel);
    EXPECT_EQ(packet1.remaining_fuel, packet2.remaining_fuel);
    EXPECT_EQ(packet1.percent_remaining, packet2.percent_remaining);
    EXPECT_EQ(packet1.flow_rate, packet2.flow_rate);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.fuel_type, packet2.fuel_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, FUEL_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_fuel_status_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 963498504, 77, 144
    };

    mavlink::common::msg::FUEL_STATUS packet_in{};
    packet_in.id = 77;
    packet_in.maximum_fuel = 17.0;
    packet_in.consumed_fuel = 45.0;
    packet_in.remaining_fuel = 73.0;
    packet_in.percent_remaining = 144;
    packet_in.flow_rate = 101.0;
    packet_in.temperature = 129.0;
    packet_in.fuel_type = 963498504;

    mavlink::common::msg::FUEL_STATUS packet2{};

    mavlink_msg_fuel_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.maximum_fuel, packet2.maximum_fuel);
    EXPECT_EQ(packet_in.consumed_fuel, packet2.consumed_fuel);
    EXPECT_EQ(packet_in.remaining_fuel, packet2.remaining_fuel);
    EXPECT_EQ(packet_in.percent_remaining, packet2.percent_remaining);
    EXPECT_EQ(packet_in.flow_rate, packet2.flow_rate);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.fuel_type, packet2.fuel_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, BATTERY_INFO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::BATTERY_INFO packet_in{};
    packet_in.id = 137;
    packet_in.battery_function = 204;
    packet_in.type = 15;
    packet_in.state_of_health = 82;
    packet_in.cells_in_series = 149;
    packet_in.cycle_count = 19315;
    packet_in.weight = 19419;
    packet_in.discharge_minimum_voltage = 17.0;
    packet_in.charging_minimum_voltage = 45.0;
    packet_in.resting_minimum_voltage = 73.0;
    packet_in.charging_maximum_voltage = 101.0;
    packet_in.charging_maximum_current = 129.0;
    packet_in.nominal_voltage = 157.0;
    packet_in.discharge_maximum_current = 185.0;
    packet_in.discharge_maximum_burst_current = 213.0;
    packet_in.design_capacity = 241.0;
    packet_in.full_charge_capacity = 269.0;
    packet_in.manufacture_date = to_char_array("XYZABCDE");
    packet_in.serial_number = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJK");
    packet_in.name = to_char_array("MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHI");

    mavlink::common::msg::BATTERY_INFO packet1{};
    mavlink::common::msg::BATTERY_INFO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.battery_function, packet2.battery_function);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.state_of_health, packet2.state_of_health);
    EXPECT_EQ(packet1.cells_in_series, packet2.cells_in_series);
    EXPECT_EQ(packet1.cycle_count, packet2.cycle_count);
    EXPECT_EQ(packet1.weight, packet2.weight);
    EXPECT_EQ(packet1.discharge_minimum_voltage, packet2.discharge_minimum_voltage);
    EXPECT_EQ(packet1.charging_minimum_voltage, packet2.charging_minimum_voltage);
    EXPECT_EQ(packet1.resting_minimum_voltage, packet2.resting_minimum_voltage);
    EXPECT_EQ(packet1.charging_maximum_voltage, packet2.charging_maximum_voltage);
    EXPECT_EQ(packet1.charging_maximum_current, packet2.charging_maximum_current);
    EXPECT_EQ(packet1.nominal_voltage, packet2.nominal_voltage);
    EXPECT_EQ(packet1.discharge_maximum_current, packet2.discharge_maximum_current);
    EXPECT_EQ(packet1.discharge_maximum_burst_current, packet2.discharge_maximum_burst_current);
    EXPECT_EQ(packet1.design_capacity, packet2.design_capacity);
    EXPECT_EQ(packet1.full_charge_capacity, packet2.full_charge_capacity);
    EXPECT_EQ(packet1.manufacture_date, packet2.manufacture_date);
    EXPECT_EQ(packet1.serial_number, packet2.serial_number);
    EXPECT_EQ(packet1.name, packet2.name);
}

#ifdef TEST_INTEROP
TEST(common_interop, BATTERY_INFO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_battery_info_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 19315, 19419, 137, 204, 15, 82, 149, "XYZABCDE", "GHIJKLMNOPQRSTUVWXYZABCDEFGHIJK", "MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHI"
    };

    mavlink::common::msg::BATTERY_INFO packet_in{};
    packet_in.id = 137;
    packet_in.battery_function = 204;
    packet_in.type = 15;
    packet_in.state_of_health = 82;
    packet_in.cells_in_series = 149;
    packet_in.cycle_count = 19315;
    packet_in.weight = 19419;
    packet_in.discharge_minimum_voltage = 17.0;
    packet_in.charging_minimum_voltage = 45.0;
    packet_in.resting_minimum_voltage = 73.0;
    packet_in.charging_maximum_voltage = 101.0;
    packet_in.charging_maximum_current = 129.0;
    packet_in.nominal_voltage = 157.0;
    packet_in.discharge_maximum_current = 185.0;
    packet_in.discharge_maximum_burst_current = 213.0;
    packet_in.design_capacity = 241.0;
    packet_in.full_charge_capacity = 269.0;
    packet_in.manufacture_date = to_char_array("XYZABCDE");
    packet_in.serial_number = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJK");
    packet_in.name = to_char_array("MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHI");

    mavlink::common::msg::BATTERY_INFO packet2{};

    mavlink_msg_battery_info_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.battery_function, packet2.battery_function);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.state_of_health, packet2.state_of_health);
    EXPECT_EQ(packet_in.cells_in_series, packet2.cells_in_series);
    EXPECT_EQ(packet_in.cycle_count, packet2.cycle_count);
    EXPECT_EQ(packet_in.weight, packet2.weight);
    EXPECT_EQ(packet_in.discharge_minimum_voltage, packet2.discharge_minimum_voltage);
    EXPECT_EQ(packet_in.charging_minimum_voltage, packet2.charging_minimum_voltage);
    EXPECT_EQ(packet_in.resting_minimum_voltage, packet2.resting_minimum_voltage);
    EXPECT_EQ(packet_in.charging_maximum_voltage, packet2.charging_maximum_voltage);
    EXPECT_EQ(packet_in.charging_maximum_current, packet2.charging_maximum_current);
    EXPECT_EQ(packet_in.nominal_voltage, packet2.nominal_voltage);
    EXPECT_EQ(packet_in.discharge_maximum_current, packet2.discharge_maximum_current);
    EXPECT_EQ(packet_in.discharge_maximum_burst_current, packet2.discharge_maximum_burst_current);
    EXPECT_EQ(packet_in.design_capacity, packet2.design_capacity);
    EXPECT_EQ(packet_in.full_charge_capacity, packet2.full_charge_capacity);
    EXPECT_EQ(packet_in.manufacture_date, packet2.manufacture_date);
    EXPECT_EQ(packet_in.serial_number, packet2.serial_number);
    EXPECT_EQ(packet_in.name, packet2.name);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GENERATOR_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GENERATOR_STATUS packet_in{};
    packet_in.status = 93372036854775807ULL;
    packet_in.generator_speed = 19107;
    packet_in.battery_current = 73.0;
    packet_in.load_current = 101.0;
    packet_in.power_generated = 129.0;
    packet_in.bus_voltage = 157.0;
    packet_in.rectifier_temperature = 19211;
    packet_in.bat_current_setpoint = 185.0;
    packet_in.generator_temperature = 19315;
    packet_in.runtime = 963498920;
    packet_in.time_until_maintenance = 963499128;

    mavlink::common::msg::GENERATOR_STATUS packet1{};
    mavlink::common::msg::GENERATOR_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.status, packet2.status);
    EXPECT_EQ(packet1.generator_speed, packet2.generator_speed);
    EXPECT_EQ(packet1.battery_current, packet2.battery_current);
    EXPECT_EQ(packet1.load_current, packet2.load_current);
    EXPECT_EQ(packet1.power_generated, packet2.power_generated);
    EXPECT_EQ(packet1.bus_voltage, packet2.bus_voltage);
    EXPECT_EQ(packet1.rectifier_temperature, packet2.rectifier_temperature);
    EXPECT_EQ(packet1.bat_current_setpoint, packet2.bat_current_setpoint);
    EXPECT_EQ(packet1.generator_temperature, packet2.generator_temperature);
    EXPECT_EQ(packet1.runtime, packet2.runtime);
    EXPECT_EQ(packet1.time_until_maintenance, packet2.time_until_maintenance);
}

#ifdef TEST_INTEROP
TEST(common_interop, GENERATOR_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_generator_status_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 963498920, 963499128, 19107, 19211, 19315
    };

    mavlink::common::msg::GENERATOR_STATUS packet_in{};
    packet_in.status = 93372036854775807ULL;
    packet_in.generator_speed = 19107;
    packet_in.battery_current = 73.0;
    packet_in.load_current = 101.0;
    packet_in.power_generated = 129.0;
    packet_in.bus_voltage = 157.0;
    packet_in.rectifier_temperature = 19211;
    packet_in.bat_current_setpoint = 185.0;
    packet_in.generator_temperature = 19315;
    packet_in.runtime = 963498920;
    packet_in.time_until_maintenance = 963499128;

    mavlink::common::msg::GENERATOR_STATUS packet2{};

    mavlink_msg_generator_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.status, packet2.status);
    EXPECT_EQ(packet_in.generator_speed, packet2.generator_speed);
    EXPECT_EQ(packet_in.battery_current, packet2.battery_current);
    EXPECT_EQ(packet_in.load_current, packet2.load_current);
    EXPECT_EQ(packet_in.power_generated, packet2.power_generated);
    EXPECT_EQ(packet_in.bus_voltage, packet2.bus_voltage);
    EXPECT_EQ(packet_in.rectifier_temperature, packet2.rectifier_temperature);
    EXPECT_EQ(packet_in.bat_current_setpoint, packet2.bat_current_setpoint);
    EXPECT_EQ(packet_in.generator_temperature, packet2.generator_temperature);
    EXPECT_EQ(packet_in.runtime, packet2.runtime);
    EXPECT_EQ(packet_in.time_until_maintenance, packet2.time_until_maintenance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ACTUATOR_OUTPUT_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.active = 963497880;
    packet_in.actuator = {{ 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0, 111.0, 112.0, 113.0, 114.0, 115.0, 116.0, 117.0, 118.0, 119.0, 120.0, 121.0, 122.0, 123.0, 124.0, 125.0, 126.0, 127.0, 128.0, 129.0, 130.0, 131.0, 132.0 }};

    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS packet1{};
    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.active, packet2.active);
    EXPECT_EQ(packet1.actuator, packet2.actuator);
}

#ifdef TEST_INTEROP
TEST(common_interop, ACTUATOR_OUTPUT_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_actuator_output_status_t packet_c {
         93372036854775807ULL, 963497880, { 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0, 111.0, 112.0, 113.0, 114.0, 115.0, 116.0, 117.0, 118.0, 119.0, 120.0, 121.0, 122.0, 123.0, 124.0, 125.0, 126.0, 127.0, 128.0, 129.0, 130.0, 131.0, 132.0 }
    };

    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.active = 963497880;
    packet_in.actuator = {{ 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0, 111.0, 112.0, 113.0, 114.0, 115.0, 116.0, 117.0, 118.0, 119.0, 120.0, 121.0, 122.0, 123.0, 124.0, 125.0, 126.0, 127.0, 128.0, 129.0, 130.0, 131.0, 132.0 }};

    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS packet2{};

    mavlink_msg_actuator_output_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.active, packet2.active);
    EXPECT_EQ(packet_in.actuator, packet2.actuator);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TIME_ESTIMATE_TO_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TIME_ESTIMATE_TO_TARGET packet_in{};
    packet_in.safe_return = 963497464;
    packet_in.land = 963497672;
    packet_in.mission_next_item = 963497880;
    packet_in.mission_end = 963498088;
    packet_in.commanded_action = 963498296;

    mavlink::common::msg::TIME_ESTIMATE_TO_TARGET packet1{};
    mavlink::common::msg::TIME_ESTIMATE_TO_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.safe_return, packet2.safe_return);
    EXPECT_EQ(packet1.land, packet2.land);
    EXPECT_EQ(packet1.mission_next_item, packet2.mission_next_item);
    EXPECT_EQ(packet1.mission_end, packet2.mission_end);
    EXPECT_EQ(packet1.commanded_action, packet2.commanded_action);
}

#ifdef TEST_INTEROP
TEST(common_interop, TIME_ESTIMATE_TO_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_time_estimate_to_target_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296
    };

    mavlink::common::msg::TIME_ESTIMATE_TO_TARGET packet_in{};
    packet_in.safe_return = 963497464;
    packet_in.land = 963497672;
    packet_in.mission_next_item = 963497880;
    packet_in.mission_end = 963498088;
    packet_in.commanded_action = 963498296;

    mavlink::common::msg::TIME_ESTIMATE_TO_TARGET packet2{};

    mavlink_msg_time_estimate_to_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.safe_return, packet2.safe_return);
    EXPECT_EQ(packet_in.land, packet2.land);
    EXPECT_EQ(packet_in.mission_next_item, packet2.mission_next_item);
    EXPECT_EQ(packet_in.mission_end, packet2.mission_end);
    EXPECT_EQ(packet_in.commanded_action, packet2.commanded_action);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TUNNEL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TUNNEL packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.payload_type = 17235;
    packet_in.payload_length = 17;
    packet_in.payload = {{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }};

    mavlink::common::msg::TUNNEL packet1{};
    mavlink::common::msg::TUNNEL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.payload_type, packet2.payload_type);
    EXPECT_EQ(packet1.payload_length, packet2.payload_length);
    EXPECT_EQ(packet1.payload, packet2.payload);
}

#ifdef TEST_INTEROP
TEST(common_interop, TUNNEL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_tunnel_t packet_c {
         17235, 139, 206, 17, { 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }
    };

    mavlink::common::msg::TUNNEL packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.payload_type = 17235;
    packet_in.payload_length = 17;
    packet_in.payload = {{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }};

    mavlink::common::msg::TUNNEL packet2{};

    mavlink_msg_tunnel_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.payload_type, packet2.payload_type);
    EXPECT_EQ(packet_in.payload_length, packet2.payload_length);
    EXPECT_EQ(packet_in.payload, packet2.payload);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAN_FRAME)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAN_FRAME packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.bus = 151;
    packet_in.len = 218;
    packet_in.id = 963497464;
    packet_in.data = {{ 29, 30, 31, 32, 33, 34, 35, 36 }};

    mavlink::common::msg::CAN_FRAME packet1{};
    mavlink::common::msg::CAN_FRAME packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.bus, packet2.bus);
    EXPECT_EQ(packet1.len, packet2.len);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAN_FRAME)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_can_frame_t packet_c {
         963497464, 17, 84, 151, 218, { 29, 30, 31, 32, 33, 34, 35, 36 }
    };

    mavlink::common::msg::CAN_FRAME packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.bus = 151;
    packet_in.len = 218;
    packet_in.id = 963497464;
    packet_in.data = {{ 29, 30, 31, 32, 33, 34, 35, 36 }};

    mavlink::common::msg::CAN_FRAME packet2{};

    mavlink_msg_can_frame_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.bus, packet2.bus);
    EXPECT_EQ(packet_in.len, packet2.len);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ONBOARD_COMPUTER_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ONBOARD_COMPUTER_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.uptime = 963497880;
    packet_in.type = 81;
    packet_in.cpu_cores = {{ 148, 149, 150, 151, 152, 153, 154, 155 }};
    packet_in.cpu_combined = {{ 172, 173, 174, 175, 176, 177, 178, 179, 180, 181 }};
    packet_in.gpu_cores = {{ 74, 75, 76, 77 }};
    packet_in.gpu_combined = {{ 86, 87, 88, 89, 90, 91, 92, 93, 94, 95 }};
    packet_in.temperature_board = -12;
    packet_in.temperature_core = {{ 55, 56, 57, 58, 59, 60, 61, 62 }};
    packet_in.fan_speed = {{ 27011, 27012, 27013, 27014 }};
    packet_in.ram_usage = 963498088;
    packet_in.ram_total = 963498296;
    packet_in.storage_type = {{ 963498504, 963498505, 963498506, 963498507 }};
    packet_in.storage_usage = {{ 963499336, 963499337, 963499338, 963499339 }};
    packet_in.storage_total = {{ 963500168, 963500169, 963500170, 963500171 }};
    packet_in.link_type = {{ 963501000, 963501001, 963501002, 963501003, 963501004, 963501005 }};
    packet_in.link_tx_rate = {{ 963502248, 963502249, 963502250, 963502251, 963502252, 963502253 }};
    packet_in.link_rx_rate = {{ 963503496, 963503497, 963503498, 963503499, 963503500, 963503501 }};
    packet_in.link_tx_max = {{ 963504744, 963504745, 963504746, 963504747, 963504748, 963504749 }};
    packet_in.link_rx_max = {{ 963505992, 963505993, 963505994, 963505995, 963505996, 963505997 }};

    mavlink::common::msg::ONBOARD_COMPUTER_STATUS packet1{};
    mavlink::common::msg::ONBOARD_COMPUTER_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.uptime, packet2.uptime);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.cpu_cores, packet2.cpu_cores);
    EXPECT_EQ(packet1.cpu_combined, packet2.cpu_combined);
    EXPECT_EQ(packet1.gpu_cores, packet2.gpu_cores);
    EXPECT_EQ(packet1.gpu_combined, packet2.gpu_combined);
    EXPECT_EQ(packet1.temperature_board, packet2.temperature_board);
    EXPECT_EQ(packet1.temperature_core, packet2.temperature_core);
    EXPECT_EQ(packet1.fan_speed, packet2.fan_speed);
    EXPECT_EQ(packet1.ram_usage, packet2.ram_usage);
    EXPECT_EQ(packet1.ram_total, packet2.ram_total);
    EXPECT_EQ(packet1.storage_type, packet2.storage_type);
    EXPECT_EQ(packet1.storage_usage, packet2.storage_usage);
    EXPECT_EQ(packet1.storage_total, packet2.storage_total);
    EXPECT_EQ(packet1.link_type, packet2.link_type);
    EXPECT_EQ(packet1.link_tx_rate, packet2.link_tx_rate);
    EXPECT_EQ(packet1.link_rx_rate, packet2.link_rx_rate);
    EXPECT_EQ(packet1.link_tx_max, packet2.link_tx_max);
    EXPECT_EQ(packet1.link_rx_max, packet2.link_rx_max);
}

#ifdef TEST_INTEROP
TEST(common_interop, ONBOARD_COMPUTER_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_onboard_computer_status_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, { 963498504, 963498505, 963498506, 963498507 }, { 963499336, 963499337, 963499338, 963499339 }, { 963500168, 963500169, 963500170, 963500171 }, { 963501000, 963501001, 963501002, 963501003, 963501004, 963501005 }, { 963502248, 963502249, 963502250, 963502251, 963502252, 963502253 }, { 963503496, 963503497, 963503498, 963503499, 963503500, 963503501 }, { 963504744, 963504745, 963504746, 963504747, 963504748, 963504749 }, { 963505992, 963505993, 963505994, 963505995, 963505996, 963505997 }, { 27011, 27012, 27013, 27014 }, 81, { 148, 149, 150, 151, 152, 153, 154, 155 }, { 172, 173, 174, 175, 176, 177, 178, 179, 180, 181 }, { 74, 75, 76, 77 }, { 86, 87, 88, 89, 90, 91, 92, 93, 94, 95 }, -12, { 55, 56, 57, 58, 59, 60, 61, 62 }
    };

    mavlink::common::msg::ONBOARD_COMPUTER_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.uptime = 963497880;
    packet_in.type = 81;
    packet_in.cpu_cores = {{ 148, 149, 150, 151, 152, 153, 154, 155 }};
    packet_in.cpu_combined = {{ 172, 173, 174, 175, 176, 177, 178, 179, 180, 181 }};
    packet_in.gpu_cores = {{ 74, 75, 76, 77 }};
    packet_in.gpu_combined = {{ 86, 87, 88, 89, 90, 91, 92, 93, 94, 95 }};
    packet_in.temperature_board = -12;
    packet_in.temperature_core = {{ 55, 56, 57, 58, 59, 60, 61, 62 }};
    packet_in.fan_speed = {{ 27011, 27012, 27013, 27014 }};
    packet_in.ram_usage = 963498088;
    packet_in.ram_total = 963498296;
    packet_in.storage_type = {{ 963498504, 963498505, 963498506, 963498507 }};
    packet_in.storage_usage = {{ 963499336, 963499337, 963499338, 963499339 }};
    packet_in.storage_total = {{ 963500168, 963500169, 963500170, 963500171 }};
    packet_in.link_type = {{ 963501000, 963501001, 963501002, 963501003, 963501004, 963501005 }};
    packet_in.link_tx_rate = {{ 963502248, 963502249, 963502250, 963502251, 963502252, 963502253 }};
    packet_in.link_rx_rate = {{ 963503496, 963503497, 963503498, 963503499, 963503500, 963503501 }};
    packet_in.link_tx_max = {{ 963504744, 963504745, 963504746, 963504747, 963504748, 963504749 }};
    packet_in.link_rx_max = {{ 963505992, 963505993, 963505994, 963505995, 963505996, 963505997 }};

    mavlink::common::msg::ONBOARD_COMPUTER_STATUS packet2{};

    mavlink_msg_onboard_computer_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.uptime, packet2.uptime);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.cpu_cores, packet2.cpu_cores);
    EXPECT_EQ(packet_in.cpu_combined, packet2.cpu_combined);
    EXPECT_EQ(packet_in.gpu_cores, packet2.gpu_cores);
    EXPECT_EQ(packet_in.gpu_combined, packet2.gpu_combined);
    EXPECT_EQ(packet_in.temperature_board, packet2.temperature_board);
    EXPECT_EQ(packet_in.temperature_core, packet2.temperature_core);
    EXPECT_EQ(packet_in.fan_speed, packet2.fan_speed);
    EXPECT_EQ(packet_in.ram_usage, packet2.ram_usage);
    EXPECT_EQ(packet_in.ram_total, packet2.ram_total);
    EXPECT_EQ(packet_in.storage_type, packet2.storage_type);
    EXPECT_EQ(packet_in.storage_usage, packet2.storage_usage);
    EXPECT_EQ(packet_in.storage_total, packet2.storage_total);
    EXPECT_EQ(packet_in.link_type, packet2.link_type);
    EXPECT_EQ(packet_in.link_tx_rate, packet2.link_tx_rate);
    EXPECT_EQ(packet_in.link_rx_rate, packet2.link_rx_rate);
    EXPECT_EQ(packet_in.link_tx_max, packet2.link_tx_max);
    EXPECT_EQ(packet_in.link_rx_max, packet2.link_rx_max);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMPONENT_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMPONENT_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.general_metadata_file_crc = 963497672;
    packet_in.general_metadata_uri = to_char_array("MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.peripherals_metadata_file_crc = 963497880;
    packet_in.peripherals_metadata_uri = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABC");

    mavlink::common::msg::COMPONENT_INFORMATION packet1{};
    mavlink::common::msg::COMPONENT_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.general_metadata_file_crc, packet2.general_metadata_file_crc);
    EXPECT_EQ(packet1.general_metadata_uri, packet2.general_metadata_uri);
    EXPECT_EQ(packet1.peripherals_metadata_file_crc, packet2.peripherals_metadata_file_crc);
    EXPECT_EQ(packet1.peripherals_metadata_uri, packet2.peripherals_metadata_uri);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMPONENT_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_component_information_t packet_c {
         963497464, 963497672, 963497880, "MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFG", "IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABC"
    };

    mavlink::common::msg::COMPONENT_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.general_metadata_file_crc = 963497672;
    packet_in.general_metadata_uri = to_char_array("MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.peripherals_metadata_file_crc = 963497880;
    packet_in.peripherals_metadata_uri = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABC");

    mavlink::common::msg::COMPONENT_INFORMATION packet2{};

    mavlink_msg_component_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.general_metadata_file_crc, packet2.general_metadata_file_crc);
    EXPECT_EQ(packet_in.general_metadata_uri, packet2.general_metadata_uri);
    EXPECT_EQ(packet_in.peripherals_metadata_file_crc, packet2.peripherals_metadata_file_crc);
    EXPECT_EQ(packet_in.peripherals_metadata_uri, packet2.peripherals_metadata_uri);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMPONENT_INFORMATION_BASIC)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMPONENT_INFORMATION_BASIC packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.time_manufacture_s = 963498088;
    packet_in.vendor_name = to_char_array("QRSTUVWXYZABCDEFGHIJKLMNOPQRSTU");
    packet_in.model_name = to_char_array("WXYZABCDEFGHIJKLMNOPQRSTUVWXYZA");
    packet_in.software_version = to_char_array("CDEFGHIJKLMNOPQRSTUVWXY");
    packet_in.hardware_version = to_char_array("ABCDEFGHIJKLMNOPQRSTUVW");
    packet_in.serial_number = to_char_array("YZABCDEFGHIJKLMNOPQRSTUVWXYZABC");

    mavlink::common::msg::COMPONENT_INFORMATION_BASIC packet1{};
    mavlink::common::msg::COMPONENT_INFORMATION_BASIC packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.capabilities, packet2.capabilities);
    EXPECT_EQ(packet1.time_manufacture_s, packet2.time_manufacture_s);
    EXPECT_EQ(packet1.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet1.model_name, packet2.model_name);
    EXPECT_EQ(packet1.software_version, packet2.software_version);
    EXPECT_EQ(packet1.hardware_version, packet2.hardware_version);
    EXPECT_EQ(packet1.serial_number, packet2.serial_number);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMPONENT_INFORMATION_BASIC)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_component_information_basic_t packet_c {
         93372036854775807ULL, 963497880, 963498088, "QRSTUVWXYZABCDEFGHIJKLMNOPQRSTU", "WXYZABCDEFGHIJKLMNOPQRSTUVWXYZA", "CDEFGHIJKLMNOPQRSTUVWXY", "ABCDEFGHIJKLMNOPQRSTUVW", "YZABCDEFGHIJKLMNOPQRSTUVWXYZABC"
    };

    mavlink::common::msg::COMPONENT_INFORMATION_BASIC packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.time_manufacture_s = 963498088;
    packet_in.vendor_name = to_char_array("QRSTUVWXYZABCDEFGHIJKLMNOPQRSTU");
    packet_in.model_name = to_char_array("WXYZABCDEFGHIJKLMNOPQRSTUVWXYZA");
    packet_in.software_version = to_char_array("CDEFGHIJKLMNOPQRSTUVWXY");
    packet_in.hardware_version = to_char_array("ABCDEFGHIJKLMNOPQRSTUVW");
    packet_in.serial_number = to_char_array("YZABCDEFGHIJKLMNOPQRSTUVWXYZABC");

    mavlink::common::msg::COMPONENT_INFORMATION_BASIC packet2{};

    mavlink_msg_component_information_basic_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.capabilities, packet2.capabilities);
    EXPECT_EQ(packet_in.time_manufacture_s, packet2.time_manufacture_s);
    EXPECT_EQ(packet_in.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet_in.model_name, packet2.model_name);
    EXPECT_EQ(packet_in.software_version, packet2.software_version);
    EXPECT_EQ(packet_in.hardware_version, packet2.hardware_version);
    EXPECT_EQ(packet_in.serial_number, packet2.serial_number);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMPONENT_METADATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMPONENT_METADATA packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.file_crc = 963497672;
    packet_in.uri = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABC");

    mavlink::common::msg::COMPONENT_METADATA packet1{};
    mavlink::common::msg::COMPONENT_METADATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.file_crc, packet2.file_crc);
    EXPECT_EQ(packet1.uri, packet2.uri);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMPONENT_METADATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_component_metadata_t packet_c {
         963497464, 963497672, "IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABC"
    };

    mavlink::common::msg::COMPONENT_METADATA packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.file_crc = 963497672;
    packet_in.uri = to_char_array("IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABC");

    mavlink::common::msg::COMPONENT_METADATA packet2{};

    mavlink_msg_component_metadata_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.file_crc, packet2.file_crc);
    EXPECT_EQ(packet_in.uri, packet2.uri);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PLAY_TUNE_V2)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PLAY_TUNE_V2 packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.format = 963497464;
    packet_in.tune = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS");

    mavlink::common::msg::PLAY_TUNE_V2 packet1{};
    mavlink::common::msg::PLAY_TUNE_V2 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.format, packet2.format);
    EXPECT_EQ(packet1.tune, packet2.tune);
}

#ifdef TEST_INTEROP
TEST(common_interop, PLAY_TUNE_V2)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_play_tune_v2_t packet_c {
         963497464, 17, 84, "GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS"
    };

    mavlink::common::msg::PLAY_TUNE_V2 packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.format = 963497464;
    packet_in.tune = to_char_array("GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS");

    mavlink::common::msg::PLAY_TUNE_V2 packet2{};

    mavlink_msg_play_tune_v2_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.format, packet2.format);
    EXPECT_EQ(packet_in.tune, packet2.tune);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SUPPORTED_TUNES)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SUPPORTED_TUNES packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.format = 963497464;

    mavlink::common::msg::SUPPORTED_TUNES packet1{};
    mavlink::common::msg::SUPPORTED_TUNES packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.format, packet2.format);
}

#ifdef TEST_INTEROP
TEST(common_interop, SUPPORTED_TUNES)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_supported_tunes_t packet_c {
         963497464, 17, 84
    };

    mavlink::common::msg::SUPPORTED_TUNES packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.format = 963497464;

    mavlink::common::msg::SUPPORTED_TUNES packet2{};

    mavlink_msg_supported_tunes_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.format, packet2.format);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, EVENT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::EVENT packet_in{};
    packet_in.destination_component = 163;
    packet_in.destination_system = 230;
    packet_in.id = 963497464;
    packet_in.event_time_boot_ms = 963497672;
    packet_in.sequence = 17651;
    packet_in.log_levels = 41;
    packet_in.arguments = {{ 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147 }};

    mavlink::common::msg::EVENT packet1{};
    mavlink::common::msg::EVENT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.destination_component, packet2.destination_component);
    EXPECT_EQ(packet1.destination_system, packet2.destination_system);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.event_time_boot_ms, packet2.event_time_boot_ms);
    EXPECT_EQ(packet1.sequence, packet2.sequence);
    EXPECT_EQ(packet1.log_levels, packet2.log_levels);
    EXPECT_EQ(packet1.arguments, packet2.arguments);
}

#ifdef TEST_INTEROP
TEST(common_interop, EVENT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_event_t packet_c {
         963497464, 963497672, 17651, 163, 230, 41, { 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147 }
    };

    mavlink::common::msg::EVENT packet_in{};
    packet_in.destination_component = 163;
    packet_in.destination_system = 230;
    packet_in.id = 963497464;
    packet_in.event_time_boot_ms = 963497672;
    packet_in.sequence = 17651;
    packet_in.log_levels = 41;
    packet_in.arguments = {{ 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147 }};

    mavlink::common::msg::EVENT packet2{};

    mavlink_msg_event_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.destination_component, packet2.destination_component);
    EXPECT_EQ(packet_in.destination_system, packet2.destination_system);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.event_time_boot_ms, packet2.event_time_boot_ms);
    EXPECT_EQ(packet_in.sequence, packet2.sequence);
    EXPECT_EQ(packet_in.log_levels, packet2.log_levels);
    EXPECT_EQ(packet_in.arguments, packet2.arguments);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CURRENT_EVENT_SEQUENCE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CURRENT_EVENT_SEQUENCE packet_in{};
    packet_in.sequence = 17235;
    packet_in.flags = 139;

    mavlink::common::msg::CURRENT_EVENT_SEQUENCE packet1{};
    mavlink::common::msg::CURRENT_EVENT_SEQUENCE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sequence, packet2.sequence);
    EXPECT_EQ(packet1.flags, packet2.flags);
}

#ifdef TEST_INTEROP
TEST(common_interop, CURRENT_EVENT_SEQUENCE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_current_event_sequence_t packet_c {
         17235, 139
    };

    mavlink::common::msg::CURRENT_EVENT_SEQUENCE packet_in{};
    packet_in.sequence = 17235;
    packet_in.flags = 139;

    mavlink::common::msg::CURRENT_EVENT_SEQUENCE packet2{};

    mavlink_msg_current_event_sequence_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sequence, packet2.sequence);
    EXPECT_EQ(packet_in.flags, packet2.flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, REQUEST_EVENT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::REQUEST_EVENT packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.first_sequence = 17235;
    packet_in.last_sequence = 17339;

    mavlink::common::msg::REQUEST_EVENT packet1{};
    mavlink::common::msg::REQUEST_EVENT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.first_sequence, packet2.first_sequence);
    EXPECT_EQ(packet1.last_sequence, packet2.last_sequence);
}

#ifdef TEST_INTEROP
TEST(common_interop, REQUEST_EVENT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_request_event_t packet_c {
         17235, 17339, 17, 84
    };

    mavlink::common::msg::REQUEST_EVENT packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.first_sequence = 17235;
    packet_in.last_sequence = 17339;

    mavlink::common::msg::REQUEST_EVENT packet2{};

    mavlink_msg_request_event_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.first_sequence, packet2.first_sequence);
    EXPECT_EQ(packet_in.last_sequence, packet2.last_sequence);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RESPONSE_EVENT_ERROR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RESPONSE_EVENT_ERROR packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.sequence = 17235;
    packet_in.sequence_oldest_available = 17339;
    packet_in.reason = 151;

    mavlink::common::msg::RESPONSE_EVENT_ERROR packet1{};
    mavlink::common::msg::RESPONSE_EVENT_ERROR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.sequence, packet2.sequence);
    EXPECT_EQ(packet1.sequence_oldest_available, packet2.sequence_oldest_available);
    EXPECT_EQ(packet1.reason, packet2.reason);
}

#ifdef TEST_INTEROP
TEST(common_interop, RESPONSE_EVENT_ERROR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_response_event_error_t packet_c {
         17235, 17339, 17, 84, 151
    };

    mavlink::common::msg::RESPONSE_EVENT_ERROR packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.sequence = 17235;
    packet_in.sequence_oldest_available = 17339;
    packet_in.reason = 151;

    mavlink::common::msg::RESPONSE_EVENT_ERROR packet2{};

    mavlink_msg_response_event_error_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.sequence, packet2.sequence);
    EXPECT_EQ(packet_in.sequence_oldest_available, packet2.sequence_oldest_available);
    EXPECT_EQ(packet_in.reason, packet2.reason);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, AVAILABLE_MODES)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::AVAILABLE_MODES packet_in{};
    packet_in.number_modes = 29;
    packet_in.mode_index = 96;
    packet_in.standard_mode = 163;
    packet_in.custom_mode = 963497464;
    packet_in.properties = 963497672;
    packet_in.mode_name = to_char_array("LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS");

    mavlink::common::msg::AVAILABLE_MODES packet1{};
    mavlink::common::msg::AVAILABLE_MODES packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.number_modes, packet2.number_modes);
    EXPECT_EQ(packet1.mode_index, packet2.mode_index);
    EXPECT_EQ(packet1.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.properties, packet2.properties);
    EXPECT_EQ(packet1.mode_name, packet2.mode_name);
}

#ifdef TEST_INTEROP
TEST(common_interop, AVAILABLE_MODES)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_available_modes_t packet_c {
         963497464, 963497672, 29, 96, 163, "LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS"
    };

    mavlink::common::msg::AVAILABLE_MODES packet_in{};
    packet_in.number_modes = 29;
    packet_in.mode_index = 96;
    packet_in.standard_mode = 163;
    packet_in.custom_mode = 963497464;
    packet_in.properties = 963497672;
    packet_in.mode_name = to_char_array("LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS");

    mavlink::common::msg::AVAILABLE_MODES packet2{};

    mavlink_msg_available_modes_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.number_modes, packet2.number_modes);
    EXPECT_EQ(packet_in.mode_index, packet2.mode_index);
    EXPECT_EQ(packet_in.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.properties, packet2.properties);
    EXPECT_EQ(packet_in.mode_name, packet2.mode_name);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CURRENT_MODE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CURRENT_MODE packet_in{};
    packet_in.standard_mode = 29;
    packet_in.custom_mode = 963497464;
    packet_in.intended_custom_mode = 963497672;

    mavlink::common::msg::CURRENT_MODE packet1{};
    mavlink::common::msg::CURRENT_MODE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.intended_custom_mode, packet2.intended_custom_mode);
}

#ifdef TEST_INTEROP
TEST(common_interop, CURRENT_MODE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_current_mode_t packet_c {
         963497464, 963497672, 29
    };

    mavlink::common::msg::CURRENT_MODE packet_in{};
    packet_in.standard_mode = 29;
    packet_in.custom_mode = 963497464;
    packet_in.intended_custom_mode = 963497672;

    mavlink::common::msg::CURRENT_MODE packet2{};

    mavlink_msg_current_mode_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.intended_custom_mode, packet2.intended_custom_mode);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, AVAILABLE_MODES_MONITOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::AVAILABLE_MODES_MONITOR packet_in{};
    packet_in.seq = 5;

    mavlink::common::msg::AVAILABLE_MODES_MONITOR packet1{};
    mavlink::common::msg::AVAILABLE_MODES_MONITOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(common_interop, AVAILABLE_MODES_MONITOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_available_modes_monitor_t packet_c {
         5
    };

    mavlink::common::msg::AVAILABLE_MODES_MONITOR packet_in{};
    packet_in.seq = 5;

    mavlink::common::msg::AVAILABLE_MODES_MONITOR packet2{};

    mavlink_msg_available_modes_monitor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ILLUMINATOR_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ILLUMINATOR_STATUS packet_in{};
    packet_in.uptime_ms = 963497464;
    packet_in.enable = 101;
    packet_in.mode_bitmask = 168;
    packet_in.error_status = 963497672;
    packet_in.mode = 235;
    packet_in.brightness = 73.0;
    packet_in.strobe_period = 101.0;
    packet_in.strobe_duty_cycle = 129.0;
    packet_in.temp_c = 157.0;
    packet_in.min_strobe_period = 185.0;
    packet_in.max_strobe_period = 213.0;

    mavlink::common::msg::ILLUMINATOR_STATUS packet1{};
    mavlink::common::msg::ILLUMINATOR_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.uptime_ms, packet2.uptime_ms);
    EXPECT_EQ(packet1.enable, packet2.enable);
    EXPECT_EQ(packet1.mode_bitmask, packet2.mode_bitmask);
    EXPECT_EQ(packet1.error_status, packet2.error_status);
    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.brightness, packet2.brightness);
    EXPECT_EQ(packet1.strobe_period, packet2.strobe_period);
    EXPECT_EQ(packet1.strobe_duty_cycle, packet2.strobe_duty_cycle);
    EXPECT_EQ(packet1.temp_c, packet2.temp_c);
    EXPECT_EQ(packet1.min_strobe_period, packet2.min_strobe_period);
    EXPECT_EQ(packet1.max_strobe_period, packet2.max_strobe_period);
}

#ifdef TEST_INTEROP
TEST(common_interop, ILLUMINATOR_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_illuminator_status_t packet_c {
         963497464, 963497672, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 101, 168, 235
    };

    mavlink::common::msg::ILLUMINATOR_STATUS packet_in{};
    packet_in.uptime_ms = 963497464;
    packet_in.enable = 101;
    packet_in.mode_bitmask = 168;
    packet_in.error_status = 963497672;
    packet_in.mode = 235;
    packet_in.brightness = 73.0;
    packet_in.strobe_period = 101.0;
    packet_in.strobe_duty_cycle = 129.0;
    packet_in.temp_c = 157.0;
    packet_in.min_strobe_period = 185.0;
    packet_in.max_strobe_period = 213.0;

    mavlink::common::msg::ILLUMINATOR_STATUS packet2{};

    mavlink_msg_illuminator_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.uptime_ms, packet2.uptime_ms);
    EXPECT_EQ(packet_in.enable, packet2.enable);
    EXPECT_EQ(packet_in.mode_bitmask, packet2.mode_bitmask);
    EXPECT_EQ(packet_in.error_status, packet2.error_status);
    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.brightness, packet2.brightness);
    EXPECT_EQ(packet_in.strobe_period, packet2.strobe_period);
    EXPECT_EQ(packet_in.strobe_duty_cycle, packet2.strobe_duty_cycle);
    EXPECT_EQ(packet_in.temp_c, packet2.temp_c);
    EXPECT_EQ(packet_in.min_strobe_period, packet2.min_strobe_period);
    EXPECT_EQ(packet_in.max_strobe_period, packet2.max_strobe_period);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CANFD_FRAME)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CANFD_FRAME packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.bus = 151;
    packet_in.len = 218;
    packet_in.id = 963497464;
    packet_in.data = {{ 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92 }};

    mavlink::common::msg::CANFD_FRAME packet1{};
    mavlink::common::msg::CANFD_FRAME packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.bus, packet2.bus);
    EXPECT_EQ(packet1.len, packet2.len);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(common_interop, CANFD_FRAME)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_canfd_frame_t packet_c {
         963497464, 17, 84, 151, 218, { 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92 }
    };

    mavlink::common::msg::CANFD_FRAME packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.bus = 151;
    packet_in.len = 218;
    packet_in.id = 963497464;
    packet_in.data = {{ 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92 }};

    mavlink::common::msg::CANFD_FRAME packet2{};

    mavlink_msg_canfd_frame_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.bus, packet2.bus);
    EXPECT_EQ(packet_in.len, packet2.len);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAN_FILTER_MODIFY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAN_FILTER_MODIFY packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.bus = 235;
    packet_in.operation = 46;
    packet_in.num_ids = 113;
    packet_in.ids = {{ 17235, 17236, 17237, 17238, 17239, 17240, 17241, 17242, 17243, 17244, 17245, 17246, 17247, 17248, 17249, 17250 }};

    mavlink::common::msg::CAN_FILTER_MODIFY packet1{};
    mavlink::common::msg::CAN_FILTER_MODIFY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.bus, packet2.bus);
    EXPECT_EQ(packet1.operation, packet2.operation);
    EXPECT_EQ(packet1.num_ids, packet2.num_ids);
    EXPECT_EQ(packet1.ids, packet2.ids);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAN_FILTER_MODIFY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_can_filter_modify_t packet_c {
         { 17235, 17236, 17237, 17238, 17239, 17240, 17241, 17242, 17243, 17244, 17245, 17246, 17247, 17248, 17249, 17250 }, 101, 168, 235, 46, 113
    };

    mavlink::common::msg::CAN_FILTER_MODIFY packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.bus = 235;
    packet_in.operation = 46;
    packet_in.num_ids = 113;
    packet_in.ids = {{ 17235, 17236, 17237, 17238, 17239, 17240, 17241, 17242, 17243, 17244, 17245, 17246, 17247, 17248, 17249, 17250 }};

    mavlink::common::msg::CAN_FILTER_MODIFY packet2{};

    mavlink_msg_can_filter_modify_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.bus, packet2.bus);
    EXPECT_EQ(packet_in.operation, packet2.operation);
    EXPECT_EQ(packet_in.num_ids, packet2.num_ids);
    EXPECT_EQ(packet_in.ids, packet2.ids);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, WHEEL_DISTANCE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::WHEEL_DISTANCE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.count = 157;
    packet_in.distance = {{ 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0 }};

    mavlink::common::msg::WHEEL_DISTANCE packet1{};
    mavlink::common::msg::WHEEL_DISTANCE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.count, packet2.count);
    EXPECT_EQ(packet1.distance, packet2.distance);
}

#ifdef TEST_INTEROP
TEST(common_interop, WHEEL_DISTANCE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_wheel_distance_t packet_c {
         93372036854775807ULL, { 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0 }, 157
    };

    mavlink::common::msg::WHEEL_DISTANCE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.count = 157;
    packet_in.distance = {{ 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0 }};

    mavlink::common::msg::WHEEL_DISTANCE packet2{};

    mavlink_msg_wheel_distance_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.count, packet2.count);
    EXPECT_EQ(packet_in.distance, packet2.distance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, WINCH_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::WINCH_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.line_length = 73.0;
    packet_in.speed = 101.0;
    packet_in.tension = 129.0;
    packet_in.voltage = 157.0;
    packet_in.current = 185.0;
    packet_in.temperature = 18899;
    packet_in.status = 963498920;

    mavlink::common::msg::WINCH_STATUS packet1{};
    mavlink::common::msg::WINCH_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.line_length, packet2.line_length);
    EXPECT_EQ(packet1.speed, packet2.speed);
    EXPECT_EQ(packet1.tension, packet2.tension);
    EXPECT_EQ(packet1.voltage, packet2.voltage);
    EXPECT_EQ(packet1.current, packet2.current);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.status, packet2.status);
}

#ifdef TEST_INTEROP
TEST(common_interop, WINCH_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_winch_status_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 963498920, 18899
    };

    mavlink::common::msg::WINCH_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.line_length = 73.0;
    packet_in.speed = 101.0;
    packet_in.tension = 129.0;
    packet_in.voltage = 157.0;
    packet_in.current = 185.0;
    packet_in.temperature = 18899;
    packet_in.status = 963498920;

    mavlink::common::msg::WINCH_STATUS packet2{};

    mavlink_msg_winch_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.line_length, packet2.line_length);
    EXPECT_EQ(packet_in.speed, packet2.speed);
    EXPECT_EQ(packet_in.tension, packet2.tension);
    EXPECT_EQ(packet_in.voltage, packet2.voltage);
    EXPECT_EQ(packet_in.current, packet2.current);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.status, packet2.status);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_BASIC_ID)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.id_type = 199;
    packet_in.ua_type = 10;
    packet_in.uas_id = {{ 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96 }};

    mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet1.id_type, packet2.id_type);
    EXPECT_EQ(packet1.ua_type, packet2.ua_type);
    EXPECT_EQ(packet1.uas_id, packet2.uas_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_BASIC_ID)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_basic_id_t packet_c {
         5, 72, { 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }, 199, 10, { 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96 }
    };

    mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.id_type = 199;
    packet_in.ua_type = 10;
    packet_in.uas_id = {{ 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96 }};

    mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID packet2{};

    mavlink_msg_open_drone_id_basic_id_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet_in.id_type, packet2.id_type);
    EXPECT_EQ(packet_in.ua_type, packet2.ua_type);
    EXPECT_EQ(packet_in.uas_id, packet2.uas_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_LOCATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_LOCATION packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.id_or_mac = {{ 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120 }};
    packet_in.status = 161;
    packet_in.direction = 18483;
    packet_in.speed_horizontal = 18587;
    packet_in.speed_vertical = 18691;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude_barometric = 73.0;
    packet_in.altitude_geodetic = 101.0;
    packet_in.height_reference = 228;
    packet_in.height = 129.0;
    packet_in.horizontal_accuracy = 39;
    packet_in.vertical_accuracy = 106;
    packet_in.barometer_accuracy = 173;
    packet_in.speed_accuracy = 240;
    packet_in.timestamp = 157.0;
    packet_in.timestamp_accuracy = 51;

    mavlink::common::msg::OPEN_DRONE_ID_LOCATION packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_LOCATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet1.status, packet2.status);
    EXPECT_EQ(packet1.direction, packet2.direction);
    EXPECT_EQ(packet1.speed_horizontal, packet2.speed_horizontal);
    EXPECT_EQ(packet1.speed_vertical, packet2.speed_vertical);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude_barometric, packet2.altitude_barometric);
    EXPECT_EQ(packet1.altitude_geodetic, packet2.altitude_geodetic);
    EXPECT_EQ(packet1.height_reference, packet2.height_reference);
    EXPECT_EQ(packet1.height, packet2.height);
    EXPECT_EQ(packet1.horizontal_accuracy, packet2.horizontal_accuracy);
    EXPECT_EQ(packet1.vertical_accuracy, packet2.vertical_accuracy);
    EXPECT_EQ(packet1.barometer_accuracy, packet2.barometer_accuracy);
    EXPECT_EQ(packet1.speed_accuracy, packet2.speed_accuracy);
    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.timestamp_accuracy, packet2.timestamp_accuracy);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_LOCATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_location_t packet_c {
         963497464, 963497672, 73.0, 101.0, 129.0, 157.0, 18483, 18587, 18691, 223, 34, { 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120 }, 161, 228, 39, 106, 173, 240, 51
    };

    mavlink::common::msg::OPEN_DRONE_ID_LOCATION packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.id_or_mac = {{ 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120 }};
    packet_in.status = 161;
    packet_in.direction = 18483;
    packet_in.speed_horizontal = 18587;
    packet_in.speed_vertical = 18691;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude_barometric = 73.0;
    packet_in.altitude_geodetic = 101.0;
    packet_in.height_reference = 228;
    packet_in.height = 129.0;
    packet_in.horizontal_accuracy = 39;
    packet_in.vertical_accuracy = 106;
    packet_in.barometer_accuracy = 173;
    packet_in.speed_accuracy = 240;
    packet_in.timestamp = 157.0;
    packet_in.timestamp_accuracy = 51;

    mavlink::common::msg::OPEN_DRONE_ID_LOCATION packet2{};

    mavlink_msg_open_drone_id_location_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet_in.status, packet2.status);
    EXPECT_EQ(packet_in.direction, packet2.direction);
    EXPECT_EQ(packet_in.speed_horizontal, packet2.speed_horizontal);
    EXPECT_EQ(packet_in.speed_vertical, packet2.speed_vertical);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude_barometric, packet2.altitude_barometric);
    EXPECT_EQ(packet_in.altitude_geodetic, packet2.altitude_geodetic);
    EXPECT_EQ(packet_in.height_reference, packet2.height_reference);
    EXPECT_EQ(packet_in.height, packet2.height);
    EXPECT_EQ(packet_in.horizontal_accuracy, packet2.horizontal_accuracy);
    EXPECT_EQ(packet_in.vertical_accuracy, packet2.vertical_accuracy);
    EXPECT_EQ(packet_in.barometer_accuracy, packet2.barometer_accuracy);
    EXPECT_EQ(packet_in.speed_accuracy, packet2.speed_accuracy);
    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.timestamp_accuracy, packet2.timestamp_accuracy);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_AUTHENTICATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_AUTHENTICATION packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.id_or_mac = {{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170 }};
    packet_in.authentication_type = 211;
    packet_in.data_page = 22;
    packet_in.last_page_index = 89;
    packet_in.length = 156;
    packet_in.timestamp = 963497464;
    packet_in.authentication_data = {{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245 }};

    mavlink::common::msg::OPEN_DRONE_ID_AUTHENTICATION packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_AUTHENTICATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet1.authentication_type, packet2.authentication_type);
    EXPECT_EQ(packet1.data_page, packet2.data_page);
    EXPECT_EQ(packet1.last_page_index, packet2.last_page_index);
    EXPECT_EQ(packet1.length, packet2.length);
    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.authentication_data, packet2.authentication_data);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_AUTHENTICATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_authentication_t packet_c {
         963497464, 17, 84, { 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170 }, 211, 22, 89, 156, { 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245 }
    };

    mavlink::common::msg::OPEN_DRONE_ID_AUTHENTICATION packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.id_or_mac = {{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170 }};
    packet_in.authentication_type = 211;
    packet_in.data_page = 22;
    packet_in.last_page_index = 89;
    packet_in.length = 156;
    packet_in.timestamp = 963497464;
    packet_in.authentication_data = {{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245 }};

    mavlink::common::msg::OPEN_DRONE_ID_AUTHENTICATION packet2{};

    mavlink_msg_open_drone_id_authentication_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet_in.authentication_type, packet2.authentication_type);
    EXPECT_EQ(packet_in.data_page, packet2.data_page);
    EXPECT_EQ(packet_in.last_page_index, packet2.last_page_index);
    EXPECT_EQ(packet_in.length, packet2.length);
    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.authentication_data, packet2.authentication_data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_SELF_ID)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_SELF_ID packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.description_type = 199;
    packet_in.description = to_char_array("XYZABCDEFGHIJKLMNOPQRS");

    mavlink::common::msg::OPEN_DRONE_ID_SELF_ID packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_SELF_ID packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet1.description_type, packet2.description_type);
    EXPECT_EQ(packet1.description, packet2.description);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_SELF_ID)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_self_id_t packet_c {
         5, 72, { 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }, 199, "XYZABCDEFGHIJKLMNOPQRS"
    };

    mavlink::common::msg::OPEN_DRONE_ID_SELF_ID packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.description_type = 199;
    packet_in.description = to_char_array("XYZABCDEFGHIJKLMNOPQRS");

    mavlink::common::msg::OPEN_DRONE_ID_SELF_ID packet2{};

    mavlink_msg_open_drone_id_self_id_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet_in.description_type, packet2.description_type);
    EXPECT_EQ(packet_in.description, packet2.description);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_SYSTEM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM packet_in{};
    packet_in.target_system = 89;
    packet_in.target_component = 156;
    packet_in.id_or_mac = {{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242 }};
    packet_in.operator_location_type = 27;
    packet_in.classification_type = 94;
    packet_in.operator_latitude = 963497464;
    packet_in.operator_longitude = 963497672;
    packet_in.area_count = 18483;
    packet_in.area_radius = 18587;
    packet_in.area_ceiling = 73.0;
    packet_in.area_floor = 101.0;
    packet_in.category_eu = 161;
    packet_in.class_eu = 228;
    packet_in.operator_altitude_geo = 129.0;
    packet_in.timestamp = 963498504;

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet1.operator_location_type, packet2.operator_location_type);
    EXPECT_EQ(packet1.classification_type, packet2.classification_type);
    EXPECT_EQ(packet1.operator_latitude, packet2.operator_latitude);
    EXPECT_EQ(packet1.operator_longitude, packet2.operator_longitude);
    EXPECT_EQ(packet1.area_count, packet2.area_count);
    EXPECT_EQ(packet1.area_radius, packet2.area_radius);
    EXPECT_EQ(packet1.area_ceiling, packet2.area_ceiling);
    EXPECT_EQ(packet1.area_floor, packet2.area_floor);
    EXPECT_EQ(packet1.category_eu, packet2.category_eu);
    EXPECT_EQ(packet1.class_eu, packet2.class_eu);
    EXPECT_EQ(packet1.operator_altitude_geo, packet2.operator_altitude_geo);
    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_SYSTEM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_system_t packet_c {
         963497464, 963497672, 73.0, 101.0, 129.0, 963498504, 18483, 18587, 89, 156, { 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242 }, 27, 94, 161, 228
    };

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM packet_in{};
    packet_in.target_system = 89;
    packet_in.target_component = 156;
    packet_in.id_or_mac = {{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242 }};
    packet_in.operator_location_type = 27;
    packet_in.classification_type = 94;
    packet_in.operator_latitude = 963497464;
    packet_in.operator_longitude = 963497672;
    packet_in.area_count = 18483;
    packet_in.area_radius = 18587;
    packet_in.area_ceiling = 73.0;
    packet_in.area_floor = 101.0;
    packet_in.category_eu = 161;
    packet_in.class_eu = 228;
    packet_in.operator_altitude_geo = 129.0;
    packet_in.timestamp = 963498504;

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM packet2{};

    mavlink_msg_open_drone_id_system_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet_in.operator_location_type, packet2.operator_location_type);
    EXPECT_EQ(packet_in.classification_type, packet2.classification_type);
    EXPECT_EQ(packet_in.operator_latitude, packet2.operator_latitude);
    EXPECT_EQ(packet_in.operator_longitude, packet2.operator_longitude);
    EXPECT_EQ(packet_in.area_count, packet2.area_count);
    EXPECT_EQ(packet_in.area_radius, packet2.area_radius);
    EXPECT_EQ(packet_in.area_ceiling, packet2.area_ceiling);
    EXPECT_EQ(packet_in.area_floor, packet2.area_floor);
    EXPECT_EQ(packet_in.category_eu, packet2.category_eu);
    EXPECT_EQ(packet_in.class_eu, packet2.class_eu);
    EXPECT_EQ(packet_in.operator_altitude_geo, packet2.operator_altitude_geo);
    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_OPERATOR_ID)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.operator_id_type = 199;
    packet_in.operator_id = to_char_array("XYZABCDEFGHIJKLMNOP");

    mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet1.operator_id_type, packet2.operator_id_type);
    EXPECT_EQ(packet1.operator_id, packet2.operator_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_OPERATOR_ID)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_operator_id_t packet_c {
         5, 72, { 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }, 199, "XYZABCDEFGHIJKLMNOP"
    };

    mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.operator_id_type = 199;
    packet_in.operator_id = to_char_array("XYZABCDEFGHIJKLMNOP");

    mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID packet2{};

    mavlink_msg_open_drone_id_operator_id_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet_in.operator_id_type, packet2.operator_id_type);
    EXPECT_EQ(packet_in.operator_id, packet2.operator_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_MESSAGE_PACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_MESSAGE_PACK packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.single_message_size = 199;
    packet_in.msg_pack_size = 10;
    packet_in.messages = {{ 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 }};

    mavlink::common::msg::OPEN_DRONE_ID_MESSAGE_PACK packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_MESSAGE_PACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet1.single_message_size, packet2.single_message_size);
    EXPECT_EQ(packet1.msg_pack_size, packet2.msg_pack_size);
    EXPECT_EQ(packet1.messages, packet2.messages);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_MESSAGE_PACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_message_pack_t packet_c {
         5, 72, { 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }, 199, 10, { 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 }
    };

    mavlink::common::msg::OPEN_DRONE_ID_MESSAGE_PACK packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;
    packet_in.id_or_mac = {{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }};
    packet_in.single_message_size = 199;
    packet_in.msg_pack_size = 10;
    packet_in.messages = {{ 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 }};

    mavlink::common::msg::OPEN_DRONE_ID_MESSAGE_PACK packet2{};

    mavlink_msg_open_drone_id_message_pack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.id_or_mac, packet2.id_or_mac);
    EXPECT_EQ(packet_in.single_message_size, packet2.single_message_size);
    EXPECT_EQ(packet_in.msg_pack_size, packet2.msg_pack_size);
    EXPECT_EQ(packet_in.messages, packet2.messages);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_ARM_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_ARM_STATUS packet_in{};
    packet_in.status = 5;
    packet_in.error = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX");

    mavlink::common::msg::OPEN_DRONE_ID_ARM_STATUS packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_ARM_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.status, packet2.status);
    EXPECT_EQ(packet1.error, packet2.error);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_ARM_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_arm_status_t packet_c {
         5, "BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX"
    };

    mavlink::common::msg::OPEN_DRONE_ID_ARM_STATUS packet_in{};
    packet_in.status = 5;
    packet_in.error = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX");

    mavlink::common::msg::OPEN_DRONE_ID_ARM_STATUS packet2{};

    mavlink_msg_open_drone_id_arm_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.status, packet2.status);
    EXPECT_EQ(packet_in.error, packet2.error);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, OPEN_DRONE_ID_SYSTEM_UPDATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE packet_in{};
    packet_in.target_system = 53;
    packet_in.target_component = 120;
    packet_in.operator_latitude = 963497464;
    packet_in.operator_longitude = 963497672;
    packet_in.operator_altitude_geo = 73.0;
    packet_in.timestamp = 963498088;

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE packet1{};
    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.operator_latitude, packet2.operator_latitude);
    EXPECT_EQ(packet1.operator_longitude, packet2.operator_longitude);
    EXPECT_EQ(packet1.operator_altitude_geo, packet2.operator_altitude_geo);
    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
}

#ifdef TEST_INTEROP
TEST(common_interop, OPEN_DRONE_ID_SYSTEM_UPDATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_open_drone_id_system_update_t packet_c {
         963497464, 963497672, 73.0, 963498088, 53, 120
    };

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE packet_in{};
    packet_in.target_system = 53;
    packet_in.target_component = 120;
    packet_in.operator_latitude = 963497464;
    packet_in.operator_longitude = 963497672;
    packet_in.operator_altitude_geo = 73.0;
    packet_in.timestamp = 963498088;

    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE packet2{};

    mavlink_msg_open_drone_id_system_update_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.operator_latitude, packet2.operator_latitude);
    EXPECT_EQ(packet_in.operator_longitude, packet2.operator_longitude);
    EXPECT_EQ(packet_in.operator_altitude_geo, packet2.operator_altitude_geo);
    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, HYGROMETER_SENSOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HYGROMETER_SENSOR packet_in{};
    packet_in.id = 17;
    packet_in.temperature = 17235;
    packet_in.humidity = 17339;

    mavlink::common::msg::HYGROMETER_SENSOR packet1{};
    mavlink::common::msg::HYGROMETER_SENSOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.humidity, packet2.humidity);
}

#ifdef TEST_INTEROP
TEST(common_interop, HYGROMETER_SENSOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_hygrometer_sensor_t packet_c {
         17235, 17339, 17
    };

    mavlink::common::msg::HYGROMETER_SENSOR packet_in{};
    packet_in.id = 17;
    packet_in.temperature = 17235;
    packet_in.humidity = 17339;

    mavlink::common::msg::HYGROMETER_SENSOR packet2{};

    mavlink_msg_hygrometer_sensor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.humidity, packet2.humidity);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, INTERCEPTION_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::INTERCEPTION_DATA packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.id = 113;
    packet_in.tgo = 73.0;
    packet_in.miss_distance = 101.0;
    packet_in.position_std_norm = 129.0;
    packet_in.substate = 963498504;
    packet_in.target_detected = 180;
    packet_in.estimated_relative_position = {{ 185.0, 186.0, 187.0 }};

    mavlink::common::msg::INTERCEPTION_DATA packet1{};
    mavlink::common::msg::INTERCEPTION_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.tgo, packet2.tgo);
    EXPECT_EQ(packet1.miss_distance, packet2.miss_distance);
    EXPECT_EQ(packet1.position_std_norm, packet2.position_std_norm);
    EXPECT_EQ(packet1.substate, packet2.substate);
    EXPECT_EQ(packet1.target_detected, packet2.target_detected);
    EXPECT_EQ(packet1.estimated_relative_position, packet2.estimated_relative_position);
}

#ifdef TEST_INTEROP
TEST(common_interop, INTERCEPTION_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_interception_data_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 963498504, { 185.0, 186.0, 187.0 }, 113, 180
    };

    mavlink::common::msg::INTERCEPTION_DATA packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.id = 113;
    packet_in.tgo = 73.0;
    packet_in.miss_distance = 101.0;
    packet_in.position_std_norm = 129.0;
    packet_in.substate = 963498504;
    packet_in.target_detected = 180;
    packet_in.estimated_relative_position = {{ 185.0, 186.0, 187.0 }};

    mavlink::common::msg::INTERCEPTION_DATA packet2{};

    mavlink_msg_interception_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.tgo, packet2.tgo);
    EXPECT_EQ(packet_in.miss_distance, packet2.miss_distance);
    EXPECT_EQ(packet_in.position_std_norm, packet2.position_std_norm);
    EXPECT_EQ(packet_in.substate, packet2.substate);
    EXPECT_EQ(packet_in.target_detected, packet2.target_detected);
    EXPECT_EQ(packet_in.estimated_relative_position, packet2.estimated_relative_position);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
