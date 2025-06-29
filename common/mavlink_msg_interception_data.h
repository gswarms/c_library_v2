#pragma once
// MESSAGE INTERCEPTION_DATA PACKING

#define MAVLINK_MSG_ID_INTERCEPTION_DATA 669


typedef struct __mavlink_interception_data_t {
 uint64_t timestamp; /*< [us] Time since system boot*/
 float tgo; /*<  Time to go*/
 float miss_distance; /*<  Miss distance*/
 float position_std_norm; /*<  Position std norm*/
 uint32_t substate; /*<  Substate*/
 float estimated_relative_position[3]; /*<  Estimated relative position [x,y,z]*/
 uint8_t id; /*<  Interceptor ID*/
 uint8_t target_detected; /*<  Target detection flag*/
} mavlink_interception_data_t;

#define MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN 38
#define MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN 38
#define MAVLINK_MSG_ID_669_LEN 38
#define MAVLINK_MSG_ID_669_MIN_LEN 38

#define MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC 221
#define MAVLINK_MSG_ID_669_CRC 221

#define MAVLINK_MSG_INTERCEPTION_DATA_FIELD_ESTIMATED_RELATIVE_POSITION_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INTERCEPTION_DATA { \
    669, \
    "INTERCEPTION_DATA", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_interception_data_t, timestamp) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_interception_data_t, id) }, \
         { "tgo", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_interception_data_t, tgo) }, \
         { "miss_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_interception_data_t, miss_distance) }, \
         { "position_std_norm", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_interception_data_t, position_std_norm) }, \
         { "substate", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_interception_data_t, substate) }, \
         { "target_detected", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_interception_data_t, target_detected) }, \
         { "estimated_relative_position", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_interception_data_t, estimated_relative_position) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INTERCEPTION_DATA { \
    "INTERCEPTION_DATA", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_interception_data_t, timestamp) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_interception_data_t, id) }, \
         { "tgo", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_interception_data_t, tgo) }, \
         { "miss_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_interception_data_t, miss_distance) }, \
         { "position_std_norm", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_interception_data_t, position_std_norm) }, \
         { "substate", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_interception_data_t, substate) }, \
         { "target_detected", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_interception_data_t, target_detected) }, \
         { "estimated_relative_position", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_interception_data_t, estimated_relative_position) }, \
         } \
}
#endif

/**
 * @brief Pack a interception_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Time since system boot
 * @param id  Interceptor ID
 * @param tgo  Time to go
 * @param miss_distance  Miss distance
 * @param position_std_norm  Position std norm
 * @param substate  Substate
 * @param target_detected  Target detection flag
 * @param estimated_relative_position  Estimated relative position [x,y,z]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_interception_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t id, float tgo, float miss_distance, float position_std_norm, uint32_t substate, uint8_t target_detected, const float *estimated_relative_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, tgo);
    _mav_put_float(buf, 12, miss_distance);
    _mav_put_float(buf, 16, position_std_norm);
    _mav_put_uint32_t(buf, 20, substate);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, target_detected);
    _mav_put_float_array(buf, 24, estimated_relative_position, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
#else
    mavlink_interception_data_t packet;
    packet.timestamp = timestamp;
    packet.tgo = tgo;
    packet.miss_distance = miss_distance;
    packet.position_std_norm = position_std_norm;
    packet.substate = substate;
    packet.id = id;
    packet.target_detected = target_detected;
    mav_array_memcpy(packet.estimated_relative_position, estimated_relative_position, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTERCEPTION_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
}

/**
 * @brief Pack a interception_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Time since system boot
 * @param id  Interceptor ID
 * @param tgo  Time to go
 * @param miss_distance  Miss distance
 * @param position_std_norm  Position std norm
 * @param substate  Substate
 * @param target_detected  Target detection flag
 * @param estimated_relative_position  Estimated relative position [x,y,z]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_interception_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t id, float tgo, float miss_distance, float position_std_norm, uint32_t substate, uint8_t target_detected, const float *estimated_relative_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, tgo);
    _mav_put_float(buf, 12, miss_distance);
    _mav_put_float(buf, 16, position_std_norm);
    _mav_put_uint32_t(buf, 20, substate);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, target_detected);
    _mav_put_float_array(buf, 24, estimated_relative_position, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
#else
    mavlink_interception_data_t packet;
    packet.timestamp = timestamp;
    packet.tgo = tgo;
    packet.miss_distance = miss_distance;
    packet.position_std_norm = position_std_norm;
    packet.substate = substate;
    packet.id = id;
    packet.target_detected = target_detected;
    mav_array_memcpy(packet.estimated_relative_position, estimated_relative_position, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTERCEPTION_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
#endif
}

/**
 * @brief Pack a interception_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Time since system boot
 * @param id  Interceptor ID
 * @param tgo  Time to go
 * @param miss_distance  Miss distance
 * @param position_std_norm  Position std norm
 * @param substate  Substate
 * @param target_detected  Target detection flag
 * @param estimated_relative_position  Estimated relative position [x,y,z]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_interception_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t id,float tgo,float miss_distance,float position_std_norm,uint32_t substate,uint8_t target_detected,const float *estimated_relative_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, tgo);
    _mav_put_float(buf, 12, miss_distance);
    _mav_put_float(buf, 16, position_std_norm);
    _mav_put_uint32_t(buf, 20, substate);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, target_detected);
    _mav_put_float_array(buf, 24, estimated_relative_position, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
#else
    mavlink_interception_data_t packet;
    packet.timestamp = timestamp;
    packet.tgo = tgo;
    packet.miss_distance = miss_distance;
    packet.position_std_norm = position_std_norm;
    packet.substate = substate;
    packet.id = id;
    packet.target_detected = target_detected;
    mav_array_memcpy(packet.estimated_relative_position, estimated_relative_position, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INTERCEPTION_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
}

/**
 * @brief Encode a interception_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param interception_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_interception_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_interception_data_t* interception_data)
{
    return mavlink_msg_interception_data_pack(system_id, component_id, msg, interception_data->timestamp, interception_data->id, interception_data->tgo, interception_data->miss_distance, interception_data->position_std_norm, interception_data->substate, interception_data->target_detected, interception_data->estimated_relative_position);
}

/**
 * @brief Encode a interception_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param interception_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_interception_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_interception_data_t* interception_data)
{
    return mavlink_msg_interception_data_pack_chan(system_id, component_id, chan, msg, interception_data->timestamp, interception_data->id, interception_data->tgo, interception_data->miss_distance, interception_data->position_std_norm, interception_data->substate, interception_data->target_detected, interception_data->estimated_relative_position);
}

/**
 * @brief Encode a interception_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param interception_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_interception_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_interception_data_t* interception_data)
{
    return mavlink_msg_interception_data_pack_status(system_id, component_id, _status, msg,  interception_data->timestamp, interception_data->id, interception_data->tgo, interception_data->miss_distance, interception_data->position_std_norm, interception_data->substate, interception_data->target_detected, interception_data->estimated_relative_position);
}

/**
 * @brief Send a interception_data message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Time since system boot
 * @param id  Interceptor ID
 * @param tgo  Time to go
 * @param miss_distance  Miss distance
 * @param position_std_norm  Position std norm
 * @param substate  Substate
 * @param target_detected  Target detection flag
 * @param estimated_relative_position  Estimated relative position [x,y,z]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_interception_data_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t id, float tgo, float miss_distance, float position_std_norm, uint32_t substate, uint8_t target_detected, const float *estimated_relative_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, tgo);
    _mav_put_float(buf, 12, miss_distance);
    _mav_put_float(buf, 16, position_std_norm);
    _mav_put_uint32_t(buf, 20, substate);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, target_detected);
    _mav_put_float_array(buf, 24, estimated_relative_position, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTERCEPTION_DATA, buf, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
#else
    mavlink_interception_data_t packet;
    packet.timestamp = timestamp;
    packet.tgo = tgo;
    packet.miss_distance = miss_distance;
    packet.position_std_norm = position_std_norm;
    packet.substate = substate;
    packet.id = id;
    packet.target_detected = target_detected;
    mav_array_memcpy(packet.estimated_relative_position, estimated_relative_position, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTERCEPTION_DATA, (const char *)&packet, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
#endif
}

/**
 * @brief Send a interception_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_interception_data_send_struct(mavlink_channel_t chan, const mavlink_interception_data_t* interception_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_interception_data_send(chan, interception_data->timestamp, interception_data->id, interception_data->tgo, interception_data->miss_distance, interception_data->position_std_norm, interception_data->substate, interception_data->target_detected, interception_data->estimated_relative_position);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTERCEPTION_DATA, (const char *)interception_data, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_interception_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t id, float tgo, float miss_distance, float position_std_norm, uint32_t substate, uint8_t target_detected, const float *estimated_relative_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, tgo);
    _mav_put_float(buf, 12, miss_distance);
    _mav_put_float(buf, 16, position_std_norm);
    _mav_put_uint32_t(buf, 20, substate);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, target_detected);
    _mav_put_float_array(buf, 24, estimated_relative_position, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTERCEPTION_DATA, buf, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
#else
    mavlink_interception_data_t *packet = (mavlink_interception_data_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->tgo = tgo;
    packet->miss_distance = miss_distance;
    packet->position_std_norm = position_std_norm;
    packet->substate = substate;
    packet->id = id;
    packet->target_detected = target_detected;
    mav_array_memcpy(packet->estimated_relative_position, estimated_relative_position, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTERCEPTION_DATA, (const char *)packet, MAVLINK_MSG_ID_INTERCEPTION_DATA_MIN_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN, MAVLINK_MSG_ID_INTERCEPTION_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE INTERCEPTION_DATA UNPACKING


/**
 * @brief Get field timestamp from interception_data message
 *
 * @return [us] Time since system boot
 */
static inline uint64_t mavlink_msg_interception_data_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field id from interception_data message
 *
 * @return  Interceptor ID
 */
static inline uint8_t mavlink_msg_interception_data_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field tgo from interception_data message
 *
 * @return  Time to go
 */
static inline float mavlink_msg_interception_data_get_tgo(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field miss_distance from interception_data message
 *
 * @return  Miss distance
 */
static inline float mavlink_msg_interception_data_get_miss_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field position_std_norm from interception_data message
 *
 * @return  Position std norm
 */
static inline float mavlink_msg_interception_data_get_position_std_norm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field substate from interception_data message
 *
 * @return  Substate
 */
static inline uint32_t mavlink_msg_interception_data_get_substate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field target_detected from interception_data message
 *
 * @return  Target detection flag
 */
static inline uint8_t mavlink_msg_interception_data_get_target_detected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field estimated_relative_position from interception_data message
 *
 * @return  Estimated relative position [x,y,z]
 */
static inline uint16_t mavlink_msg_interception_data_get_estimated_relative_position(const mavlink_message_t* msg, float *estimated_relative_position)
{
    return _MAV_RETURN_float_array(msg, estimated_relative_position, 3,  24);
}

/**
 * @brief Decode a interception_data message into a struct
 *
 * @param msg The message to decode
 * @param interception_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_interception_data_decode(const mavlink_message_t* msg, mavlink_interception_data_t* interception_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    interception_data->timestamp = mavlink_msg_interception_data_get_timestamp(msg);
    interception_data->tgo = mavlink_msg_interception_data_get_tgo(msg);
    interception_data->miss_distance = mavlink_msg_interception_data_get_miss_distance(msg);
    interception_data->position_std_norm = mavlink_msg_interception_data_get_position_std_norm(msg);
    interception_data->substate = mavlink_msg_interception_data_get_substate(msg);
    mavlink_msg_interception_data_get_estimated_relative_position(msg, interception_data->estimated_relative_position);
    interception_data->id = mavlink_msg_interception_data_get_id(msg);
    interception_data->target_detected = mavlink_msg_interception_data_get_target_detected(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN? msg->len : MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN;
        memset(interception_data, 0, MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN);
    memcpy(interception_data, _MAV_PAYLOAD(msg), len);
#endif
}
