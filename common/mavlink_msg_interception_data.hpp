// MESSAGE INTERCEPTION_DATA support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief INTERCEPTION_DATA message
 *
 * Custom interceptor telemetry
 */
struct INTERCEPTION_DATA : mavlink::Message {
    static constexpr msgid_t MSG_ID = 669;
    static constexpr size_t LENGTH = 38;
    static constexpr size_t MIN_LENGTH = 38;
    static constexpr uint8_t CRC_EXTRA = 221;
    static constexpr auto NAME = "INTERCEPTION_DATA";


    uint64_t timestamp; /*< [us] Time since system boot */
    uint8_t id; /*<  Interceptor ID */
    float tgo; /*<  Time to go */
    float miss_distance; /*<  Miss distance */
    float position_std_norm; /*<  Position std norm */
    uint32_t substate; /*<  Substate */
    uint8_t target_detected; /*<  Target detection flag */
    std::array<float, 3> estimated_relative_position; /*<  Estimated relative position [x,y,z] */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  timestamp: " << timestamp << std::endl;
        ss << "  id: " << +id << std::endl;
        ss << "  tgo: " << tgo << std::endl;
        ss << "  miss_distance: " << miss_distance << std::endl;
        ss << "  position_std_norm: " << position_std_norm << std::endl;
        ss << "  substate: " << substate << std::endl;
        ss << "  target_detected: " << +target_detected << std::endl;
        ss << "  estimated_relative_position: [" << to_string(estimated_relative_position) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp;                     // offset: 0
        map << tgo;                           // offset: 8
        map << miss_distance;                 // offset: 12
        map << position_std_norm;             // offset: 16
        map << substate;                      // offset: 20
        map << estimated_relative_position;   // offset: 24
        map << id;                            // offset: 36
        map << target_detected;               // offset: 37
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp;                     // offset: 0
        map >> tgo;                           // offset: 8
        map >> miss_distance;                 // offset: 12
        map >> position_std_norm;             // offset: 16
        map >> substate;                      // offset: 20
        map >> estimated_relative_position;   // offset: 24
        map >> id;                            // offset: 36
        map >> target_detected;               // offset: 37
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
