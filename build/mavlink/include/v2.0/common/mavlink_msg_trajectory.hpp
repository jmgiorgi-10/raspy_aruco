// MESSAGE TRAJECTORY support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief TRAJECTORY message
 *
 * WORK IN PROGRESS! DO NOT DEPLOY! Message to describe a trajectory in the local frame. Supported trajectory types are enumerated in MAV_TRAJECTORY_REPRESENTATION
 */
struct TRAJECTORY : mavlink::Message {
    static constexpr msgid_t MSG_ID = 332;
    static constexpr size_t LENGTH = 234;
    static constexpr size_t MIN_LENGTH = 234;
    static constexpr uint8_t CRC_EXTRA = 131;
    static constexpr auto NAME = "TRAJECTORY";


    uint64_t time_usec; /*< Timestamp (microseconds since system boot or since UNIX epoch). */
    uint8_t type; /*< Waypoints, Bezier etc. see MAV_TRAJECTORY_REPRESENTATION */
    std::array<float, 11> point_1; /*< Depending on the type (see MAV_TRAJECTORY_REPRESENTATION) */
    std::array<float, 11> point_2; /*< Depending on the type (see MAV_TRAJECTORY_REPRESENTATION) */
    std::array<float, 11> point_3; /*< Depending on the type (see MAV_TRAJECTORY_REPRESENTATION) */
    std::array<float, 11> point_4; /*< Depending on the type (see MAV_TRAJECTORY_REPRESENTATION) */
    std::array<float, 11> point_5; /*< Depending on the type (see MAV_TRAJECTORY_REPRESENTATION) */
    std::array<uint8_t, 5> point_valid; /*< States if respective point is valid (boolean) */


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
        ss << "  time_usec: " << time_usec << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  point_1: [" << to_string(point_1) << "]" << std::endl;
        ss << "  point_2: [" << to_string(point_2) << "]" << std::endl;
        ss << "  point_3: [" << to_string(point_3) << "]" << std::endl;
        ss << "  point_4: [" << to_string(point_4) << "]" << std::endl;
        ss << "  point_5: [" << to_string(point_5) << "]" << std::endl;
        ss << "  point_valid: [" << to_string(point_valid) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << point_1;                       // offset: 8
        map << point_2;                       // offset: 52
        map << point_3;                       // offset: 96
        map << point_4;                       // offset: 140
        map << point_5;                       // offset: 184
        map << type;                          // offset: 228
        map << point_valid;                   // offset: 229
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> point_1;                       // offset: 8
        map >> point_2;                       // offset: 52
        map >> point_3;                       // offset: 96
        map >> point_4;                       // offset: 140
        map >> point_5;                       // offset: 184
        map >> type;                          // offset: 228
        map >> point_valid;                   // offset: 229
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
