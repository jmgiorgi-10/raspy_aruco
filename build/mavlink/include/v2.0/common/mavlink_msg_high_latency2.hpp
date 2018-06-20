// MESSAGE HIGH_LATENCY2 support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief HIGH_LATENCY2 message
 *
 * WIP: Message appropriate for high latency connections like Iridium (version 2)
 */
struct HIGH_LATENCY2 : mavlink::Message {
    static constexpr msgid_t MSG_ID = 235;
    static constexpr size_t LENGTH = 42;
    static constexpr size_t MIN_LENGTH = 42;
    static constexpr uint8_t CRC_EXTRA = 179;
    static constexpr auto NAME = "HIGH_LATENCY2";


    uint32_t timestamp; /*< Timestamp (milliseconds since boot or Unix epoch) */
    uint8_t type; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM) */
    uint8_t autopilot; /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM */
    uint16_t custom_mode; /*< A bitfield for use for autopilot-specific flags (2 byte version). */
    int32_t latitude; /*< Latitude, expressed as degrees * 1E7 */
    int32_t longitude; /*< Longitude, expressed as degrees * 1E7 */
    int16_t altitude; /*< Altitude above mean sea level */
    int16_t target_altitude; /*< Altitude setpoint */
    uint8_t heading; /*< Heading (degrees / 2) */
    uint8_t target_heading; /*< Heading setpoint (degrees / 2) */
    uint16_t target_distance; /*< Distance to target waypoint or position (meters / 10) */
    uint8_t throttle; /*< Throttle (percentage) */
    uint8_t airspeed; /*< Airspeed (m/s * 5) */
    uint8_t airspeed_sp; /*< Airspeed setpoint (m/s * 5) */
    uint8_t groundspeed; /*< Groundspeed (m/s * 5) */
    uint8_t windspeed; /*< Windspeed (m/s * 5) */
    uint8_t wind_heading; /*< Wind heading (deg / 2) */
    uint8_t eph; /*< Maximum error horizontal position since last message (m * 10) */
    uint8_t epv; /*< Maximum error vertical position since last message (m * 10) */
    int8_t temperature_air; /*< Air temperature (degrees C) from airspeed sensor */
    int8_t climb_rate; /*< Maximum climb rate magnitude since last message (m/s * 10) */
    int8_t battery; /*< Battery (percentage, -1 for DNU) */
    uint16_t wp_num; /*< Current waypoint number */
    uint16_t failure_flags; /*< Indicates failures as defined in HL_FAILURE_FLAG ENUM.  */
    int8_t custom0; /*< Field for custom payload. */
    int8_t custom1; /*< Field for custom payload. */
    int8_t custom2; /*< Field for custom payload. */


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
        ss << "  type: " << +type << std::endl;
        ss << "  autopilot: " << +autopilot << std::endl;
        ss << "  custom_mode: " << custom_mode << std::endl;
        ss << "  latitude: " << latitude << std::endl;
        ss << "  longitude: " << longitude << std::endl;
        ss << "  altitude: " << altitude << std::endl;
        ss << "  target_altitude: " << target_altitude << std::endl;
        ss << "  heading: " << +heading << std::endl;
        ss << "  target_heading: " << +target_heading << std::endl;
        ss << "  target_distance: " << target_distance << std::endl;
        ss << "  throttle: " << +throttle << std::endl;
        ss << "  airspeed: " << +airspeed << std::endl;
        ss << "  airspeed_sp: " << +airspeed_sp << std::endl;
        ss << "  groundspeed: " << +groundspeed << std::endl;
        ss << "  windspeed: " << +windspeed << std::endl;
        ss << "  wind_heading: " << +wind_heading << std::endl;
        ss << "  eph: " << +eph << std::endl;
        ss << "  epv: " << +epv << std::endl;
        ss << "  temperature_air: " << +temperature_air << std::endl;
        ss << "  climb_rate: " << +climb_rate << std::endl;
        ss << "  battery: " << +battery << std::endl;
        ss << "  wp_num: " << wp_num << std::endl;
        ss << "  failure_flags: " << failure_flags << std::endl;
        ss << "  custom0: " << +custom0 << std::endl;
        ss << "  custom1: " << +custom1 << std::endl;
        ss << "  custom2: " << +custom2 << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp;                     // offset: 0
        map << latitude;                      // offset: 4
        map << longitude;                     // offset: 8
        map << custom_mode;                   // offset: 12
        map << altitude;                      // offset: 14
        map << target_altitude;               // offset: 16
        map << target_distance;               // offset: 18
        map << wp_num;                        // offset: 20
        map << failure_flags;                 // offset: 22
        map << type;                          // offset: 24
        map << autopilot;                     // offset: 25
        map << heading;                       // offset: 26
        map << target_heading;                // offset: 27
        map << throttle;                      // offset: 28
        map << airspeed;                      // offset: 29
        map << airspeed_sp;                   // offset: 30
        map << groundspeed;                   // offset: 31
        map << windspeed;                     // offset: 32
        map << wind_heading;                  // offset: 33
        map << eph;                           // offset: 34
        map << epv;                           // offset: 35
        map << temperature_air;               // offset: 36
        map << climb_rate;                    // offset: 37
        map << battery;                       // offset: 38
        map << custom0;                       // offset: 39
        map << custom1;                       // offset: 40
        map << custom2;                       // offset: 41
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp;                     // offset: 0
        map >> latitude;                      // offset: 4
        map >> longitude;                     // offset: 8
        map >> custom_mode;                   // offset: 12
        map >> altitude;                      // offset: 14
        map >> target_altitude;               // offset: 16
        map >> target_distance;               // offset: 18
        map >> wp_num;                        // offset: 20
        map >> failure_flags;                 // offset: 22
        map >> type;                          // offset: 24
        map >> autopilot;                     // offset: 25
        map >> heading;                       // offset: 26
        map >> target_heading;                // offset: 27
        map >> throttle;                      // offset: 28
        map >> airspeed;                      // offset: 29
        map >> airspeed_sp;                   // offset: 30
        map >> groundspeed;                   // offset: 31
        map >> windspeed;                     // offset: 32
        map >> wind_heading;                  // offset: 33
        map >> eph;                           // offset: 34
        map >> epv;                           // offset: 35
        map >> temperature_air;               // offset: 36
        map >> climb_rate;                    // offset: 37
        map >> battery;                       // offset: 38
        map >> custom0;                       // offset: 39
        map >> custom1;                       // offset: 40
        map >> custom2;                       // offset: 41
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
