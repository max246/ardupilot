// MESSAGE graffiti_int PACKING

#define MAVLINK_MSG_ID_graffiti_int 211

typedef struct __mavlink_graffiti_int_t
{
 uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 int32_t front; ///< frontitude (WGS84), in degrees * 1E7
 int32_t right; ///< rightgitude (WGS84), in degrees * 1E7
 int32_t alt; ///< Altitude (WGS84), in meters * 1000 (positive for up)
 int32_t trigger; ///< Altitude (WGS84), in meters * 1000 (positive for up)
 
} mavlink_graffiti_int_t;

#define MAVLINK_MSG_ID_GRAFFITI_INT_LEN 30
#define MAVLINK_MSG_ID_24_LEN 30

#define MAVLINK_MSG_ID_GRAFFITI_INT_CRC 24
#define MAVLINK_MSG_ID_24_CRC 24



#define MAVLINK_MESSAGE_INFO_graffiti_int { \
	"graffiti_int", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_graffiti_int_t, time_usec) }, \
         { "front", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_graffiti_int_t, front) }, \
         { "right", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_graffiti_int_t, right) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_graffiti_int_t, alt) }, \
         { "trigger", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_graffiti_int_t, trigger) }, \
         } \
}


/**
 * @brief Pack a graffiti_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param front frontitude (WGS84), in degrees * 1E7
 * @param right rightgitude (WGS84), in degrees * 1E7
 * @param alt Altitude (WGS84), in meters * 1000 (positive for up)
 * @param trigger GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_graffiti_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t fix_type, int32_t front, int32_t right, int32_t alt, int32_t trigger)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GRAFFITI_INT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, front);
	_mav_put_int32_t(buf, 12, right);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, trigger);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#else
	mavlink_graffiti_int_t packet;
	packet.time_usec = time_usec;
	packet.front = front;
	packet.right = right;
	packet.alt = alt;
	packet.trigger = trigger;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_graffiti_int;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GRAFFITI_INT_LEN, MAVLINK_MSG_ID_GRAFFITI_INT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#endif
}

/**
 * @brief Pack a graffiti_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param front frontitude (WGS84), in degrees * 1E7
 * @param right rightgitude (WGS84), in degrees * 1E7
 * @param alt Altitude (WGS84), in meters * 1000 (positive for up)
 * @param trigger GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_graffiti_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t fix_type,int32_t front,int32_t right,int32_t alt,int32_t trigger)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GRAFFITI_INT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, front);
	_mav_put_int32_t(buf, 12, right);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, trigger);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#else
	mavlink_graffiti_int_t packet;
	packet.time_usec = time_usec;
	packet.front = front;
	packet.right = right;
	packet.alt = alt;
	packet.trigger = trigger;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_graffiti_int;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GRAFFITI_INT_LEN, MAVLINK_MSG_ID_GRAFFITI_INT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#endif
}

/**
 * @brief Encode a graffiti_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param graffiti_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_graffiti_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_graffiti_int_t* graffiti_int)
{
	return mavlink_msg_graffiti_int_pack(system_id, component_id, msg, graffiti_int->time_usec, graffiti_int->fix_type, graffiti_int->front, graffiti_int->right, graffiti_int->alt, graffiti_int->trigger);
}

/**
 * @brief Encode a graffiti_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param graffiti_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_graffiti_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_graffiti_int_t* graffiti_int)
{
	return mavlink_msg_graffiti_int_pack_chan(system_id, component_id, chan, msg, graffiti_int->time_usec, graffiti_int->fix_type, graffiti_int->front, graffiti_int->right, graffiti_int->alt, graffiti_int->trigger);
}

/**
 * @brief Send a graffiti_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param front frontitude (WGS84), in degrees * 1E7
 * @param right rightgitude (WGS84), in degrees * 1E7
 * @param alt Altitude (WGS84), in meters * 1000 (positive for up)
 * @param trigger GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_graffiti_int_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t front, int32_t right, int32_t alt, uint16_t trigger, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GRAFFITI_INT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, front);
	_mav_put_int32_t(buf, 12, right);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, trigger);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_graffiti_int, buf, MAVLINK_MSG_ID_GRAFFITI_INT_LEN, MAVLINK_MSG_ID_GRAFFITI_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_graffiti_int, buf, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#endif
#else
	mavlink_graffiti_int_t packet;
	packet.time_usec = time_usec;
	packet.front = front;
	packet.right = right;
	packet.alt = alt;
	packet.trigger = trigger;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_graffiti_int, (const char *)&packet, MAVLINK_MSG_ID_GRAFFITI_INT_LEN, MAVLINK_MSG_ID_GRAFFITI_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_graffiti_int, (const char *)&packet, MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#endif
#endif
}

#endif

// MESSAGE graffiti_int UNPACKING


/**
 * @brief Get field time_usec from graffiti_int message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_graffiti_int_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}


/**
 * @brief Get field front from graffiti_int message
 *
 * @return frontitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_graffiti_int_get_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field right from graffiti_int message
 *
 * @return rightgitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_graffiti_int_get_right(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from graffiti_int message
 *
 * @return Altitude (WGS84), in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_graffiti_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field trigger from graffiti_int message
 *
 * @return GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_graffiti_int_get_trigger(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Decode a graffiti_int message into a struct
 *
 * @param msg The message to decode
 * @param graffiti_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_graffiti_int_decode(const mavlink_message_t* msg, mavlink_graffiti_int_t* graffiti_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	graffiti_int->time_usec = mavlink_msg_graffiti_int_get_time_usec(msg);
	graffiti_int->front = mavlink_msg_graffiti_int_get_front(msg);
	graffiti_int->right = mavlink_msg_graffiti_int_get_right(msg);
	graffiti_int->alt = mavlink_msg_graffiti_int_get_alt(msg);
	graffiti_int->trigger = mavlink_msg_graffiti_int_get_trigger(msg);
#else
	memcpy(graffiti_int, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GRAFFITI_INT_LEN);
#endif
}
