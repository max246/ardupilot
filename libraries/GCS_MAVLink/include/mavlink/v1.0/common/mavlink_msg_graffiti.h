// MESSAGE GRAFFITI PACKING

#define MAVLINK_MSG_ID_GRAFFITI 211

typedef struct __mavlink_graffiti_t
{
 uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 int32_t front; ///< frontitude (WGS84), in degrees * 1E7
 int32_t right; ///< rightgitude (WGS84), in degrees * 1E7
 int32_t alt; ///< Altitude (WGS84), in meters * 1000 (positive for up)
 int32_t trigger; ///< Altitude (WGS84), in meters * 1000 (positive for up)
 
} mavlink_graffiti_t;

#define MAVLINK_MSG_ID_GRAFFITI_LEN 22
#define MAVLINK_MSG_ID_211_LEN 22

#define MAVLINK_MSG_ID_GRAFFITI_CRC 168
#define MAVLINK_MSG_ID_211_CRC 168



#define MAVLINK_MESSAGE_INFO_GRAFFITI { \
	"GRAFFITI", \
	5, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_graffiti_t, time_usec) }, \
         { "front", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_graffiti_t, front) }, \
         { "right", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_graffiti_t, right) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_graffiti_t, alt) }, \
         { "trigger", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_graffiti_t, trigger) }, \
         } \
}


/**
 * @brief Pack a GRAFFITI message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for
 IMU)
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
static inline uint16_t mavlink_msg_graffiti_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, int32_t front, int32_t right, int32_t alt, int32_t trigger)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GRAFFITI_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, front);
	_mav_put_int32_t(buf, 12, right);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, trigger);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GRAFFITI_LEN);
#else
	mavlink_graffiti_t packet;
	packet.time_usec = time_usec;
	packet.front = front;
	packet.right = right;
	packet.alt = alt;
	packet.trigger = trigger;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GRAFFITI_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GRAFFITI;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GRAFFITI_LEN, MAVLINK_MSG_ID_GRAFFITI_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GRAFFITI_LEN);
#endif
}

/**
 * @brief Pack a GRAFFITI message on a channel
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
static inline uint16_t mavlink_msg_graffiti_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,int32_t front,int32_t right,int32_t alt,int32_t trigger)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GRAFFITI_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, front);
	_mav_put_int32_t(buf, 12, right);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf,20, trigger);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GRAFFITI_LEN);
#else
	mavlink_graffiti_t packet;
	packet.time_usec = time_usec;
	packet.front = front;
	packet.right = right;
	packet.alt = alt;
	packet.trigger = trigger;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GRAFFITI_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GRAFFITI;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GRAFFITI_LEN, MAVLINK_MSG_ID_GRAFFITI_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GRAFFITI_LEN);
#endif
}

/**
 * @brief Encode a GRAFFITI struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param GRAFFITI C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_graffiti_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_graffiti_t* GRAFFITI)
{
	return mavlink_msg_graffiti_pack(system_id, component_id, msg, GRAFFITI->time_usec, GRAFFITI->front, GRAFFITI->right, GRAFFITI->alt, GRAFFITI->trigger);
}

/**
 * @brief Encode a GRAFFITI struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param GRAFFITI C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_graffiti_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_graffiti_t* GRAFFITI)
{
	return mavlink_msg_graffiti_pack_chan(system_id, component_id, chan, msg, GRAFFITI->time_usec,  GRAFFITI->front, GRAFFITI->right, GRAFFITI->alt, GRAFFITI->trigger);
}

/**
 * @brief Send a GRAFFITI message
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

static inline void mavlink_msg_graffiti_send(mavlink_channel_t chan, uint64_t time_usec,  int32_t front, int32_t right, int32_t alt, uint16_t trigger)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GRAFFITI_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, front);
	_mav_put_int32_t(buf, 12, right);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, trigger);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRAFFITI, buf, MAVLINK_MSG_ID_GRAFFITI_LEN, MAVLINK_MSG_ID_GRAFFITI_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRAFFITI, buf, MAVLINK_MSG_ID_GRAFFITI_LEN);
#endif
#else
	mavlink_graffiti_t packet;
	packet.time_usec = time_usec;
	packet.front = front;
	packet.right = right;
	packet.alt = alt;
	packet.trigger = trigger;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRAFFITI, (const char *)&packet, MAVLINK_MSG_ID_GRAFFITI_LEN, MAVLINK_MSG_ID_GRAFFITI_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRAFFITI, (const char *)&packet, MAVLINK_MSG_ID_GRAFFITI_LEN);
#endif
#endif
}

#endif

// MESSAGE GRAFFITI UNPACKING


/**
 * @brief Get field time_usec from GRAFFITI message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_graffiti_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}


/**
 * @brief Get field front from GRAFFITI message
 *
 * @return frontitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_graffiti_get_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field right from GRAFFITI message
 *
 * @return rightgitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_graffiti_get_right(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from GRAFFITI message
 *
 * @return Altitude (WGS84), in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_graffiti_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field trigger from GRAFFITI message
 *
 * @return GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_graffiti_get_trigger(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Decode a GRAFFITI message into a struct
 *
 * @param msg The message to decode
 * @param GRAFFITI C-struct to decode the message contents into
 */
static inline void mavlink_msg_graffiti_decode(const mavlink_message_t* msg, mavlink_graffiti_t* GRAFFITI)
{
#if MAVLINK_NEED_BYTE_SWAP
	GRAFFITI->time_usec = mavlink_msg_graffiti_get_time_usec(msg);
	GRAFFITI->front = mavlink_msg_graffiti_get_front(msg);
	GRAFFITI->right = mavlink_msg_graffiti_get_right(msg);
	GRAFFITI->alt = mavlink_msg_graffiti_get_alt(msg);
	GRAFFITI->trigger = mavlink_msg_graffiti_get_trigger(msg);
#else
	memcpy(GRAFFITI, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GRAFFITI_LEN);
#endif
}
