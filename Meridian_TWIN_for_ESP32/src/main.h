#ifndef __MERIDIAN_LOCAL_FUNC__
#define __MERIDIAN_LOCAL_FUNC__

#include <Arduino.h>

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

/**
 * @brief Convert a period-separated IP address string to an IPAddress object.
 *
 * @param ip_str const char*, period-separated IP address string.
 * @return IPAddress
 */
IPAddress makeIPAddress(const char *ip_str);

/**
 * @brief Get a Bluetooth pairing address.
 *
 * @param bda A pointer that points to the Bluetooth Device Address (BDA).
 *            The BDA is represented as a 6-byte array of bytes.
 * @param str A pointer to store the converted result string.
 * @param size The size of the str buffer.
 * @return char*
 */
char *bda2str(const uint8_t *bda, char *str, size_t size);

/**
 * @brief Pairing Bluetooth.
 *
 */
bool initBluetooth();

/**
 * @brief Receive input values from the PS4 remote control
 *        and store them in the following variables:
 *        pad_btn, pad_L2_val, pad_R2_val, pad_stick_L, pad_stick_R, pad_stick_V
 */
void pad_ps4_receive();

/**
 * @brief Receive input values from the wiimote
 *        and store them in pad_btn.
 */
void pad_wiimote_receive();

/**
 * @brief Send s_udp_meridim to UDP.
 *
 */
void udp_send();

/**
 * @brief Check Check received UDP packets.
 *  　　　　When a reception is complete, store the values in the union r_udp_meridim
 *  　　　　and set the flag_udp_rsvd flag to true.
 */
void udp_receive();

/**
 * @brief Setting for PS4 Bluetooth.
 *
 */
void bt_settings();

/**
 * @brief Thread for Bluetooth communication processing.
 *
 * @param args ?
 */
void Core0_BT_r(void *args);

#endif
