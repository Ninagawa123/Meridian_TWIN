#include "ps4.h"

#include <esp_system.h>
#include <string.h>

#include "ps4_int.h"

/********************************************************************************/
/*                              C O N S T A N T S */
/********************************************************************************/

static const uint8_t hid_cmd_payload_ps4_enable[] = {0x43, 0x02};

/********************************************************************************/
/*                         L O C A L    V A R I A B L E S */
/********************************************************************************/

static ps4_connection_callback_t ps4_connection_cb = NULL;
static ps4_connection_object_callback_t ps4_connection_object_cb = NULL;
static void* ps4_connection_object = NULL;

static ps4_event_callback_t ps4_event_cb = NULL;
static ps4_event_object_callback_t ps4_event_object_cb = NULL;
static void* ps4_event_object = NULL;

static bool is_active = false;

/********************************************************************************/
/*                      P U B L I C    F U N C T I O N S */
/********************************************************************************/

/*******************************************************************************
**
** Function         ps4Init
**
** Description      This initializes the bluetooth services to listen
**                  for an incoming PS4 controller connection.
**
**
** Returns          void
**
*******************************************************************************/
void ps4Init() {
  sppInit();
  ps4_l2cap_init_services();
}

/*******************************************************************************
**
** Function         ps4IsConnected
**
** Description      This returns whether a PS4 controller is connected, based
**                  on whether a successful handshake has taken place.
**
**
** Returns          bool
**
*******************************************************************************/
bool ps4IsConnected() { return is_active; }

/*******************************************************************************
**
** Function         ps4Enable
**
** Description      This triggers the PS4 controller to start continually
**                  sending its data.
**
**
** Returns          void
**
*******************************************************************************/
void ps4Enable() {
  uint16_t length = sizeof(hid_cmd_payload_ps4_enable);
  hid_cmd_t hidCommand;

  hidCommand.code = hid_cmd_code_set_report | hid_cmd_code_type_feature;
  hidCommand.identifier = hid_cmd_identifier_ps4_enable;

  memcpy(hidCommand.data, hid_cmd_payload_ps4_enable, length);

  ps4_l2cap_send_hid(&hidCommand, length);
  ps4SetLed(32, 32, 200);
}

/*******************************************************************************
**
** Function         ps4Cmd
**
** Description      Send a command to the PS4 controller.
**
**
** Returns          void
**
*******************************************************************************/
void ps4Cmd(ps4_cmd_t cmd) {
  hid_cmd_t hidCommand = {.data = {0x80, 0x00, 0xFF}};
  uint16_t length = sizeof(hidCommand.data);

  hidCommand.code = hid_cmd_code_set_report | hid_cmd_code_type_output;
  hidCommand.identifier = hid_cmd_identifier_ps4_control;

  hidCommand.data[ps4_control_packet_index_small_rumble] = cmd.smallRumble;  // Small Rumble
  hidCommand.data[ps4_control_packet_index_large_rumble] = cmd.largeRumble;  // Big rumble

  hidCommand.data[ps4_control_packet_index_red] = cmd.r;    // Red
  hidCommand.data[ps4_control_packet_index_green] = cmd.g;  // Green
  hidCommand.data[ps4_control_packet_index_blue] = cmd.b;   // Blue

  // Time to flash bright (255 = 2.5 seconds)
  hidCommand.data[ps4_control_packet_index_flash_on_time] = cmd.flashOn;
  // Time to flash dark (255 = 2.5 seconds)
  hidCommand.data[ps4_control_packet_index_flash_off_time] = cmd.flashOff;

  ps4_l2cap_send_hid(&hidCommand, length);
}

/*******************************************************************************
**
** Function         ps4SetLedOnly
**
** Description      Sets the LEDs on the PS4 controller.
**
**
** Returns          void
**
*******************************************************************************/
void ps4SetLed(uint8_t r, uint8_t g, uint8_t b) {
  ps4_cmd_t cmd = {0};

  cmd.r = r;
  cmd.g = g;
  cmd.b = b;

  ps4Cmd(cmd);
}

/*******************************************************************************
**
** Function         ps4SetOutput
**
** Description      Sets feedback on the PS4 controller.
**
**
** Returns          void
**
*******************************************************************************/
void ps4SetOutput(ps4_cmd_t prevCommand) { ps4Cmd(prevCommand); }

/*******************************************************************************
**
** Function         ps4SetConnectionCallback
**
** Description      Registers a callback for receiving PS4 controller
**                  connection notifications
**
**
** Returns          void
**
*******************************************************************************/
void ps4SetConnectionCallback(ps4_connection_callback_t cb) {
  ps4_connection_cb = cb;
}

/*******************************************************************************
**
** Function         ps4SetConnectionObjectCallback
**
** Description      Registers a callback for receiving PS4 controller
**                  connection notifications
**
**
** Returns          void
**
*******************************************************************************/
void ps4SetConnectionObjectCallback(void* object, ps4_connection_object_callback_t cb) {
  ps4_connection_object_cb = cb;
  ps4_connection_object = object;
}

/*******************************************************************************
**
** Function         ps4SetEventCallback
**
** Description      Registers a callback for receiving PS4 controller events
**
**
** Returns          void
**
*******************************************************************************/
void ps4SetEventCallback(ps4_event_callback_t cb) { ps4_event_cb = cb; }

/*******************************************************************************
**
** Function         ps4SetEventObjectCallback
**
** Description      Registers a callback for receiving PS4 controller events
**
**
** Returns          void
**
*******************************************************************************/
void ps4SetEventObjectCallback(void* object, ps4_event_object_callback_t cb) {
  ps4_event_object_cb = cb;
  ps4_event_object = object;
}

/*******************************************************************************
**
** Function         ps4SetBluetoothMacAddress
**
** Description      Writes a Registers a callback for receiving PS4 controller
*events
**
**
** Returns          void
**
*******************************************************************************/
void ps4SetBluetoothMacAddress(const uint8_t* mac) {
  // The bluetooth MAC address is derived from the base MAC address
  // https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/system/system.html#mac-address
  uint8_t baseMac[6];
  memcpy(baseMac, mac, 6);
  baseMac[5] -= 2;
  esp_base_mac_addr_set(baseMac);
}

/********************************************************************************/
/*                      L O C A L    F U N C T I O N S */
/********************************************************************************/

void ps4ConnectEvent(uint8_t is_connected) {
    if (is_connected) {
        ps4Enable();
    } else {
        is_active = false;
    }
}


void ps4PacketEvent(ps4_t ps4, ps4_event_t event) {
    // Trigger packet event, but if this is the very first packet
    // after connecting, trigger a connection event instead
    if (is_active) {
        if(ps4_event_cb != NULL) {
            ps4_event_cb(ps4, event);
        }

        if (ps4_event_object_cb != NULL && ps4_event_object != NULL) {
            ps4_event_object_cb(ps4_event_object, ps4, event);
        }
    } else {
        is_active = true;

        if(ps4_connection_cb != NULL) {
            ps4_connection_cb(is_active);
        }

        if (ps4_connection_object_cb != NULL && ps4_connection_object != NULL) {
            ps4_connection_object_cb(ps4_connection_object, is_active);
        }
    }
}
