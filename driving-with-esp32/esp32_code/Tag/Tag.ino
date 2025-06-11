#include "dw3000.h"

// Constants
#define DEVICE_ID 2
#define FCS_LEN 2
#define MSG_LEN 20

// Pin definitions
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

// Global variables
uint8_t msg_index = 0;
uint8_t sender_id = 0;
static uint8_t curr_stage = 0;
static uint32_t status_reg = 0;

// Buffers
static uint8_t rx_buffer[MSG_LEN];
uint32_t poll_tx_ts, resp_rx_ts;
uint64_t tof_cycles;

// Constants
const double c_speed = 299702547.0; // in meters per second
const double DTU_TO_S = 1 / (128 * 499.2 * 1e6);  // 15.65 ps per DTU = 0.01565 ns

// Configuration mode for DWT
static dwt_config_t config = {
        5,               /* Channel number. */
        DWT_PLEN_128,    /* Preamble length. Used in TX only. */
        DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
        DWT_BR_6M8,      /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        DWT_PHRRATE_STD, /* PHY header rate. */
        (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        DWT_STS_MODE_OFF, /* STS disabled */
        DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
        DWT_PDOA_M0      /* PDOA mode off */
};

// Tx configurations
extern dwt_txconfig_t txconfig_options;

// Standard Tx and Rx messages that will be communicated
// Description:
// 0: Sender ID (0 - 1st Anchor, 1 - 2nd Anchor, 2 - Tag)
// 1: Receiver ID (0 - 1st Anchor, 1 - 2nd Anchor, 2 - Tag)
// 2: Message Number (0 - 1st Message, 1 - 2nd Message, 2 - 3rd Message, etc.)
// 3: Message Type (0 - Ping, 1 - Pong, 2 - Pong with processing time)
// 4: Processing Time - 1 byte
// 5: Processing Time - 1 byte
// 6: Processing Time - 1 byte
// 7: Processing Time - 1 byte
// 8: Processing Time - 1 byte
// 9: Processing Time - 1 byte
// 10: Processing Time - 1 byte
// 11: Processing Time - 1 byte
// Constants to access the message fields
#define SENDER_ID_IDX 0
#define RECEIVER_ID_IDX 1
#define MESSAGE_NUMBER_IDX 2
#define MESSAGE_TYPE_IDX 3
#define START_PROCESSING_TIME_IDX 4
// Constants for message types
#define PING_MESSAGE 0
#define PONG_MESSAGE 1
#define PONG_WITH_PROCESSING_TIME_MESSAGE 2

static uint8_t tx_pong_msg[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// static uint8_t rx_pong_msg[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint64_t get_time_tx() {
    uint32_t hi32 = dwt_readtxtimestamphi32(); // bits [8..39], stored in bits [0..31] here
    uint32_t lo32 = dwt_readtxtimestamplo32(); // bits [0..31]

    // Extract the low 8 bits from lo32
    uint8_t low8 = (uint8_t)(lo32 & 0xFF);

    // Shift hi32 left by 8 bits to make space for the low 8 bits
    uint64_t timestamp = ((uint64_t)hi32 << 8) | low8;

    return timestamp;  // Full 40-bit RX timestamp
}

uint64_t get_time_rx() {
    uint32_t hi32 = dwt_readrxtimestamphi32(); // bits [8..39], stored in bits [0..31] here
    uint32_t lo32 = dwt_readrxtimestamplo32(); // bits [0..31]

    // Extract the low 8 bits from lo32
    uint8_t low8 = (uint8_t)(lo32 & 0xFF);

    // Shift hi32 left by 8 bits to make space for the low 8 bits
    uint64_t timestamp = ((uint64_t)hi32 << 8) | low8;

    return timestamp;  // Full 40-bit RX timestamp
}

void setup() {
  // Initialize serial
  UART_init();
  Serial.println("Setting up the anchor board for TWR-SS");

  // Set pins
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2);

  // Check if its idle or not
  while (!dwt_checkidlerc()) 
  {
    UART_puts("IDLE FAILED\r\n");
    while (1) ;
  }
  dwt_softreset();

  // Initialize the DWT board
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1) ;
  }

  // Enabling LEDs here
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Load the configuration into DWT
  if(dwt_configure(&config)){
    UART_puts("CONFIG FAILED\r\n");
    while (1) ;
  }

  // Applying the default Tx configuration options
  dwt_configuretxrf(&txconfig_options);

  // // Setting the delay values for Tx and Rx
  // dwt_setrxantennadelay(RX_ANT_DLY);
  // dwt_settxantennadelay(TX_ANT_DLY);

  // This is to set the transmit and receive power for better ranging
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

// Description of the stages:
// 0: Reception of a PING message
// 1: Transmitting a PONG message
// 2: Transmitting a PONG with processing time message

void loop() {
  switch(curr_stage) {
    case 0: {
      // Activate message retrieval immediately
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      
      // Clear the transmitted flag
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        // polling
      }

      // If a good frame is received
      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        // Compute frame length for received frame
        uint32_t frame_len;

        // Clear the good frame received flag
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        // Reading the frame
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
          // Putting the frame into buffer
          dwt_readrxdata(rx_buffer, frame_len, 0);

          if (rx_buffer[RECEIVER_ID_IDX] == DEVICE_ID) {
            if (rx_buffer[MESSAGE_TYPE_IDX] == PING_MESSAGE) {
              sender_id = rx_buffer[SENDER_ID_IDX];
              msg_index = rx_buffer[MESSAGE_NUMBER_IDX];
              curr_stage = 1;
            }
          }
          else {
            // If the message was not meant for this anchor, clear the good frame received flag
            Serial.println("Message not meant for this tag!");
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            curr_stage = 0;
          }
        }
      }
      break;
    }
    case 1: {
      // Transmitting a PONG message
      tx_pong_msg[SENDER_ID_IDX] = DEVICE_ID;
      tx_pong_msg[RECEIVER_ID_IDX] = sender_id;
      tx_pong_msg[MESSAGE_NUMBER_IDX] = msg_index + 10;
      tx_pong_msg[MESSAGE_TYPE_IDX] = PONG_MESSAGE;

      // Clear Tx status flag
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      
      // Transmitting the data and correcting for buffer offset
      dwt_writetxdata(sizeof(tx_pong_msg), tx_pong_msg, 0);
      dwt_writetxfctrl(sizeof(tx_pong_msg), 0, 1);
      
      // Immediately start transmission along with response flag expected
      status_reg = dwt_starttx(DWT_START_TX_IMMEDIATE);
        
      // Waiting for transmission to complete
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        // Do nothing
      }
      if (status_reg == DWT_SUCCESS) {
        Serial.println("PONG Transmission Successful");
      }
      else {
        Serial.println("PONG Transmission Failed");
        curr_stage = 0;
        break;
      }

      // Clearing the Tx status flag
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      tof_cycles = get_time_tx() - get_time_rx();
      Serial.println(tof_cycles);
      
      curr_stage = 2;
      break;
    }
    case 2: {
      // Transmitting a PONG with processing time message
      tx_pong_msg[MESSAGE_NUMBER_IDX] = msg_index + 20;
      tx_pong_msg[MESSAGE_TYPE_IDX] = PONG_WITH_PROCESSING_TIME_MESSAGE;
      for (int i = 0; i < 8; i++) {
        tx_pong_msg[START_PROCESSING_TIME_IDX + i] = tof_cycles >> (56 - 8 * i);
      }
      
      // Transmit the message
      dwt_writetxdata(sizeof(tx_pong_msg), tx_pong_msg, 0);
      dwt_writetxfctrl(sizeof(tx_pong_msg), 0, 1);
      
      // Delay to ensure the message is transmitted
      delay(1);
      // Serial.println("Starting data transmission");
      status_reg = dwt_starttx(DWT_START_TX_IMMEDIATE);
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        // Do nothing
      }
      if (status_reg == DWT_SUCCESS) {
        Serial.println("PONG with Processing Time Transmission Successful");
      }
      else {
        Serial.println("PONG with Processing Time Transmission Failed");
        curr_stage = 0;
        break;
      }
      // Clearing the Tx status flag
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      curr_stage = 0;
      break;
    }
  }
}
