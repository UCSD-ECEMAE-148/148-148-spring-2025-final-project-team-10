#include "dw3000.h"
#include <Arduino.h>

// Constants
#define DEVICE_ID 1
#define OTHER_ANCHOR_ID 0
#define TAG_ID 2
#define FCS_LEN 2
#define MSG_LEN 20
#define TIMING_OFFSET_CONST 0.999987885
// ToF: 30540311 - Processing Time: 30540531.00 - Corrected ToF: -220.00

// Pin definitions
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

// Global variables
static int curr_stage = 0;
uint8_t msg_index = 0;
static uint32_t status_reg = 0;
static int message_timeout = 1000; // in ms, timeout for message reception

// Buffers
static uint8_t rx_buffer[MSG_LEN];
uint32_t poll_tx_ts, resp_rx_ts;
uint8_t rx_buffer_tof_cycles[MSG_LEN];

// Variables for TWR-SS
static uint64_t tof, tag_processing_time;
static double corrected_processing_time, corrected_tof, distance;

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
#define CLEARANCE_MESSAGE 3

static uint8_t tx_ping_msg[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
// 0: Waiting for message reception - This contains clearance message from Anchor 1
// 1: Transmitting PING message for distance measurement
// 2: Waiting for message reception - PONG message
// 3: Waiting for message reception - PONG with processing time messages
// 4: Send clear message to Anchor 1 to start its distance measurement

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

          if (rx_buffer[SENDER_ID_IDX] == OTHER_ANCHOR_ID && rx_buffer[RECEIVER_ID_IDX] == DEVICE_ID && rx_buffer[MESSAGE_TYPE_IDX] == CLEARANCE_MESSAGE) {
            // Serial.println("Clearance message received!");
            curr_stage = 1;
          }
          else {
            // If the message was not meant for this anchor, clear the good frame received flag
            // Serial.println("Message not meant for this tag!");
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            curr_stage = 0;
          }
        }
      }
      break;
    }
    case 1: {
      // Transmitting a PONG message
      tx_ping_msg[SENDER_ID_IDX] = DEVICE_ID;
      tx_ping_msg[RECEIVER_ID_IDX] = TAG_ID;
      tx_ping_msg[MESSAGE_NUMBER_IDX] = msg_index + 10;
      tx_ping_msg[MESSAGE_TYPE_IDX] = PING_MESSAGE;

      // Clear Tx status flag
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      
      // Transmitting the data and correcting for buffer offset
      dwt_writetxdata(sizeof(tx_ping_msg), tx_ping_msg, 0);
      dwt_writetxfctrl(sizeof(tx_ping_msg), 0, 1);
      
      // Immediately start transmission along with response flag expected
      status_reg = dwt_starttx(DWT_START_TX_IMMEDIATE);
      
      // Waiting for transmission to complete
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        // Do nothing
      }
      if (status_reg == DWT_SUCCESS) {
        // Serial.println("PING Transmission Successful");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
      else {
        // Serial.println("PING Transmission Failed");
        curr_stage = 0;
        break;
      }

      // Clearing the Tx status flag
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      
      msg_index += 1;
      curr_stage = 2;
      break;
    }
    case 2: {
      // Waiting for message reception
      uint32_t start_time = millis();
      // Enable message reception
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        if (millis() - start_time > message_timeout) {
          curr_stage = 1;
          break;
        }
      }

      if (curr_stage == 1) {
        Serial.println("Message timeout, send another ping message");
        ESP.restart();
        break;
      }

      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        // Compute frame length for received frame
        uint32_t frame_len;

        // Clear the good frame received flag
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        
        // Reading the frame
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
          // Reading the frame from Rx buffer
          dwt_readrxdata(rx_buffer, frame_len, 0);

          // If the message was sent by the tag and meant for PONG message
          if (rx_buffer[SENDER_ID_IDX] == TAG_ID && rx_buffer[RECEIVER_ID_IDX] == DEVICE_ID && rx_buffer[MESSAGE_TYPE_IDX] == PONG_MESSAGE) {
            // Record the transmit and receive timestamps for TWR-SS
            poll_tx_ts = get_time_tx();
            resp_rx_ts = get_time_rx();
            tof = resp_rx_ts - poll_tx_ts;
            curr_stage = 3;
          }
          else {
            // If the message was not meant for this anchor, clear the good frame received flag
            Serial.println("There is an error in the protocol!");
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            curr_stage = 4;
            break;
          }
          // Start reception of the next message
          dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
      }
      else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        Serial.println("Bad frame received");
        curr_stage = 4;
        break;
      }
      break;
    }
    case 3: {
      // Waiting for message reception
      uint32_t start_time = millis();
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        if (millis() - start_time > message_timeout) {
          curr_stage = 4;
          break;
        }
      }

      if (curr_stage == 4) {
        Serial.println("Message timeout, send another ping message");
        ESP.restart();
        break;
      }

      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        // Compute frame length for received frame
        uint32_t frame_len;

        // Clear the good frame received flag
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        
        // Reading the frame
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
          // Reading the frame from Rx buffer
          dwt_readrxdata(rx_buffer, frame_len, 0);

          // If the message was sent by the tag and meant for PONG with processing time message
          if (rx_buffer[SENDER_ID_IDX] == TAG_ID && rx_buffer[RECEIVER_ID_IDX] == DEVICE_ID && rx_buffer[MESSAGE_TYPE_IDX] == PONG_WITH_PROCESSING_TIME_MESSAGE) {
            // Storing the processing time of the tag
            tag_processing_time = 0;
            for (int i = 0; i < 8; i++) {
              tag_processing_time |= ((uint64_t)rx_buffer[START_PROCESSING_TIME_IDX + i]) << (56 - 8 * i);
            }

            corrected_processing_time = tag_processing_time * TIMING_OFFSET_CONST;
            // corrected_processing_time = tag_processing_time;
            corrected_tof = tof - corrected_processing_time;
            
            // Serial.print("ToF: ");
            // Serial.print(tof);
            // Serial.print(" - Processing Time: ");
            // Serial.print(corrected_processing_time);
            // Serial.print(" - Corrected ToF: ");
            // Serial.println(corrected_tof);

            msg_index += 1;
            curr_stage = 4;
            // Distance in cm
            distance = 100 * corrected_tof * DWT_TIME_UNITS * SPEED_OF_LIGHT / 2;
            Serial.println(distance);
          }
          else {
            // If the message was not meant for this anchor, clear the good frame received flag
            Serial.println("There is an error in the protocol!");
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            curr_stage = 4;
            break;
          }
        }
      }
      else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        Serial.println("Bad frame received");
        curr_stage = 4;
        break;
      }
      break;
    }
    case 4: {
      // Transmitting clearance message
      delay(1);
      tx_ping_msg[SENDER_ID_IDX] = DEVICE_ID;
      tx_ping_msg[RECEIVER_ID_IDX] = OTHER_ANCHOR_ID;
      tx_ping_msg[MESSAGE_TYPE_IDX] = CLEARANCE_MESSAGE;

      // Clear Tx status flag
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

      // Transmitting the data and correcting for buffer offset
      dwt_writetxdata(sizeof(tx_ping_msg), tx_ping_msg, 0);
      dwt_writetxfctrl(sizeof(tx_ping_msg), 0, 1);
      
      // Immediately start transmission along with response flag expected
      dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
      // Serial.println("Waiting for transmission");

      /// Waiting for transmission to complete
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        // Do nothing
      }
      // Serial.println("Clearance message transmitted!");
      curr_stage = 0;
      // // Print the average of the cumulative tof and tag processing time
      // double average_tof = cumulative_tof / 9;
      // double average_tag_processing_time = cumulative_tag_processing_time / 9;
      // Serial.print("Average ToF: ");
      // Serial.print(average_tof);
      // Serial.print(" - Average Tag Processing Time: ");
      // Serial.print(average_tag_processing_time);
      // Serial.print(" - Average Adjusted ToF: ");
      // Serial.println(average_tof - average_tag_processing_time);
      break;
    }
  }
}
