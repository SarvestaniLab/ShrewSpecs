/*
Giga_M7_BNO055.ino – Combined Video + IMU Streamer for Arduino Giga R1
- Reads 128×240 8-bit video from M4 core
- Downscales 3:1 → 128×80, then crops to 80×80
- Reads BNO055 IMU (quaternion + linear acceleration) over I2C (if one is connected)
- Sends video and IMU packets over USB Serial at 5 Mbaud
*/

#include <algorithm>
#include <arduino.h>
#include <Wire.h>
#include <RPC.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// ----- Hardcoded user settings -----
const uint8_t camera_marker = 0x01;             // ************** camera identifier for this board (0x01 for cam1, 0x02 for cam2) **************
const bool debugging = false;                   // enable debug text messages
const bool stream_checkerboard = false;         // send synthetic checkerboard instead of real video (for testing)
const bool do_advanced_processing = false;      // back porch correction, line registration (not implemented)
const uint16_t num_frames_to_skip = 0;          // drop N frames between sent images
String version_name = "0.11";                   // version

// ----- SRAM shared with M4 core for image transfer -----
const uint32_t SDRAM_START_ADDRESS_4 = 0x38008000; //shifted over to avoid overlap with RPC memory areas? Or whatever, it works
volatile uint32_t *sdramMemory = (uint32_t*)0x38008000;

// Shared memory structure (30,736 bytes)
struct DATA {
  uint32_t timestamp;       // Byte 0-3: frame timestamp
  uint16_t frame_num;       // Byte 4-5: frame counter
  uint16_t captured_lines;  // Byte 6-7: total lines captured in this frame
  uint8_t image[30720];     // Byte 8-30727: raw 128x240 byte image
  uint8_t ready;            // Byte 30728: flag for image
  uint8_t m4_status;        // Byte 30729: state of M4 (0=no ADC, 1=no signal, 2=all working)
  uint8_t reserved_pad[6];  // Byte 30730-30735: padding for 8-byte alignment
} __attribute__((aligned(8)));
DATA* shared_data = (DATA*)(SDRAM_START_ADDRESS_4);

// ----- Image data -----
const int frame_w = 128;          // frame: raw image captured by M4 core
const int frame_h = 240;
const int small_frame_w = 128;    // small_frame: image after 3-row averaging
const int small_frame_h = 80;      
const int crop_frame_w = 80;       // crop_frame: cropped image to stream
const int crop_frame_h = 80;

uint8_t crop_x = 24;     // default centered crop x (128-80)/2 = 24
uint8_t crop_y = 0;      // crop y only used for sending smaller frames

const int frame_bytes = frame_w * frame_h;
const int small_frame_bytes = small_frame_w * small_frame_h;
const int crop_frame_bytes = crop_frame_w * crop_frame_h;

uint8_t frame[frame_bytes];
uint8_t small_frame[small_frame_bytes];
uint8_t crop_frame[crop_frame_bytes];

const uint16_t video_payload_bytes = 1 + 2 + 4 + crop_frame_bytes; // cam_id + frame# + timestamp + image
uint16_t num_frames = 0;
uint16_t skipped_frames = 0;
const unsigned long min_frame_interval = 11; // caps at 90 fps (11.11 ms)

// ----- Streaming control flags -----
bool stream_frame = false;
bool stream_imu = false;
bool stream_tracking = false;

// ----- IMU data (BNO055) -----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float imuData[7] = {0};                  // [qw, qx, qy, qz, ax, ay, az]
uint16_t num_imu_samples = 0;
const uint16_t imu_payload_bytes = 2 + 4 + 28; // sample# + timestamp + 7 floats
const unsigned long imu_interval = 17;   // for 60 Hz streaming (16.67 ms)
bool BNO_connected = false;

// ----- Serial protocol markers -----
const uint8_t packet_start_marker = 0xFE;
const uint8_t imu_marker = 0x03;
const uint8_t settings_marker = 0x04;
const uint8_t frame_marker = 0x05;
const uint8_t text_marker = 0x06;

// ----- Streaming commands -----
const uint8_t CMD_STATUS = 0x20;          // request status byte
const uint8_t CMD_START_FRAME = 0x21;      // start video streaming
const uint8_t CMD_START_IMU = 0x22;        // start IMU streaming (if connected)
const uint8_t CMD_START_TRACKING = 0x23;   // start real-time eye‑tracking (not implemented)
const uint8_t CMD_STOP_ALL = 0x11;         // stop all streaming

// ----- Timing variables -----
unsigned long current_loop_time = 0;
unsigned long last_frame_sent_time = 0;
unsigned long last_imu_time = 0;
unsigned long last_command_time = 0;


// ============================================================================
// Helper functions (checksum, packet sending, debug)
// ============================================================================
uint8_t updateChecksum(uint8_t sum, const uint8_t* data, size_t length) {
  for (size_t idx = 0; idx < length; idx++) sum += data[idx];
  return sum;
}

void sendPacket(uint8_t type, const uint8_t* payload, uint16_t payload_len) {
  uint8_t header[3] = { type, uint8_t(payload_len & 0xFF), uint8_t(payload_len >> 8) };
  Serial.write(packet_start_marker);
  Serial.write(header, 3);
  uint8_t checksum = updateChecksum(0, header, 3);
  if (payload_len) {
    Serial.write(payload, payload_len);
    checksum = updateChecksum(checksum, payload, payload_len);
  }
  Serial.write(checksum);
}

void sendVideoFrame(uint16_t frame_num, uint32_t timestamp, uint8_t* image_data, uint8_t camera_id) {
  uint8_t header[3] = { frame_marker, uint8_t(video_payload_bytes & 0xFF), uint8_t(video_payload_bytes >> 8) };
  Serial.write(packet_start_marker);
  Serial.write(header, 3);
  uint8_t checksum = updateChecksum(0, header, 3);

  Serial.write(&camera_id, 1);
  checksum += camera_id;

  Serial.write((uint8_t*)&frame_num, 2);
  checksum = updateChecksum(checksum, (uint8_t*)&frame_num, 2);

  Serial.write((uint8_t*)&timestamp, 4);
  checksum = updateChecksum(checksum, (uint8_t*)&timestamp, 4);

  Serial.write(image_data, crop_frame_bytes);
  checksum = updateChecksum(checksum, image_data, crop_frame_bytes);

  Serial.write(checksum);
}

void sendIMURecord(uint16_t sample_num, uint32_t timestamp) {
  uint8_t header[3] = { imu_marker, uint8_t(imu_payload_bytes & 0xFF), uint8_t(imu_payload_bytes >> 8) };
  Serial.write(packet_start_marker);
  Serial.write(header, 3);
  uint8_t checksum = updateChecksum(0, header, 3);

  Serial.write((uint8_t*)&sample_num, 2);
  checksum = updateChecksum(checksum, (uint8_t*)&sample_num, 2);

  Serial.write((uint8_t*)&timestamp, 4);
  checksum = updateChecksum(checksum, (uint8_t*)&timestamp, 4);

  Serial.write((uint8_t*)&imuData, 28);
  checksum = updateChecksum(checksum, (uint8_t*)&imuData, 28);

  Serial.write(checksum);
}

void sendTextMessage(const char* text) {
  sendPacket(text_marker, (const uint8_t*)text, strlen(text));
}

void debugPrintln(const String &msg) {
  sendTextMessage(msg.c_str());
}

void fillCheckerboard(uint8_t* buffer) {
  for (int row = 0; row < crop_frame_h; row++) {
    bool top_half = row < (crop_frame_h / 2);
    for (int col = 0; col < crop_frame_w; col++) {
      bool left_half = col < (crop_frame_w / 2);
      uint8_t val;
      if (top_half)      val = left_half ? 0   : 170;
      else               val = left_half ? 255 : 85;
      buffer[row * crop_frame_w + col] = val;
    }
  }
}

bool validateIMUData() {
  float q_norm_sq = imuData[0]*imuData[0] + imuData[1]*imuData[1] +
                    imuData[2]*imuData[2] + imuData[3]*imuData[3];
  if (q_norm_sq < 0.5f || q_norm_sq > 2.0f) {
    if (debugging) {
      debugPrintln("IMU invalid: quaternion norm");
    }
    return false;
  }
  if (fabs(imuData[4]) > 50.0f || fabs(imuData[5]) > 50.0f || fabs(imuData[6]) > 50.0f) {
    if (debugging) {
      debugPrintln("IMU invalid: acceleration out of range");
    }
    return false;
  }
  for (int i = 0; i < 7; i++) {
    if (!isfinite(imuData[i])) {
      if (debugging) {
        debugPrintln("IMU invalid: NaN/Inf");
      }
      return false;
    }
  }
  return true;
}


// ============================================================================
// Setup
// ============================================================================
void setup() {
  if (debugging) {
    Serial.begin(115200);
  } else {
    Serial.begin(5000000);   // 5 Mbaud for real-time streaming
  }

  delay(100);
  RPC.begin();                  // start communication with M4
  shared_data->ready = 0;

  // Initialise BNO055 over I2C (pins 20=SDA, 21=SCL)
  delay(100);
  if (bno.begin()) {
    delay(100);
    BNO_connected = true;
    bno.setExtCrystalUse(true);
    if (debugging) {
      debugPrintln("IMU detected and initialised");
    }
  }
  delay(100);

  if (debugging) {
    debugPrintln("M7 ready");
  }
}

// ============================================================================
// Main loop
// ============================================================================
void loop() {
  current_loop_time = millis();

  // --- 1. Forward any debug messages from M4 core ---
  if (RPC.available() > 0) {
    char buf[128];
    int idx = 0;
    delay(1);
    while (RPC.available() && idx < 127) {
      int c = RPC.read();
      if (c < 0) break;
      buf[idx++] = (char)c;
    }
    buf[idx] = '\0';
    debugPrintln("M4:" + String(buf));
  }

  // --- 2. Video frame ready from M4? ---
  if ((stream_frame || stream_tracking) && (shared_data->ready == 1) && (current_loop_time - last_frame_sent_time >= min_frame_interval)) {
    last_frame_sent_time = current_loop_time;
    
    // copy frame and release M4
    memcpy(frame, (uint8_t*)shared_data->image, frame_bytes);
    uint16_t cur_frame_num = shared_data->frame_num;
    uint32_t cur_timestamp = shared_data->timestamp;
    shared_data->ready = 0;

    // optional advanced processing (back porch, shift)
    if (do_advanced_processing) {
      for (int r = 0; r < frame_h; r++) {
        int row_start = r * frame_w;
        uint16_t bp_sum = 0;
        for (int i = 13; i <= 18; i++) bp_sum += frame[row_start + i]; //back porch estimated to always be with columns 13-18
        uint8_t bp_mean = bp_sum / 6;
        for (int c = 0; c < frame_w; c++) {
          int val = frame[row_start + c] - bp_mean;
          frame[row_start + c] = (val < 0) ? 0 : val;
        }
        // (sub‑pixel shift omitted – can be re‑added if needed)
      }
    }

    // downscale 3:1 (240 → 80 rows)
    for (int y = 0; y < small_frame_h; y++) {
      int src_y = y * 3;
      for (int x = 0; x < small_frame_w; x++) {
        uint16_t sum = frame[src_y * frame_w + x] +
                       frame[(src_y+1) * frame_w + x] +
                       frame[(src_y+2) * frame_w + x];
        small_frame[y * small_frame_w + x] = sum / 3;
      }
    }

    // crop to 80×80 (from 128×80)
    int safe_crop_x = constrain(crop_x, 0, small_frame_w - crop_frame_w);
    int safe_crop_y = constrain(crop_y, 0, small_frame_h - crop_frame_h);
    for (int cy = 0; cy < crop_frame_h; cy++) {
      for (int cx = 0; cx < crop_frame_w; cx++) {
        crop_frame[cy * crop_frame_w + cx] = small_frame[(safe_crop_y + cy) * small_frame_w + (safe_crop_x + cx)];
      }
    }

    // rate‑limit and send
    num_frames++;
        // --- Branch for tracking streaming ---
    if (stream_tracking) {
      // TODO: Implement eye-tracking on crop_frame
      // Need to define tracking_payload_bytes and a tracking packet format
      // e.g. sendTrackingData(cur_frame_num, cur_timestamp, tracking_results, camera_marker);
      
      // Placeholder: send a text message instead
      if (debugging) {
          debugPrintln("Tracking data would be sent here");
      }
    }
    // --- Branch for frame streaming ---
    if (stream_frame) {
      if (skipped_frames >= num_frames_to_skip) {
        skipped_frames = 0;
        if (debugging) {
          debugPrintln("Frame transfer");
        } else {
          if (stream_checkerboard) {
            fillCheckerboard(crop_frame);
          } else {
            sendVideoFrame(cur_frame_num, cur_timestamp, crop_frame, camera_marker);
          }
        }
      } else {
        skipped_frames++;
      }
    }
  }

  // --- 3. Read BNO055 IMU ---
  if (stream_imu && BNO_connected && (current_loop_time - last_imu_time >= imu_interval)) {
    last_imu_time = current_loop_time;

    // obtain quaternion and linear acceleration
    imu::Quaternion quat = bno.getQuat();
    sensors_event_t linAccEvent;
    bno.getEvent(&linAccEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

    imuData[0] = quat.w();
    imuData[1] = quat.x();
    imuData[2] = quat.y();
    imuData[3] = quat.z();
    imuData[4] = linAccEvent.acceleration.x;
    imuData[5] = linAccEvent.acceleration.y;
    imuData[6] = linAccEvent.acceleration.z;

    num_imu_samples++;
    if (validateIMUData()) {
      if (debugging) {
        debugPrintln("IMU transfer");
      } else {
        sendIMURecord(num_imu_samples, current_loop_time);
      }
    } else {
      if (debugging) {
        debugPrintln("IMU data invalid, skipped");
      }
    }
  }

  // --- 4. Serial commands ---
  while (Serial.available() >= 1) {
    uint8_t cmd = Serial.read();

    switch (cmd) {
      case CMD_START_FRAME:
        stream_frame = true;
        debugPrintln("Frame streaming started");
        break;

      case CMD_START_IMU:
        stream_imu = true;
        if (BNO_connected) {
          debugPrintln("IMU streaming started");
        }
        break;

      case CMD_START_TRACKING:
        stream_tracking = true;
        debugPrintln("Tracking streaming started (not implemented)");
        break;

      case CMD_STOP_ALL:
        stream_frame = false;
        stream_imu = false;
        stream_tracking = false;
        debugPrintln("All streaming stopped");
        break;

      case CMD_STATUS:
        debugPrintln("Giga CVBS Version " + version_name); // send firmware version name
        if (shared_data->m4_status == 0) {
          debugPrintln("M7 ADC not started, needs reprogramming");
        } else if (shared_data->m4_status == 1) {
          debugPrintln("No signal detected on ADC, check power and connections");
        } else if (shared_data->m4_status == 2) {
          debugPrintln("Camera connected");
        }
        if (BNO_connected) {
          debugPrintln("IMU Connected");
        }
        break;

      case settings_marker:
        last_command_time = millis();
        while ((Serial.available() < 2) && ((millis() - last_command_time) < 5)) {
          // wait briefly for crop_x and crop_y -- the next two bytes
        }
        if (Serial.available() >= 2) {
          crop_x = Serial.read();
          crop_y = Serial.read();
          debugPrintln("New crop settings received");
        }
        break;

      default:
        // ignore unknown commands
        break;
    }
  }

  delay(1);
}