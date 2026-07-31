/*
Giga_CVBS_M4.ino – Composite B/W Video Decoder for Arduino Giga R1
- 8 MS/s CVBS video sampling at 8-bit resolution
- Auto clock-level and blank-level detection using rolling median
- 128W×240H video transferred at 60 fps to SRAM for sharing with M7 core
*/

#include <Arduino_AdvancedAnalog.h>
#include <algorithm>
#include <arduino.h>
#include <RPC.h>

const bool debugging = false;

AdvancedADC adc_cam(A0);

// Shared memory (AHB SRAM4) for M4-M7 communication -- offset by 32KB to avoid the RPC OpenAMP buffers
const uint32_t SDRAM_START_ADDRESS_4 = 0x38008000;
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
DATA* shared_data = (DATA*)(sdramMemory);

// Clock detection with moving average
// typical size of image line at 8 MS/s is 127 samples
// typical size of shorter, early interframe line at 8 MS/s is ~63 samples
int min_short_line_samples = 50; // any line shorter is a mistake
int max_short_line_samples = 75; // any line longer is not an early interframe clock

// For auto-calibrated of signal values
const int num_samples_to_inspect = 48000; // number of ADC samples to collect
const int ind_to_find_clock = 1000;
uint8_t calibration_array[48000];
uint8_t min_level = 0;
uint8_t clock_level = 0;
uint8_t blank_level = 60;
uint8_t max_level = 200;
uint8_t clock_threshold = 5;

// Line and frame arrays
const int frame_w = 128;          // close to full line size (including clock, porches, image)
const int frame_h = 240;          
uint8_t current_line[frame_w];
uint8_t current_frame[frame_w * frame_h];

// State variables
bool frame_capturing = true;
bool wait_for_falling_edge = false;

// Signal variables
int current_sample_ind = 0;       // index of current sample into current line
int current_sample_num = 0;       // total samples collected in current line
int current_line_num = 0;         // total rows collected in current frame
uint16_t num_frames_captured = 0; // total frames collected
uint8_t current_sample = 255;     // current ADC value
unsigned long frame_time = millis(); // timestamp of current frame


// Rolling median helper (naive, called once at startup)
uint8_t rolling_median_block(uint8_t* arr, int start, int len) {
  uint8_t window[len];
  for (int i = 0; i < len; i++) window[i] = arr[start + i];
  std::sort(window, window + len);
  return window[len/2];
}

// ========== Save line into frame array ==========
void process_line(uint8_t* raw_samples, int end_ind, int row) {
  row = row % frame_h;
  int dest_offset = row * frame_w;
  int start_idx = (end_ind + 1) % frame_w;
  int first_chunk_len = frame_w - start_idx;
  
  // Copy the back half of the circular buffer to the front of the line
  memcpy(&current_frame[dest_offset], &raw_samples[start_idx], first_chunk_len);
  // Copy the front half of the circular buffer to the back of the line
  memcpy(&current_frame[dest_offset + first_chunk_len], &raw_samples[0], start_idx);
}


// ========== Save frame into SDRAM ==========
void process_frame(int captured_lines) {
  num_frames_captured++;
  frame_time = millis();
  
  if (debugging) {
    RPC.println("Captured frame " + String(num_frames_captured) + " t=" + String(frame_time) + " lines=" + String(captured_lines) + " ready=" + String(shared_data->ready));
  } else if (shared_data->ready == 0) {
    shared_data->timestamp = frame_time;
    shared_data->frame_num = num_frames_captured;
    shared_data->captured_lines = current_line_num; 
    memcpy((uint8_t*)shared_data->image, current_frame, 30720); // Transfer the frame array into SRAM as quickly as possible
    shared_data->ready = 1;
  }
}


// ========== Setup ==========
void setup() {
  RPC.begin();
  delay(10000);

  shared_data->m4_status = 0;
  //start ADC at 8-bit, 8 MS/s, 64-byte buffers, 64 buffers deep
  if (!adc_cam.begin(AN_RESOLUTION_8, 8000000, 64, 64)) {
    if (debugging) {
      Serial.println("Failed to start ADC");
    }
    while (1);
  }
  shared_data->m4_status = 1; // ADC started

  // ----- Perform signal calibration with rolling median -----
  current_sample_num = 0;
  while (current_sample_num < num_samples_to_inspect) {
    SampleBuffer buf = adc_cam.read();
    for (int i = 0; i < buf.size(); i++) {
      calibration_array[current_sample_num++] = buf.data()[i];
    }
    buf.release();
  }
  current_sample_num = 0;

  // Moving median to find blank level
  uint8_t min_median = 255;
  for (int i = 600; i < num_samples_to_inspect - 600; i = i + 400) {
    uint8_t med = rolling_median_block(calibration_array, i - 600, 1200);
    if (med < min_median) min_median = med;
    if (debugging && (med<2)) {
      delay(100);
      RPC.println("printing raw values...");
      for (int j = i-600; j < i+600; j++) {
        RPC.println(calibration_array[j]);
      }
      RPC.println("done.");
    }
  }
  blank_level = min_median;
  if (blank_level < 2){
    if (debugging) {
      RPC.println("No camera signal detected. Check camera power and connections, then restart.");
    }
    while (1);
  }
  std::sort(calibration_array, calibration_array + num_samples_to_inspect);
  min_level = calibration_array[1];
  clock_level = calibration_array[ind_to_find_clock];
  max_level = calibration_array[num_samples_to_inspect - ind_to_find_clock];
  clock_threshold = (clock_level + blank_level) / 2;
  if (debugging) {
    RPC.println("min_level " + String(min_level));
    delay(10);
    RPC.println("clock_level " + String(clock_level));
    delay(10);
    RPC.println("blank_level " + String(blank_level));
    delay(10);
    RPC.println("max_level " + String(max_level));
    delay(10);
  }

  shared_data->m4_status = 2; //signal detected, all working
}


// ========== Main Loop with Rising/Falling Clock Edge Detection State Machine ==========
void loop() {
  if (adc_cam.available() > 0) {
    SampleBuffer buf = adc_cam.read();
    
    for (int i = 0; i < buf.size(); i++) {
      current_sample = buf.data()[i];

      // --- Edge detection state machine ---
      if (wait_for_falling_edge) {
        if (current_sample < clock_threshold) {
          
          // Noise filter: Ignore falling edges that happen way too soon
          if (current_sample_num >= min_short_line_samples) {
            
            // Falling edge detected – end of previous line's active video
            if (current_sample_num > max_short_line_samples) {
              // --- Normal image line ---
              if (!frame_capturing) {
                frame_capturing = true;
                current_line_num = 0;
              }
              // Save line, increment for the next line
              process_line(current_line, current_sample_ind, current_line_num);
              current_line_num++;
              
            } else {
              // --- Interframe short line ---
              if (frame_capturing) { // short line indicates frame is done
                // Save frame and reset for the next frame
                process_frame(current_line_num);
                frame_capturing = false;
              }
              current_line_num = 0;
            }

            // Reset for the start of the next line
            current_sample_num = 0;
            current_sample_ind = 0;
            wait_for_falling_edge = false;
          }
        }
      } else { 
        // After falling edge, keep looking for the next rising edge
        if (current_sample > clock_threshold) {
          wait_for_falling_edge = true;
        }
      }

      // --- Store sample in circular line buffer ---
      current_line[current_sample_ind] = current_sample;
      current_sample_ind = (current_sample_ind + 1) % frame_w;
      current_sample_num++;
    }

    buf.release(); // release completed buffer back into the pool
  }
}
