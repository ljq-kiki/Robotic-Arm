// // =================================================================================
// // File: TeachRecordXYZ.ino
// // Function: UI-Driven Teaching & Recording (Bulletproof Bus Collision Fix)
// // =================================================================================
// #include <math.h>

// extern int read_servo_pwm(uint8_t idx, uint32_t timeout_ms);
// extern void set_servo(int servo_index, int pwm_value, int move_time);
// extern void all_uart_send_str(char *str);
// extern HardwareSerial Serial1; // 【核心修复】暴露出底层串口，用于强制清空总线垃圾

// // 状态标志
// bool has_start = false;
// bool has_mid = false;
// bool has_end = false;

// int start_pos[5] = {1500, 1500, 1500, 1500, 1500};
// int mid_pos[5]   = {1500, 1500, 1500, 1500, 1500};
// int end_pos[5]   = {1500, 1500, 1500, 1500, 1500};

// const float FK_L0 = 109.0, FK_L1 = 105.0, FK_L2 = 87.0, FK_L3_ACTUAL = 130.0;

// void set_torque_all(const char* cmd_suffix) {
//   char cmd[16];
//   snprintf(cmd, sizeof(cmd), "#255%s!", cmd_suffix);
//   all_uart_send_str(cmd);
// }

// void print_xyz_from_pwm(int* pwm_array, const char* point_name) {
//     float theta6 = (1500.0 - pwm_array[0]) * 270.0 / 2000.0;
//     float theta5 = (pwm_array[1] - 1500.0) * 270.0 / 2000.0 + 90.0;
//     float theta4 = (pwm_array[2] - 1500.0) * 270.0 / 2000.0;
//     float theta3 = (pwm_array[3] - 1500.0) * 270.0 / 2000.0;
//     float alpha_deg = theta5 - theta4 + theta3;
//     float t6 = theta6 * PI / 180.0, t5 = theta5 * PI / 180.0, t4 = theta4 * PI / 180.0, alpha = alpha_deg * PI / 180.0;
//     float r = FK_L1 * cos(t5) + FK_L2 * cos(t5 - t4) + FK_L3_ACTUAL * cos(alpha);
//     float z = FK_L0 + FK_L1 * sin(t5) + FK_L2 * sin(t5 - t4) + FK_L3_ACTUAL * sin(alpha);
//     float x = r * sin(t6), y = r * cos(t6);

//     Serial.print("📍 ["); Serial.print(point_name); Serial.print(" Coords] X: "); Serial.print(x, 1);
//     Serial.print(" | Y: "); Serial.print(y, 1);
//     Serial.print(" | Z: "); Serial.print(z, 1);
//     Serial.print(" | Rx: "); Serial.print(alpha_deg, 1); Serial.println(" deg");
// }

// // 核心读取函数：带智能延时、暴力清空和重试机制
// bool read_current_servos(int* buffer) {
//   bool success = true;

//   for (int i = 0; i < 5; i++) {
//     int pwm = -1;
    
//     // 赋予 3 次重试机会
//     for (int retry = 0; retry < 3; retry++) {
//       // 【关键修复】读取前，暴力清空底层硬件缓冲区里的乱码和冲突响应
//       while (Serial1.available() > 0) {
//         Serial1.read();
//       }
//       delay(30);

//       pwm = read_servo_pwm(i, 200); // 增加超时时间
//       if (pwm > 0) break; // 读取成功，跳出重试
      
//       delay(100); // 如果失败，避让 100ms 等总线彻底安静
//     }

//     if (pwm < 0) {
//       success = false;
//       buffer[i] = 1500; 
//     } else {
//       buffer[i] = pwm;
//     }
//   }

//   // 【必须保留】以特定格式打印，确保前端网页的正则能够抓取到 "JointX="
//   if (success) {
//     for (int i = 0; i < 5; i++) {
//        Serial.print("Joint"); Serial.print(i); Serial.print("="); Serial.print(buffer[i]); Serial.print("  ");
//     }
//     Serial.println();
//   }

//   return success;
// }

// void loop_teach_record() {
//   if (Serial.available() > 0) {
//     char key = (char)Serial.peek();
    
//     if (key == '\n' || key == '\r') {
//         Serial.read();
//         return; 
//     }
//     key = toupper(key);

//     if (key == 'A' || key == 'M' || key == 'B' || key == 'N' || key == 'V') {
//         Serial.read();
//         switch (key) {
//           case 'A': // Grab Point
//             if (read_current_servos(start_pos)) {
//                 has_start = true;
//                 Serial.println("\n✅ [OK] Grab Point (START) Saved!");
//                 print_xyz_from_pwm(start_pos, "Grab");
//             } else {
//                 Serial.println("\n❌ [Error] Failed to read Grab Point.");
//             }
//             break;

//           case 'M': // Mid Point
//             if (read_current_servos(mid_pos)) {
//                 has_mid = true;
//                 Serial.println("\n✅ [OK] Mid Point Saved!");
//                 print_xyz_from_pwm(mid_pos, "Mid");
//             } else {
//                 Serial.println("\n❌ [Error] Failed to read Mid Point.");
//             }
//             break;

//           case 'B': // Drop Point
//             if (read_current_servos(end_pos)) {
//                 has_end = true;
//                 Serial.println("\n✅ [OK] Drop Point (END) Saved!");
//                 print_xyz_from_pwm(end_pos, "Drop");
//             } else {
//                 Serial.println("\n❌ [Error] Failed to read Drop Point.");
//             }
//             break;

//           case 'N': // Reset
//             has_start = false; has_mid = false; has_end = false;
//             Serial.println("\n🗑️ [Reset] All recorded points cleared.");
//             break;

//           case 'V': // Auto Run
//             if (has_start && has_end) {
//               Serial.println("\n▶️ [Auto Run] Starting trajectory...");
//               set_torque_all("PULR"); // 先锁定
//               delay(500);

//               Serial.println(">>> 1. Moving to Grab Point...");
//               for (int i = 0; i < 5; i++) set_servo(i, start_pos[i], 2000);
//               delay(2500);

//               if (has_mid) {
//                   Serial.println(">>> 2. Moving to Mid Point...");
//                   for (int i = 0; i < 5; i++) set_servo(i, mid_pos[i], 2000);
//                   delay(2500);
//                   Serial.println(">>> 3. Moving to Drop Point...");
//               } else {
//                   Serial.println(">>> 2. Moving direct to Drop Point...");
//               }
              
//               for (int i = 0; i < 5; i++) set_servo(i, end_pos[i], 2000);
//               delay(2500);
//               Serial.println("🏁 [Auto Run] Complete!");
//             } else {
//               Serial.println("\n⚠️ [Error] Need at least Grab (A) and Drop (B) points.");
//             }
//             break;
//         }
//     }
//   }
// }














// #include <math.h>

// extern int read_servo_pwm(uint8_t idx, uint32_t timeout_ms);
// extern void set_servo(int servo_index, int pwm_value, int move_time);
// extern void all_uart_send_str(char *str);

// const float FK_L0 = 109.0; 
// const float FK_L1 = 105.0; 
// const float FK_L2 = 87.0;  
// const float FK_L3_ACTUAL = 80.0 + 50.0;

// enum TeachState { STATE_IDLE, STATE_TEACHING, STATE_WAIT_CONFIRM_A, STATE_WAIT_CONFIRM_B, STATE_READY };

// TeachState current_state = STATE_IDLE;
// int start_pos_buffer[5], end_pos_buffer[5], final_start_pos[5], final_end_pos[5];
// bool has_start = false, has_end = false;

// void set_torque_all(const char* cmd_suffix) {
//   char cmd[16];
//   snprintf(cmd, sizeof(cmd), "#255%s!", cmd_suffix);
//   all_uart_send_str(cmd); 
// }

// void print_xyz_from_pwm(int* pwm_array) {
//     float pwm0 = pwm_array[0], pwm1 = pwm_array[1], pwm2 = pwm_array[2], pwm3 = pwm_array[3];
//     float theta6 = (1500.0 - pwm0) * 270.0 / 2000.0;
//     float theta5 = (pwm1 - 1500.0) * 270.0 / 2000.0 + 90.0;
//     float theta4 = (pwm2 - 1500.0) * 270.0 / 2000.0;
//     float theta3 = (pwm3 - 1500.0) * 270.0 / 2000.0;

//     float alpha_deg = theta5 - theta4 + theta3;
//     float t6 = theta6 * PI / 180.0, t5 = theta5 * PI / 180.0, t4 = theta4 * PI / 180.0, alpha = alpha_deg * PI / 180.0;

//     float r = FK_L1 * cos(t5) + FK_L2 * cos(t5 - t4) + FK_L3_ACTUAL * cos(alpha);
//     float z = FK_L0 + FK_L1 * sin(t5) + FK_L2 * sin(t5 - t4) + FK_L3_ACTUAL * sin(alpha);
//     float x = r * sin(t6), y = r * cos(t6);

//     Serial.println("--------------------------------------------------");
//     Serial.print("📍 [Coords] X: "); Serial.print(x, 1);
//     Serial.print(" mm | Y: "); Serial.print(y, 1);
//     Serial.print(" mm | Z: "); Serial.print(z, 1);
//     Serial.print(" mm | Pitch: "); Serial.print(alpha_deg, 1); Serial.println(" deg");
//     Serial.println("--------------------------------------------------");
// }

// bool read_all_servos(int* buffer) {
//   Serial.println(">>> Reading physical joint angles...");
//   bool success = true;
//   for (int i = 0; i < 5; i++) {
//     int pwm = read_servo_pwm(i, 100);
//     if (pwm < 0) {
//       Serial.print("Error: Read servo "); Serial.print(i); Serial.println(" failed!");
//       success = false; buffer[i] = 1500; 
//     } else {
//       buffer[i] = pwm;
//       Serial.print("Joint"); Serial.print(i); Serial.print("="); Serial.print(pwm); Serial.print("  ");
//     }
//   }
//   Serial.println();
//   if (success) print_xyz_from_pwm(buffer);
//   return success;
// }

// void loop_teach_record() {
//   if (Serial.available() > 0) {
//     char key = toupper((char)Serial.peek()); 
//     if (key == '\n' || key == '\r') return; 
    
//     if (key == 'C' || key == 'A' || key == 'Y' || key == 'B' || key == 'N' || key == 'V') {
//         Serial.read();
//         switch (key) {
//           case 'C':
//             Serial.println("\n[CMD] Start Teaching (C)");
//             set_torque_all("PULK"); 
//             current_state = STATE_TEACHING; has_start = false; has_end = false;
//             Serial.println(">>> Unlocked. Manually move to START pos.");
//             break;
//           case 'A':
//             if (current_state == STATE_TEACHING) {
//               Serial.println("\n[CMD] Record START (A)");
//               if (read_all_servos(start_pos_buffer)) {
//                 current_state = STATE_WAIT_CONFIRM_A;
//                 Serial.println(">>> Coords calculated. Press 'Y' to confirm, 'N' to cancel.");
//               }
//             } else Serial.println(">>> Error: Press 'C' first.");
//             break;
//           case 'Y':
//             if (current_state == STATE_WAIT_CONFIRM_A) {
//               memcpy(final_start_pos, start_pos_buffer, sizeof(start_pos_buffer));
//               has_start = true; current_state = STATE_TEACHING; 
//               Serial.println("\n✅ [OK] START saved! Move to END pos.");
//             } else if (current_state == STATE_WAIT_CONFIRM_B) {
//               memcpy(final_end_pos, end_pos_buffer, sizeof(end_pos_buffer));
//               has_end = true; set_torque_all("PULR"); current_state = STATE_READY;
//               Serial.println("\n✅ [OK] END saved! Locked. Ready for AUTO RUN (V).");
//             }
//             break;
//           case 'B':
//             if (current_state == STATE_TEACHING && has_start) {
//               Serial.println("\n[CMD] Record END (B)");
//               if (read_all_servos(end_pos_buffer)) {
//                 current_state = STATE_WAIT_CONFIRM_B;
//                 Serial.println(">>> Coords calculated. Press 'Y' to confirm, 'N' to cancel.");
//               }
//             } else Serial.println(">>> Error: Record START first.");
//             break;
//           case 'N':
//             Serial.println("\n[CMD] Cancel/Reset (N)");
//             if (current_state == STATE_WAIT_CONFIRM_A || current_state == STATE_WAIT_CONFIRM_B) {
//                 current_state = STATE_TEACHING;
//                 Serial.println(">>> Cancelled. Move again and record.");
//             } else {
//                 set_torque_all("PULR"); current_state = STATE_IDLE; has_start = false; has_end = false;
//                 Serial.println(">>> 🗑️ System Reset.");
//             }
//             break;
//           case 'V':
//             Serial.println("\n[CMD] Auto Run (V)");
//             if (has_start && has_end) {
//               set_torque_all("PULR"); delay(500);
//               Serial.println(">>> 1. Moving to START...");
//               for (int i = 0; i < 5; i++) set_servo(i, final_start_pos[i], 2000);
//               delay(2500);
//               Serial.println(">>> 2. Moving to END...");
//               for (int i = 0; i < 5; i++) set_servo(i, final_end_pos[i], 2000);
//               delay(2500);
//               Serial.println(">>> 🏁 Run complete!");
//             } else Serial.println(">>> Error: Set START and END first.");
//             break;
//         }
//     }
//   }
// }


// =================================================================================
// File: TeachRecordXYZ.ino 之前运行的没问题的
// Function: Keyboard-controlled teaching and recording with real-time XYZ output
// =================================================================================
#include <math.h>

extern int read_servo_pwm(uint8_t idx, uint32_t timeout_ms);
extern void set_servo(int servo_index, int pwm_value, int move_time);
extern void all_uart_send_str(char *str);

const float FK_L0 = 109.0;  
const float FK_L1 = 105.0;  
const float FK_L2 = 87.0;   
const float FK_L3_ACTUAL = 80.0 + 50.0; 

enum TeachState {
  STATE_IDLE,
  STATE_TEACHING,
  STATE_WAIT_CONFIRM_A,
  STATE_WAIT_CONFIRM_B,
  STATE_READY
};

TeachState current_state = STATE_IDLE;
int start_pos_buffer[5];
int end_pos_buffer[5];
int final_start_pos[5];
int final_end_pos[5];
bool has_start = false;
bool has_end = false;

void set_torque_all(const char* cmd_suffix) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "#255%s!", cmd_suffix);
  all_uart_send_str(cmd); 
}

void print_xyz_from_pwm(int* pwm_array) {
    float pwm0 = pwm_array[0], pwm1 = pwm_array[1], pwm2 = pwm_array[2], pwm3 = pwm_array[3];
    float theta6 = (1500.0 - pwm0) * 270.0 / 2000.0;
    float theta5 = (pwm1 - 1500.0) * 270.0 / 2000.0 + 90.0;
    float theta4 = (pwm2 - 1500.0) * 270.0 / 2000.0;
    float theta3 = (pwm3 - 1500.0) * 270.0 / 2000.0;
    float alpha_deg = theta5 - theta4 + theta3;
    float t6 = theta6 * PI / 180.0, t5 = theta5 * PI / 180.0, t4 = theta4 * PI / 180.0, alpha = alpha_deg * PI / 180.0;
    float r = FK_L1 * cos(t5) + FK_L2 * cos(t5 - t4) + FK_L3_ACTUAL * cos(alpha);
    float z = FK_L0 + FK_L1 * sin(t5) + FK_L2 * sin(t5 - t4) + FK_L3_ACTUAL * sin(alpha);
    float x = r * sin(t6), y = r * cos(t6);

    Serial.println("--------------------------------------------------");
    Serial.print("📍 [Coords] X: "); Serial.print(x, 1);
    Serial.print(" mm | Y: "); Serial.print(y, 1);
    Serial.print(" mm | Z: "); Serial.print(z, 1);
    Serial.print(" mm | Pitch: "); Serial.print(alpha_deg, 1); Serial.println(" deg");
    Serial.println("--------------------------------------------------");
}

bool read_all_servos(int* buffer) {
  Serial.println(">>> Reading physical joint angles...");
  bool success = true;
  
  for (int i = 0; i < 5; i++) {
    int pwm = read_servo_pwm(i, 100);
    
    // 【修复 2】如果读取失败，总线可能在忙，喘息 20ms 后立刻重试一次
    if (pwm < 0) {
        delay(20);
        pwm = read_servo_pwm(i, 100);
    }

    if (pwm < 0) {
      Serial.print("Error: Read servo "); Serial.print(i); Serial.println(" failed!");
      success = false;
      buffer[i] = 1500; 
    } else {
      buffer[i] = pwm;
      Serial.print("Joint"); Serial.print(i); Serial.print("="); Serial.print(pwm); Serial.print("  ");
    }
  }
  Serial.println();
  if (success) print_xyz_from_pwm(buffer);
  return success;
}

void loop_teach_record() {
  if (Serial.available() > 0) {
    char key = (char)Serial.peek(); 
    
    // 【修复 3】看到换行符必须强制拿走（read），否则会造成无限死循环阻断其他按键！
    if (key == '\n' || key == '\r') {
        Serial.read(); 
        return; 
    }
    key = toupper(key);

    if (key == 'C' || key == 'A' || key == 'Y' || key == 'B' || key == 'N' || key == 'V') {
        Serial.read(); 
        switch (key) {
          case 'C':
            Serial.println("\n[CMD] Start Teaching (C)");
            set_torque_all("PULK"); 
            current_state = STATE_TEACHING;
            has_start = false; has_end = false;
            Serial.println(">>> Unlocked. Manually move to START pos.");
            break;

          case 'A':
            if (current_state == STATE_TEACHING) {
              Serial.println("\n[CMD] Record START (A)");
              if (read_all_servos(start_pos_buffer)) {
                current_state = STATE_WAIT_CONFIRM_A;
                Serial.println(">>> Coords calculated. Press 'Y' to confirm, 'N' to cancel.");
              } else {
                Serial.println(">>> Error: Read failed, try 'A' again.");
              }
            } else Serial.println(">>> Error: Press 'C' first to enter teaching mode.");
            break;

          case 'Y':
            if (current_state == STATE_WAIT_CONFIRM_A) {
              memcpy(final_start_pos, start_pos_buffer, sizeof(start_pos_buffer));
              has_start = true; current_state = STATE_TEACHING; 
              Serial.println("\n✅ [OK] START saved!");
              Serial.println(">>> Move to END pos and press 'B'.");
            } else if (current_state == STATE_WAIT_CONFIRM_B) {
              memcpy(final_end_pos, end_pos_buffer, sizeof(end_pos_buffer));
              has_end = true; set_torque_all("PULR"); current_state = STATE_READY;
              Serial.println("\n✅ [OK] END saved!");
              Serial.println(">>> Locked. Ready for AUTO RUN (V).");
            }
            break;

          case 'B':
            if (current_state == STATE_TEACHING) {
              if (!has_start) {
                Serial.println(">>> Error: Record START (A) first.");
                break;
              }
              Serial.println("\n[CMD] Record END (B)");
              if (read_all_servos(end_pos_buffer)) {
                current_state = STATE_WAIT_CONFIRM_B;
                Serial.println(">>> Coords calculated. Press 'Y' to confirm, 'N' to cancel.");
              } else Serial.println(">>> Error: Read failed, try 'B' again.");
            } else Serial.println(">>> Error: Invalid state for 'B'.");
            break;

          case 'N':
            Serial.println("\n[CMD] Cancel/Reset (N)");
            if (current_state == STATE_WAIT_CONFIRM_A) {
                current_state = STATE_TEACHING;
                Serial.println(">>> Cancelled START. Move again and press 'A'.");
            } else if (current_state == STATE_WAIT_CONFIRM_B) {
                current_state = STATE_TEACHING;
                Serial.println(">>> Cancelled END. Move again and press 'B'.");
            } else {
                set_torque_all("PULR"); current_state = STATE_IDLE;
                has_start = false; has_end = false;
                Serial.println(">>> 🗑️ System Reset. All data cleared.");
            }
            break;

          case 'V':
            Serial.println("\n[CMD] Auto Run (V)");
            if (has_start && has_end) {
              set_torque_all("PULR"); delay(500);
              Serial.println(">>> 1. Moving to START...");
              for (int i = 0; i < 5; i++) set_servo(i, final_start_pos[i], 2000);
              delay(2500);
              Serial.println(">>> 2. Moving to END...");
              for (int i = 0; i < 5; i++) set_servo(i, final_end_pos[i], 2000);
              delay(2500);
              Serial.println(">>> 🏁 Run complete!");
            } else Serial.println(">>> Error: Set START and END first.");
            break;
        }
    }
  }
}
