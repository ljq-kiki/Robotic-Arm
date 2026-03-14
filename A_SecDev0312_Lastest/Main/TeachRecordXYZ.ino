// =================================================================================
// File: TeachRecordXYZ.ino
// Function: UI-Driven Teaching & Recording (Hardened against Bus Collisions)
// =================================================================================
#include <math.h>

extern int read_servo_pwm(uint8_t idx, uint32_t timeout_ms);
extern void set_servo(int servo_index, int pwm_value, int move_time);
extern void all_uart_send_str(char *str);

// 状态标志
bool has_start = false;
bool has_mid = false;
bool has_end = false;

int start_pos[5] = {1500, 1500, 1500, 1500, 1500};
int mid_pos[5]   = {1500, 1500, 1500, 1500, 1500};
int end_pos[5]   = {1500, 1500, 1500, 1500, 1500};

const float FK_L0 = 109.0, FK_L1 = 105.0, FK_L2 = 87.0, FK_L3_ACTUAL = 130.0;

void set_torque_all(const char* cmd_suffix) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "#255%s!", cmd_suffix);
  all_uart_send_str(cmd);
}

// 打印特定点的 XYZ 坐标
void print_xyz_from_pwm(int* pwm_array, const char* point_name) {
    float theta6 = (1500.0 - pwm_array[0]) * 270.0 / 2000.0;
    float theta5 = (pwm_array[1] - 1500.0) * 270.0 / 2000.0 + 90.0;
    float theta4 = (pwm_array[2] - 1500.0) * 270.0 / 2000.0;
    float theta3 = (pwm_array[3] - 1500.0) * 270.0 / 2000.0;
    float alpha_deg = theta5 - theta4 + theta3;
    float t6 = theta6 * PI / 180.0, t5 = theta5 * PI / 180.0, t4 = theta4 * PI / 180.0, alpha = alpha_deg * PI / 180.0;
    float r = FK_L1 * cos(t5) + FK_L2 * cos(t5 - t4) + FK_L3_ACTUAL * cos(alpha);
    float z = FK_L0 + FK_L1 * sin(t5) + FK_L2 * sin(t5 - t4) + FK_L3_ACTUAL * sin(alpha);
    float x = r * sin(t6), y = r * cos(t6);

    Serial.print("📍 ["); Serial.print(point_name); Serial.print(" Coords] X: "); Serial.print(x, 1);
    Serial.print(" | Y: "); Serial.print(y, 1);
    Serial.print(" | Z: "); Serial.print(z, 1);
    Serial.print(" | Rx: "); Serial.print(alpha_deg, 1); Serial.println(" deg");
}

// 核心读取函数：带智能延时，防止与 U 指令的回传撞车
bool read_current_servos(int* buffer) {
  bool success = true;
  for (int i = 0; i < 5; i++) {
    delay(20); // 【关键修复】每次读取前给串口总线20ms喘息时间
    int pwm = read_servo_pwm(i, 150);
    if (pwm < 0) {
        delay(50); // 遇到拥堵，避让50ms再试
        pwm = read_servo_pwm(i, 150);
    }
    if (pwm < 0) {
      success = false;
      buffer[i] = 1500; 
    } else {
      buffer[i] = pwm;
    }
  }
  return success;
}

void loop_teach_record() {
  if (Serial.available() > 0) {
    char key = (char)Serial.peek();
    
    if (key == '\n' || key == '\r') {
        Serial.read();
        return; 
    }
    key = toupper(key);

    if (key == 'A' || key == 'M' || key == 'B' || key == 'N' || key == 'V') {
        Serial.read();
        switch (key) {
          case 'A': // Grab Point
            if (read_current_servos(start_pos)) {
                has_start = true;
                Serial.println("\n✅ [OK] Grab Point (START) Saved!");
                print_xyz_from_pwm(start_pos, "Grab");
            } else {
                Serial.println("\n❌ [Error] Failed to read Grab Point.");
            }
            break;

          case 'M': // Mid Point
            if (read_current_servos(mid_pos)) {
                has_mid = true;
                Serial.println("\n✅ [OK] Mid Point Saved!");
                print_xyz_from_pwm(mid_pos, "Mid");
            } else {
                Serial.println("\n❌ [Error] Failed to read Mid Point.");
            }
            break;

          case 'B': // Drop Point
            if (read_current_servos(end_pos)) {
                has_end = true;
                Serial.println("\n✅ [OK] Drop Point (END) Saved!");
                print_xyz_from_pwm(end_pos, "Drop");
            } else {
                Serial.println("\n❌ [Error] Failed to read Drop Point.");
            }
            break;

          case 'N': // Reset
            has_start = false; has_mid = false; has_end = false;
            Serial.println("\n🗑️ [Reset] All recorded points cleared.");
            break;

          case 'V': // Auto Run
            if (has_start && has_end) {
              Serial.println("\n▶️ [Auto Run] Starting trajectory...");
              set_torque_all("PULR"); // 先锁定
              delay(500);

              Serial.println(">>> 1. Moving to Grab Point...");
              for (int i = 0; i < 5; i++) set_servo(i, start_pos[i], 2000);
              delay(2500);

              if (has_mid) {
                  Serial.println(">>> 2. Moving to Mid Point...");
                  for (int i = 0; i < 5; i++) set_servo(i, mid_pos[i], 2000);
                  delay(2500);
                  Serial.println(">>> 3. Moving to Drop Point...");
              } else {
                  Serial.println(">>> 2. Moving direct to Drop Point...");
              }
              
              for (int i = 0; i < 5; i++) set_servo(i, end_pos[i], 2000);
              delay(2500);
              Serial.println("🏁 [Auto Run] Complete!");
            } else {
              Serial.println("\n⚠️ [Error] Need at least Grab (A) and Drop (B) points.");
            }
            break;
        }
    }
  }
}
