// =================================================================================
// File: TeachRecordXYZ.ino
// Function: UI-Driven Teaching & Recording (Supports Grab, Mid, and Drop Points)
// =================================================================================
#include <math.h>

extern int read_servo_pwm(uint8_t idx, uint32_t timeout_ms);
extern void set_servo(int servo_index, int pwm_value, int move_time);
extern void all_uart_send_str(char *str);

// 状态标志
bool has_start = false;
bool has_mid = false;
bool has_end = false;

// 坐标数组 (存放5个关节的PWM值)
int start_pos[5] = {1500, 1500, 1500, 1500, 1500};
int mid_pos[5]   = {1500, 1500, 1500, 1500, 1500};
int end_pos[5]   = {1500, 1500, 1500, 1500, 1500};

void set_torque_all(const char* cmd_suffix) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "#255%s!", cmd_suffix);
  all_uart_send_str(cmd);
}

// 核心读取函数 (带自动重试防干扰)
bool read_current_servos(int* buffer) {
  bool success = true;
  for (int i = 0; i < 5; i++) {
    int pwm = read_servo_pwm(i, 100);
    if (pwm < 0) {
        delay(20);
        pwm = read_servo_pwm(i, 100); // 失败重试一次
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
    
    // 吞掉换行符
    if (key == '\n' || key == '\r') {
        Serial.read();
        return; 
    }
    key = toupper(key);

    // 匹配到指令才正式读取
    if (key == 'A' || key == 'M' || key == 'B' || key == 'N' || key == 'V') {
        Serial.read();
        switch (key) {
          case 'A': // 记录 Grab Point (起点)
            if (read_current_servos(start_pos)) {
                has_start = true;
                Serial.println("\n✅ [OK] Grab Point (START) Saved!");
            } else {
                Serial.println("\n❌ [Error] Failed to read Grab Point.");
            }
            break;

          case 'M': // 记录 Mid Point (可选中间点)
            if (read_current_servos(mid_pos)) {
                has_mid = true;
                Serial.println("\n✅ [OK] Mid Point Saved!");
            } else {
                Serial.println("\n❌ [Error] Failed to read Mid Point.");
            }
            break;

          case 'B': // 记录 Drop Point (终点)
            if (read_current_servos(end_pos)) {
                has_end = true;
                Serial.println("\n✅ [OK] Drop Point (END) Saved!");
            } else {
                Serial.println("\n❌ [Error] Failed to read Drop Point.");
            }
            break;

          case 'N': // 取消/重置所有点
            has_start = false;
            has_mid = false;
            has_end = false;
            Serial.println("\n🗑️ [Reset] All recorded points cleared.");
            break;

          case 'V': // 执行轨迹
            if (has_start && has_end) {
              Serial.println("\n▶️ [Auto Run] Starting trajectory...");
              set_torque_all("PULR"); // 先上力锁定
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
              Serial.println("\n⚠️ [Error] Cannot run. You must record at least Grab (A) and Drop (B) points.");
            }
            break;
        }
    }
  }
}
