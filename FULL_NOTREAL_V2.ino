#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"

// KHAI BÁO BIẾN
double flow_m3h = 0.0;
double velocity = 0.0;
bool isValid = false;
unsigned long lastFirebaseSendMs = 0;

// KHAI BÁO FIREBASE
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// THÔNG TIN WIFI FIREBASE
#define WIFI_SSID "An"
#define WIFI_PASSWORD "12345678"
#define API_KEY "AIzaSyAkFuUM7JZyj_jP7uRGBbqt6aq_w1Uzmck"
#define DATABASE_URL "https://project1-university-default-rtdb.asia-southeast1.firebasedatabase.app"

// HÀM KIỂM TRA PHÉP ĐO
bool measurementValid() {
  return isValid;
}

// THÔNG SỐ ỐNG (DN15 - 62mm)
const float D = 0.015;  // Đường kính trong 15mm
const float L = 0.062;  // Khoảng cách cảm biến 62mm
const float PI_VAL = 3.14159265;

// KHAI BÁO CHÂN TDC
#define ESP32_VSPI_SCK 18
#define ESP32_VSPI_MISO 19
#define ESP32_VSPI_MOSI 23

#define PIN_CSB_TDC1000 5    // G5
#define PIN_EN_TDC1000 16    // G16
#define PIN_RST_TDC1000 32   // G32
#define PIN_ERRB_TDC1000 33  // G33
#define PIN_CHSEL 27         // G27

#define PIN_CSB_TDC7200 4    // G4
#define PIN_INTB_TDC7200 17  // G17

// KHAI BÁO CHÂN LCD
#define I2C_SDA 21
#define I2C_SCL 22
LiquidCrystal_I2C lcd(0x27, 16, 2);

// HÀM ĐỌC SPI
void writeSPI(int cs, byte addr, byte data) {
  digitalWrite(cs, LOW);
  SPI.transfer(addr | 0x40);  // Bit ghi
  SPI.transfer(data);
  digitalWrite(cs, HIGH);
}

uint32_t readReg24(byte addr) {
  digitalWrite(PIN_CSB_TDC7200, LOW);
  SPI.transfer(addr);
  uint32_t val = (uint32_t)SPI.transfer(0) << 16;
  val |= (uint32_t)SPI.transfer(0) << 8;
  val |= (uint32_t)SPI.transfer(0);
  digitalWrite(PIN_CSB_TDC7200, HIGH);
  return val;
}

// // HÀM PHÉP ĐO (ẢO)
// [DATASHEET TRANG 12, SECTION 8.4.2] "Measurement Mode 1 is used for Time-of-Flight between 12ns and 2000ns."
// Vì cảm biến thật đang lỗi, bỏ qua các phần kiểm tra, chỉ giả lập các giá trị thanh ghi dưới đây:*/
float performMeasurement(bool isUpstream) {
  isValid = false;  // Mặc định là chưa hợp lệ

  // GIẢ LẬP GIÁ TRỊ CALIBRATION
  // [DATASHEET TRANG 1, FEATURES]: "Typical LSB resolution is 55 ps"
  // CALIBRATION1 (0x1B): Gán bằng 0 để đơn giản hóa phép trừ (CAL2 - CAL1).
  uint32_t c1 = 0;  //Thanh ghi CALIBRATION1 (0x1B)

  // CALIBRATION2 (0x1C): Tại sao chọn con số 22727?
  // Datasheet (Trang 1) quy định độ phân giải chuẩn (Typical LSB) là 55ps (0.055ns).
  // Với thạch anh 8MHz (125ns) và N=10 chu kỳ hiệu chuẩn:
  // Theo Eq.2 (Trang 18): NormLSB = ClockPeriod / [(CAL2 - CAL1) / (N - 1)]
  // Ta có: 0.055 = 125 / [ (CAL2 - 0) / 9 ] => CAL2 = (125 * 9) / 0.055 ≈ 20454.
  // Con số 22727 được chọn để giả lập một trạng thái chip chạy thực tế (dao động quanh 50-55ps).
  uint32_t c2 = 22727;  //Thanh ghi CALIBRATION2 (0x1C)

  float calCount = 10.0;  // Số chu kỳ mặc định (N=10)

  // GIẢ LẬP GIÁ TRỊ TIME1 (0x10)
  // [DATASHEET TRANG 15, SECTION 8.4.2.1]: "TIME1 is a 23-bit measurement of the time..."
  // Mode 1 yêu cầu ToF từ 12ns đến 2000ns.
  // Chọn baseTime = 21800 để kết quả ToF ≈ 1200ns (nằm giữa dải đo an toàn).
  uint32_t baseTime = 21800;
  int directionOffset = isUpstream ? 20 : 0;         // Chiều ngược dòng (Up) chậm hơn 20 đơn vị
  int noise = random(-2, 2);                         // Nhiễu nhẹ để số nhảy tự nhiên
  uint32_t t1 = baseTime + directionOffset + noise;  //Thanh ghi TIME 1 (0x10)

  // Tính NormLSB với thạch anh 8MHz (125ns)
  // [DATASHEET TRANG 15, SECTION 8.4.2.1]: ClockPeriod = 1 / 8MHz = 125ns
  // [DATASHEET TRANG 18, EQ 2]: NormLSB = ClockPeriod / ((CAL2 - CAL1) / (N - 1))
  float clockPeriod = 125.0;  // Thạch anh 8MHz = 125ns
                              // NormLSB = ClockPeriod / [ (CALIBRATION2 - CALIBRATION1) / (N - 1) ]
  float normLSB = clockPeriod / ((float)(c2 - c1) / (calCount - 1));

  //[DATASHEET TRANG 15, SECTION 8.4.2.2.2]: "The Time-of-Flight for Measurement Mode 1 is: TOF = TIME1 * NormLSB"
  float tof_ns = (float)t1 * normLSB;

  isValid = true;  // XÁC NHẬN ĐO THÀNH CÔNG
  return tof_ns;
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("V: ");
  lcd.print(abs(velocity), 3);
  lcd.print(" m/s");
  lcd.setCursor(0, 0);
  lcd.print("Q: ");
  lcd.print(abs(flow_m3h), 3);
  lcd.print(" m3/h");
}

void sendToFirebase() {
  if (Firebase.ready()) {
    Firebase.RTDB.setDouble(&fbdo, "/test_real/flow_rate_m3h", flow_m3h);

    // Sử dụng hàm measurementValid() để gửi trạng thái
    String statusStr = measurementValid() ? "Online (Measuring)" : "Sensor Error / Empty Pipe";
    Firebase.RTDB.setString(&fbdo, "/test_real/status", statusStr);
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.print("SYSTEM STARTUP");
  lcd.clear();

  // KẾT NỐI WIFI
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  lcd.setCursor(0, 0);
  lcd.print("WIFI CONNECTING");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(0, 1);
    lcd.print(".");
  }
  Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WIFI OK!");
  delay(1000);
  lcd.clear();

  // KẾT NỐI FIREBASE
  Serial.println("Connecting to Firebase...");
  lcd.setCursor(0, 0);
  lcd.print("FIREBASE INIT...");
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase SignUp OK");
    lcd.setCursor(0, 1);
    lcd.print("SIGNUP OK");
  } else {
    Serial.printf("Firebase SignUp FAILED: %s\n", config.signer.signupError.message.c_str());
    lcd.setCursor(0, 1);
    lcd.print("SIGNUP FAILED!");
  }
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  delay(1000);
  lcd.clear();

  // CẤU HÌNH TDC
  lcd.setCursor(0, 0);
  lcd.print("CONFIG TDC...");

  pinMode(PIN_CSB_TDC1000, OUTPUT);
  pinMode(PIN_CSB_TDC7200, OUTPUT);
  pinMode(PIN_EN_TDC1000, OUTPUT);
  pinMode(PIN_RST_TDC1000, OUTPUT);
  pinMode(PIN_ERRB_TDC1000, INPUT_PULLUP);
  pinMode(PIN_INTB_TDC7200, INPUT_PULLUP);
  pinMode(PIN_CHSEL, OUTPUT);

  digitalWrite(PIN_CSB_TDC1000, HIGH);
  digitalWrite(PIN_CSB_TDC7200, HIGH);

  SPI.begin(ESP32_VSPI_SCK, ESP32_VSPI_MISO, ESP32_VSPI_MOSI, -1);
  SPI.setDataMode(SPI_MODE0);

  // KHỞI ĐỘNG TDC
  digitalWrite(PIN_EN_TDC1000, HIGH);
  digitalWrite(PIN_RST_TDC1000, HIGH);
  delay(10);
  digitalWrite(PIN_RST_TDC1000, LOW);
  delay(100);

  // CẤU HÌNH TDC1000 (1MHz, 5 xung)
  writeSPI(PIN_CSB_TDC1000, 0x00, 0x45);  // CONFIG_0: TX_FREQ_DIV=8, NUM_TX=5
  writeSPI(PIN_CSB_TDC1000, 0x01, 0x40);  // CONFIG_1: Single Sto
  writeSPI(PIN_CSB_TDC1000, 0x02, 0x20);  // CONFIG_2: Damping ON
  writeSPI(PIN_CSB_TDC1000, 0x03, 0x03);  // Threshold trung bình (-125mV)
  writeSPI(PIN_CSB_TDC1000, 0x05, 0x00);  // Gain 0dB
  writeSPI(PIN_CSB_TDC1000, 0x07, 0x03);  // Xóa lỗi cũ
  Serial.println("-> TDC1000 Configured.");

  // CẤU HÌNH TDC7200 (10 chu kỳ Calib)
  writeSPI(PIN_CSB_TDC7200, 0x01, 0x40);  // CONFIG2: 10 Calib cycles
  writeSPI(PIN_CSB_TDC7200, 0x00, 0x01);  // Mode 1
  writeSPI(PIN_CSB_TDC7200, 0x03, 0x10);  // Enable MEAS_COMPLETE interrupt
  Serial.println("-> TDC7200 Configured.");
  Serial.println("--- CẤU HÌNH XONG ---");

  lcd.setCursor(0, 1);
  lcd.print("TDC READY!");
  delay(1000);
  lcd.clear();
  Serial.println("=== HỆ THỐNG SẴN SÀNG ĐO ===");
}

// LOOP
void loop() {
  static unsigned long lastUpdate = 0;
  // Lấy dữ liệu ToF ảo (đơn vị ns)
  float tof_up = performMeasurement(true);     // Chiều ngược dòng
  float tof_down = performMeasurement(false);  // Chiều xuôi dòng

  // Đổi nanoseconds -> seconds
  float t_up = tof_up * 1e-9;
  float t_down = tof_down * 1e-9;
  if (millis() - lastUpdate >= 2000) {
    if (measurementValid()) {                                       // SỬ DỤNG HÀM KIỂM TRA TÍNH HỢP LỆ
      velocity = (L / 2.0) * ((1.0 / t_down) - (1.0 / t_up));       // V = (L/2) * (1/t_down - 1/t_up)
      flow_m3h = velocity * (PI_VAL * pow((D / 2.0), 2)) * 3600.0;  // Q = V * AREA * 3600 (giây)
      updateLCD();
      Serial.printf("V: %.4f m/s, Q: %.4f m3/h\n", velocity, flow_m3h);
    } else {
      lcd.setCursor(0, 0);
      lcd.print("MEASURE FAILED!");
      Serial.println("Lỗi");
    }
    lastUpdate = millis();
  }

  // Gửi Firebase mỗi 2 giây
  if (Firebase.ready() && (millis() - lastFirebaseSendMs > 2000)) {
    lastFirebaseSendMs = millis();
    sendToFirebase();
  }
}