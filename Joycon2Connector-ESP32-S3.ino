#include "device/usbd_pvt.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cstdint>
#include <functional>
#include <string>
#include <tusb.h>
#include <vector>

// USB CDC (Serial) の有効/無効切り替え
// true: XInput + Serial (Composite Device)
// false: XInput Only (Single Device)
#define CDC_ENABLED false

#if !CDC_ENABLED
#define Serial Serial_dummy
class DummySerial {
public:
  void begin(...) {}
  void print(...) {}
  void println(...) {}
  void printf(...) {}
  operator bool() { return true; }
};
static DummySerial Serial_dummy;
#endif

// BLE関連の定数
const uint16_t TARGET_MANUFACTURER_ID = 0x0553;
const char *SERVICE_UUID = "AB7DE9BE-89FE-49AD-828F-118F09DF7FD0";
const char *WRITE_CHARACTERISTIC_UUID = "649d4ac9-8eb7-4e6c-af44-1ea54fe5f005";
const char *SUBSCRIBE_CHARACTERISTIC_UUID =
    "ab7de9be-89fe-49ad-828f-118f09df7fd2";

// BLE関連のグローバル変数
static NimBLEAdvertisedDevice *pServerDevice = nullptr;
static NimBLERemoteCharacteristic *pWriteCharacteristic = nullptr;
static NimBLERemoteCharacteristic *pNotifyCharacteristic = nullptr;
static NimBLEClient *pClient = nullptr;
static boolean doConnect = false;
static boolean isConnected = false;

// 接続完了後に送信するコマンド
const uint8_t write_command1[] = {0x0c, 0x91, 0x01, 0x02, 0x00, 0x04,
                                  0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
const uint8_t write_command2[] = {0x0c, 0x91, 0x01, 0x04, 0x00, 0x04,
                                  0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};

// デバイスディスクリプタ:
const tusb_desc_device_t xinputDeviceDescriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,

#if CDC_ENABLED
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
#else
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
#endif

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = 0x045E,  // Microsoft
    .idProduct = 0x028E, // Xbox 360 Controller
    .bcdDevice = 0x0572,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01};

const uint8_t xinputConfigurationDescriptor[] = {
    // Configuration Descriptor:
    0x09, // bLength
    0x02, // bDescriptorType
#if CDC_ENABLED
    0x72, 0x00, // wTotalLength   (114 bytes)
    0x03,       // bNumInterfaces (1 XInput + 2 CDC)
#else
    0x30, 0x00, // wTotalLength   (48 bytes)
    0x01,       // bNumInterfaces (1 XInput)
#endif
    0x01, // bConfigurationValue
    0x00, // iConfiguration
    0x80, // bmAttributes   (Bus-powered Device)
    0xFA, // bMaxPower      (500 mA)

    // --- XInput Interface (Interface 0) ---
    // Interface Descriptor:
    0x09, // bLength
    0x04, // bDescriptorType
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x02, // bNumEndPoints
    0xFF, // bInterfaceClass      (Vendor specific)
    0x5D, // bInterfaceSubClass
    0x01, // bInterfaceProtocol
    0x00, // iInterface

    // XInput Specific Descriptor:
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x81, 0x14, 0x03, 0x00, 0x03, 0x13,
    0x02, 0x00, 0x03, 0x00,

    // Endpoint Descriptor: IN
    0x07,       // bLength
    0x05,       // bDescriptorType
    0x81,       // bEndpointAddress  (IN endpoint 1)
    0x03,       // bmAttributes      (Transfer: Interrupt)
    0x20, 0x00, // wMaxPacketSize    (32 bytes)
    0x04,       // bInterval         (4 ms)

    // Endpoint Descriptor: OUT
    0x07,       // bLength
    0x05,       // bDescriptorType
    0x02,       // bEndpointAddress  (OUT endpoint 2)
    0x03,       // bmAttributes      (Transfer: Interrupt)
    0x20, 0x00, // wMaxPacketSize    (32 bytes)
    0x08,       // bInterval         (8 ms)

#if CDC_ENABLED
    // --- CDC Interface (Interface 1 & 2) ---
    // IAD
    0x08, 0x0B, 0x01, 0x02, 0x02, 0x02, 0x01, 0x00,
    // Interface 1: Control
    0x09, 0x04, 0x01, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    // Header Functional
    0x05, 0x24, 0x00, 0x10, 0x01,
    // Call Management Functional
    0x05, 0x24, 0x01, 0x00, 0x02,
    // ACM Functional
    0x04, 0x24, 0x02, 0x02,
    // Union Functional
    0x05, 0x24, 0x06, 0x01, 0x02,
    // Endpoint 3: Notification (Interrupt IN)
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x10,
    // Interface 2: Data
    0x09, 0x04, 0x02, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    // Endpoint 4: Data OUT (Bulk)
    0x07, 0x05, 0x04, 0x02, 0x40, 0x00, 0x00,
    // Endpoint 4: Data IN (Bulk)
    0x07, 0x05, 0x84, 0x02, 0x40, 0x00, 0x00
#endif
};

char const *string_desc_arr_xinput[] = {
    (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "GENERIC",                  // 1: Manufacturer
    "XINPUT CONTROLLER",        // 2: Product
    "1.0",                      // 3: Serials
    "XInput Interface",         // 4: XInput string (if needed)
    "CDC Serial"                // 5: CDC Interface
};

// 1つの軸に対するキャリブレーション値を保持する構造体
struct AxisCalibration {
  int32_t adc_min;
  int32_t adc_max;
  int32_t dead_zone;
};

// TinyUSB HID callbacks
const uint8_t *tud_descriptor_device_cb(void) {
  return (const uint8_t *)(&xinputDeviceDescriptor);
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
  return xinputConfigurationDescriptor;
}
const uint8_t *tud_hid_descriptor_report_cb(uint8_t instance) //
{
  return (const uint8_t *)(&xinputDeviceDescriptor);
}

// 文字列ディスクリプタのコールバック
static uint16_t _desc_str[32];
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  uint8_t chr_count;
  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr_xinput[0], 2);
    chr_count = 1;
  } else {
    // Convert ASCII string into UTF-16

    if (!(index <
          sizeof(string_desc_arr_xinput) / sizeof(string_desc_arr_xinput[0])))
      return NULL;

    const char *str = string_desc_arr_xinput[index];

    // Cap at max char
    chr_count = strlen(str);
    if (chr_count > 31)
      chr_count = 31;

    for (uint8_t i = 0; i < chr_count; ++i) {
      _desc_str[1 + i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

  return _desc_str;
}

// Report

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t rid;
  uint8_t rsize;
  uint8_t digital_buttons_1;
  uint8_t digital_buttons_2;
  uint8_t lt;
  uint8_t rt;
  int16_t l_x;
  int16_t l_y;
  int16_t r_x;
  int16_t r_y;
  uint8_t reserved_1[6];
} ReportDataXinput;
ReportDataXinput XboxButtonData;     // data to send
ReportDataXinput prevXboxButtonData; // data sent last time

bool inputChanged() {
  return memcmp(&XboxButtonData, &prevXboxButtonData, sizeof(XboxButtonData)) !=
         0;
}

const uint8_t endpoint_in = 0x81;
void sendReportData() {
  if (!inputChanged()) {
    return;
  }

  // Remote wakeup
  if (tud_suspended()) {
    tud_remote_wakeup();
  }

  XboxButtonData.rid = 0;
  XboxButtonData.rsize = 20;
  for (int8_t i = 0; i < 6; ++i)
    XboxButtonData.reserved_1[i] = 0;

  if (tud_ready() && !usbd_edpt_busy(0, endpoint_in)) {
    usbd_edpt_claim(0, endpoint_in);
    usbd_edpt_xfer(0, endpoint_in, (uint8_t *)&XboxButtonData, 20);
    usbd_edpt_release(0, endpoint_in);

    prevXboxButtonData = XboxButtonData;
  }
}

static void xinput_init() {}

static void xinput_reset(uint8_t rhport) {}

static uint16_t xinput_open(uint8_t rhport,
                            tusb_desc_interface_t const *itf_desc,
                            uint16_t max_len) {
  //+16 is for the unknown descriptor
  const uint16_t drv_len =
      sizeof(tusb_desc_interface_t) +
      itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t) + 16;
  TU_VERIFY(max_len >= drv_len, 0);

  const uint8_t *p_desc = tu_desc_next(itf_desc);
  uint8_t found_endpoints = 0;
  while (found_endpoints < itf_desc->bNumEndpoints) {
    const tusb_desc_endpoint_t *desc_ep = (const tusb_desc_endpoint_t *)p_desc;
    if (TUSB_DESC_ENDPOINT == tu_desc_type(desc_ep)) {
      TU_ASSERT(usbd_edpt_open(rhport, desc_ep));
      found_endpoints += 1;
    }
    p_desc = tu_desc_next(p_desc);
  }

  return drv_len;
}

// BLEデータからXInputレポートへの変換
/**
 * @brief 12bitのアナログ値をXInput用のint16_t(-32768 to 32767)に変換
 */

int16_t map_stick_axis_xinput(uint16_t val, const AxisCalibration &cal) {
  // 中心点はキャリブレーション値から動的に計算
  const int32_t ADC_CENTER = (cal.adc_max + cal.adc_min) / 2;
  const int32_t XINPUT_MIN = -32768;
  const int32_t XINPUT_MAX = 32767;

  int32_t temp = static_cast<int32_t>(val) - ADC_CENTER;

  // デッドゾーン処理
  if (abs(temp) <= cal.dead_zone) {
    return 0;
  }

  // スケール変換 (マッピング)
  int32_t mapped_value;
  if (temp > 0) {
    // 観測した最大値を使ってマッピング
    mapped_value =
        map(temp, cal.dead_zone + 1, cal.adc_max - ADC_CENTER, 1, XINPUT_MAX);
  } else {
    // 観測した最小値を使ってマッピング
    mapped_value =
        map(temp, cal.adc_min - ADC_CENTER, -cal.dead_zone - 1, XINPUT_MIN, -1);
  }
  return static_cast<int16_t>(constrain(mapped_value, XINPUT_MIN, XINPUT_MAX));
}

void parse_stick(const uint8_t *data, uint16_t &x, uint16_t &y) {
  uint32_t val = data[0] | (data[1] << 8) | (data[2] << 16);
  x = val & 0xFFF;
  y = (val >> 12) & 0xFFF;
}

// --- アナログトリガーのキャリブレーション値定義 ---
// 左トリガー (Left Trigger)
const uint8_t LT_MIN = 37;
const uint8_t LT_MAX = 190;

// 右トリガー (Right Trigger)
const uint8_t RT_MIN = 30;
const uint8_t RT_MAX = 190;

uint8_t parse_trigger(const uint8_t raw_trigger, uint8_t min, uint8_t max) {
  uint8_t constrained = constrain(raw_trigger, min, max);
  return map(constrained, min, max, 0, 255);
}

const int32_t COMMON_DEAD_ZONE = 10; // デッドゾーンは共通と仮定

// --- Left Stick ---
// X軸 (右が最大、左が最小)
const AxisCalibration left_stick_x_cal = {
    .adc_min = 905, .adc_max = 3340, .dead_zone = COMMON_DEAD_ZONE};
// Y軸 (上が最大、下が最小)
// 元の値: min=870, max=3290
// 反転後(4095-val)の値: min=4095-3290=805, max=4095-870=3225
const AxisCalibration left_stick_y_cal = {
    .adc_min = 805,  // 4095 - 3290 (元のmax)
    .adc_max = 3225, // 4095 - 870  (元のmin)
    .dead_zone = COMMON_DEAD_ZONE};

// --- Right Stick ---
// X軸 (右が最大、左が最小)
const AxisCalibration right_stick_x_cal = {
    .adc_min = 926, .adc_max = 3180, .dead_zone = COMMON_DEAD_ZONE};
// Y軸 (上が最大、下が最小)
// 元の値: min=950, max=3160
// 反転後(4095-val)の値: min=4095-3160=935, max=4095-950=3145
const AxisCalibration right_stick_y_cal = {
    .adc_min = 935,  // 4095 - 3160 (元のmax)
    .adc_max = 3145, // 4095 - 950  (元のmin)
    .dead_zone = COMMON_DEAD_ZONE};

// 受信したデータを解析し、XInputレポートを更新する
void update_gamepad_report(const uint8_t *data, size_t length) {
  if (length < 62) { // データ長が期待値より短い場合は無視
    Serial.println("Received data too short!");
    return;
  }

  // ボタン情報のパース (オフセット 0x03, 4バイト)
  uint32_t buttons;
  memcpy(&buttons, &data[0x03], sizeof(buttons));

  Serial.println(buttons);

  uint8_t db1 = 0, db2 = 0;
  // digital_buttons_1
  if (buttons & 0x02000000)
    db1 |= 0x01; // DPAD_UP
  if (buttons & 0x01000000)
    db1 |= 0x02; // DPAD_DOWN
  if (buttons & 0x08000000)
    db1 |= 0x04; // DPAD_LEFT
  if (buttons & 0x04000000)
    db1 |= 0x08; // DPAD_RIGHT

  if (buttons & 0x00020000)
    db1 |= 0x10; // START
  if (buttons & 0x00010000)
    db1 |= 0x20; // BACK (SELECT)
  if (buttons & 0x00080000)
    db1 |= 0x40; // LEFT_STICK
  if (buttons & 0x00040000)
    db1 |= 0x80; // RIGHT_STICK

  // digital_buttons_2
  if (buttons & 0x80000000)
    db2 |= 0x01; // LEFT_BUMPER (L)
  if (buttons & 0x00008000)
    db2 |= 0x02; // RIGHT_BUMPER (R)
  if (buttons & 0x00100000)
    db2 |= 0x04; // GUIDE (HOME)
  if (buttons & 0x00000800)
    db2 |= 0x10; // A
  if (buttons & 0x00000400)
    db2 |= 0x20; // B
  if (buttons & 0x00000200)
    db2 |= 0x40; // X
  if (buttons & 0x00000100)
    db2 |= 0x80; // Y

  XboxButtonData.digital_buttons_1 = db1;
  XboxButtonData.digital_buttons_2 = db2;

  // アナログトリガー情報のパース (オフセット 0x3C, 0x3D)
  XboxButtonData.lt = parse_trigger(data[0x3C], LT_MIN, LT_MAX);
  XboxButtonData.rt = parse_trigger(data[0x3D], RT_MIN, RT_MAX);

  // L/Rがデジタルの場合、アナログトリガー値を上書き
  if (buttons & 0x40000000)
    XboxButtonData.lt = 255;
  if (buttons & 0x00004000)
    XboxButtonData.rt = 255;

  // SL,SR (L)
  if (buttons & 0x20000000)
    XboxButtonData.lt = 255;
  if (buttons & 0x10000000)
    XboxButtonData.rt = 255;

  // SL,SR (R)
  if (buttons & 0x00002000)
    XboxButtonData.lt = 255;
  if (buttons & 0x00001000)
    XboxButtonData.rt = 255;

  // スティック情報のパース
  uint16_t lx, ly, rx, ry;
  parse_stick(&data[0x0A], lx, ly); // Left Stick
  parse_stick(&data[0x0D], rx, ry); // Right Stick

  XboxButtonData.l_x = map_stick_axis_xinput(lx, left_stick_x_cal);
  XboxButtonData.l_y =
      map_stick_axis_xinput(4095 - ly, left_stick_y_cal); // Y軸反転
  XboxButtonData.r_x = map_stick_axis_xinput(rx, right_stick_x_cal);
  XboxButtonData.r_y =
      map_stick_axis_xinput(4095 - ry, right_stick_y_cal); // Y軸反転
}

// BLE関連コールバック/クライアント処理
void notifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic,
                    uint8_t *pData, size_t length, bool isNotify) {
  update_gamepad_report(pData, length);
}

class ClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *pclient) {
    Serial.println("Connected to BLE server");
    isConnected = true;
  }
  void onDisconnect(NimBLEClient *pclient) {
    isConnected = false;
    pServerDevice = nullptr;
    Serial.println("Disconnected from BLE server");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(pServerDevice->getAddress().toString().c_str());

  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(new ClientCallback());
  pClient->connect(pServerDevice);

  NimBLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    pRemoteService =
        pClient->getService(NimBLEUUID(pServerDevice->getServiceUUID(0)));
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(SERVICE_UUID);
      pClient->disconnect();
      return false;
    }
  }
  Serial.println(" - Found our service");

  pWriteCharacteristic =
      pRemoteService->getCharacteristic(WRITE_CHARACTERISTIC_UUID);
  pNotifyCharacteristic =
      pRemoteService->getCharacteristic(SUBSCRIBE_CHARACTERISTIC_UUID);

  if (pWriteCharacteristic == nullptr || pNotifyCharacteristic == nullptr) {
    Serial.println("Failed to find one of the characteristics");
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristics");

  if (pNotifyCharacteristic->canNotify()) {
    pNotifyCharacteristic->subscribe(
        true, (NimBLERemoteCharacteristic::notify_callback)notifyCallback);
    // CCCD(0x2902)を有効にする
    NimBLERemoteDescriptor *pDescriptor =
        pNotifyCharacteristic->getDescriptor(NimBLEUUID((uint16_t)0x2902));
    if (pDescriptor) {
      uint8_t val[] = {0x01, 0x00};
      pDescriptor->writeValue(val, (uint16_t)2, true);
      Serial.println(" - Registered for notifications");
    }
  }

  delay(500);
  Serial.println("Sending init commands...");
  pWriteCharacteristic->writeValue((const uint8_t *)write_command1,
                                   (size_t)sizeof(write_command1), true);
  delay(500);
  pWriteCharacteristic->writeValue((const uint8_t *)write_command2,
                                   (size_t)sizeof(write_command2), true);
  Serial.println("Init commands sent.");

  return true;
}

class ScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    if (advertisedDevice->haveManufacturerData()) {
      std::string mData = advertisedDevice->getManufacturerData(0);
      // Manufacturer IDは通常、データの最初の2バイト
      if (mData.length() >= 2) {
        uint16_t manufacturerId =
            (uint16_t)((uint8_t)mData[0] | ((uint8_t)mData[1] << 8));
        if (manufacturerId == TARGET_MANUFACTURER_ID) {
          Serial.printf("*** Found our device! Manufacturer ID: 0x%04X ***\n",
                        manufacturerId);
          NimBLEDevice::getScan()->stop();
          pServerDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
          doConnect = true;
        }
      }
    }
  }
};

static bool xinput_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                   tusb_control_request_t const *request) {
  return true;
}

static bool xinput_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                           xfer_result_t result, uint32_t xferred_bytes) {
  return true;
}

static const usbd_class_driver_t xinput_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "XINPUT",
#endif
    .init = xinput_init,
    .reset = xinput_reset,
    .open = xinput_open,
    .control_xfer_cb = xinput_control_xfer_cb,
    .xfer_cb = xinput_xfer_cb,
    .sof = NULL};

// TinyUSB内部のCDC関数を外部宣言
extern "C" {
void cdcd_init(void);
void cdcd_reset(uint8_t rhport);
uint16_t cdcd_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                   uint16_t max_len);
bool cdcd_control_xfer_cb(uint8_t rhport, uint8_t stage,
                          tusb_control_request_t const *request);
bool cdcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                  uint32_t xferred_bytes);
}

static const usbd_class_driver_t my_cdc_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "CDC",
#endif
    .init = cdcd_init,
    .reset = cdcd_reset,
    .open = cdcd_open,
    .control_xfer_cb = cdcd_control_xfer_cb,
    .xfer_cb = cdcd_xfer_cb,
    .sof = NULL};

const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count) {
#if CDC_ENABLED
  static usbd_class_driver_t const drivers[] = {xinput_driver, my_cdc_driver};
#else
  static usbd_class_driver_t const drivers[] = {xinput_driver};
#endif
  *driver_count = sizeof(drivers) / sizeof(drivers[0]);
  return drivers;
}
void resetReportStruct() {
  memset(&XboxButtonData, 0, sizeof(XboxButtonData));
  XboxButtonData.rid = 0;
  XboxButtonData.rsize = 20;
}

void setup() {
  // シリアル通信の初期化
  Serial.begin(115200);

  pinMode(0, INPUT_PULLUP);

  // XInputレポートの初期化
  resetReportStruct();
  prevXboxButtonData = XboxButtonData;

  // TinyUSBの初期化
  tusb_init();

  // BLEの初期化とペアリング設定
  NimBLEDevice::init("");
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  NimBLEScan *pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(new ScanCallbacks(),
                             false); // Delete callbacks when done
  pBLEScan->setInterval(400);
  pBLEScan->setWindow(200);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, false);
  Serial.println("BLE Scan started...");
}

void loop() {
  tud_task_ext(0, false);
  // BLE接続処理
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Successfully connected to the BLE Server.");
    } else {
      Serial.println("Failed to connect to the server.");
    }
    doConnect = false;
  }

  // BLE接続中はレポートを送信、切断されたら再スキャン
  if (isConnected) {
    // 4msごとにレポートを送信するポーリング
    const unsigned long interval_ms = 4;
    static unsigned long last_send_ms = 0;
    if (millis() - last_send_ms >= interval_ms) {
      last_send_ms = millis();
      sendReportData();
    }
  } else {
    // 接続が切れたら再スキャンを開始
    if (pServerDevice == nullptr && !NimBLEDevice::getScan()->isScanning()) {
      Serial.println("Disconnected. Restarting scan...");
      NimBLEDevice::getScan()->start(0, false);
    }
  }

  delay(1);
}