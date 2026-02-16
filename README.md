# Joycon2Connector-ESP32-S3

English | [日本語](#joycon2connector-esp32-s3-日本語)

A program to make AtomS3U (ESP32-S3) function as a receiver for Joy-Con 2 and GameCube controllers for Nintendo Switch 2, allowing them to be recognized as USB XInput devices on PCs and other platforms.

## Overview

- Leverages the USB OTG and BLE functions of the AtomS3U to handle Joy-Con 2 BLE communication, enabling its use as an XInput controller.
- Developed specifically for Joy-Con 2, which utilizes BLE and lacks simple pairing. On macOS, accessing CoreHID requires an Apple Developer Program (ADP) subscription, making an external device-based implementation the most cost-effective solution.

## Status

This project is **under active (leisurely) development**.

### Verified Devices
Currently, only the **GameCube Controller for Switch 2** has been confirmed to work. Operation with standard Joy-Con 2 controllers is unverified.

### Verified OS/Platforms
- macOS
    - Google Chrome, Dolphin
- iOS
    - Safari, PPSSPP

## Key Features

- **BLE Connection**: Supports wireless connection with Joy-Con 2.
- **XInput Controller Operation**: Can be used directly as a gamepad on PCs, etc.
- **Button/Stick/Trigger Parsing for GC Controller**: At least the GameCube controller works fully (except for the gyro).
- **Serial CDC (Optional)**: USB serial communication can be enabled or disabled via settings.
    - Note: Serial communication is currently unstable.
- **Auto Re-scan**: Automatically searches for the connection target upon disconnection.

## Requirements

### Hardware
- **ESP32-S3**: Native USB (USB-OTG) functionality is required. It will not work on standard ESP32 (1st generation) or ESP32-C3.
- **Joy-Con 2 Series Controllers**: Those designed for Nintendo Switch 2, including the GC controller.

### Software/Libraries
The following libraries must be installed in your Arduino environment:
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32) (using TinyUSB stack)

## Installation and Flashing

1. After cloning the repository, open `Joycon2Connector-ESP32-S3.ino` in the Arduino IDE.
2. Select `ESP32S3 Dev Module` in the Board Manager.
3. Set **USB Mode** to `Hardware CDC and JTAG` or `USB-OTG (TinyUSB)`.
4. Build and flash the program.

## Credits & References

The following repositories were referenced during the development of this project.

- [TheFrano/joycon2py](https://github.com/TheFrano/joycon2py)
- [TheFrano/joycon2cpp](https://github.com/TheFrano/joycon2cpp)

## License

This project is released under the [MIT License](LICENSE).
Copyright (c) 2026 mikuta0407

---

# Joycon2Connector-ESP32-S3 (日本語)

[English](#joycon2connector-esp32-s3) | 日本語

AtomS3U (ESP32-S3) を、Nintendo Switch 2向けのJoy-Con 2・ゲームキューブコントローラーのレシーバーとして動作させ、USB XInputデバイスとしてPC等に認識させて使うためのプログラムです。

## 概要

- AtomS3UのUSB OTG機能とBLE機能を活用し、Joy-Con 2のBLE通信をハンドリングし、XInputコントローラとして使えるようにします。
- Joy-Conと異なりBLEとなり、簡単なペアリングが不可能になったJoy-Con2を使うために作りました。特にmacOSではCoreHIDを使うためにはADPに加入する必要があるため、外部デバイスによる実装が最安です。

## ステータス

このプロジェクトは **のんびりと開発中** です。

### 動作確認済みデバイス
現時点で動作が確認できているのは **Switch 2向けゲームキューブコントローラー** のみです。通常のJoy-Con 2の動作は未検証です。

### 動作確認済みOS/プラットフォーム
- macOS
    - Google Chrome, Dolphin
- iOS
    - Safari, PPSSPP

## 主な機能

- **BLEでの接続**: Joy-Con 2とのワイヤレス接続に対応。
- **XInputコントローラーとしての動作**: PCなどでそのままゲームパッドとして利用可能。
- **ゲームキューブコントローラーの各ボタン・スティック・トリガーのパース**: 少なくともGCコンはジャイロ以外は動きます。
- **Serial CDC (オプション)**: 設定によりUSBシリアル通信を有効化または無効化可能
    - 正直シリアル通信うまく動きません。なんで?
- **自動再スキャン**: 切断時に自動で接続先を探します。

## 必要要件

### ハードウェア
- **ESP32-S3**: ネイティブUSB (USB-OTG) 機能が必要です。通常のESP32（第1世代）やESP32-C3などでは動作しません。
- **Joy-Con 2系コントローラー**: Nintendo Switch 2向けのもの。GCコンも含む。

### ソフトウェア・ライブラリ
以下のライブラリがArduino環境にインストールされている必要があります。
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- ESP32 Arduino Core (TinyUSBスタックを使用)

## インストールと書き込み

1. git clone後、Arduino IDEで `Joycon2Connector-ESP32-S3.ino` を開きます。
2. ボードマネージャーで `ESP32S3 Dev Module` を選択します。
3. **USB Mode** を `Hardware CDC and JTAG` または `USB-OTG (TinyUSB)` に設定してください。
4. プログラムをビルドして書き込みます。

## クレジット・参考

このプロジェクトの開発にあたり、以下のリポジトリを参考にさせていただきました。

- [TheFrano/joycon2py](https://github.com/TheFrano/joycon2py)
- [TheFrano/joycon2cpp](https://github.com/TheFrano/joycon2cpp)

## ライセンス

このプロジェクトは [MIT ライセンス](LICENSE) の下で公開されています。
Copyright (c) 2026 mikuta0407
