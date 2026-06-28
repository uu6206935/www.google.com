#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <cstdint>
#include <sstream>
#include <utility>
#include <mutex>
#include <fstream>
#include <conio.h> // コンソール入出力用

// --- Windows & デバイス関連ヘッダー ---
#define NOMINMAX
#include <windows.h>
#include <XInput.h>
#pragma comment(lib, "xinput.lib")
#include "LabJackUD.h"

// DirectInput (ハンコン) 用ヘッダー
#include <dinput.h>
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")

// --- 共有メモリ関連のグローバル変数 ---
HANDLE hFM_SharedData = nullptr;
double* sm = nullptr;

// --- 共有メモリ関連の関数プロトタイプ ---
bool init_shared_memory();
void shutdown_shared_memory();

// --- DirectInput 関連の関数プロトタイプ ---
bool init_directinput();
void shutdown_directinput();
void poll_dfp();

// --- 基本設定値 ---
const wchar_t* SERIAL_PORT_STEERING = L"\\\\.\\COM5";
const wchar_t* SERIAL_PORT_SPEED = L"\\\\.\\COM7";
const wchar_t* SERIAL_PORT_GNSS = L"\\\\.\\COM24";
const int BAUDRATE_STEERING = 115200;
const int BAUDRATE_SPEED = 115200;
const int BAUDRATE_GNSS = 230400;
const double MOTOR_FORWARD_V = 5.0;
const double MOTOR_REVERSE_V = 0.0;
const double MOTOR_STOP_V = 2.5;
const double TOLERANCE_V = 0.03;
const double RT_UNPRESSED_V = 0.8;
const double POT_MIN_V = 0.0;     // ブレーキ解放時にポテンショ(AIN0)が示す実測電圧(≒0V)に合わせる
const double POT_MAX_V = 0.748;
const double RT_PRESSED_MIN_V = 1.5;
const double RT_PRESSED_MAX_V = 3.4;
// これ以下のブレーキ指令は「ブレーキ解放」とみなし、ブレーキモーターを停止させる。
// 何も操作していない状態(brake_norm≒0)で到達不能な目標を追い続けてモーターが
// カタカタ鳴るのを防ぐためのデッドゾーン。
const double BRAKE_RELEASE_THRESHOLD = 0.02;
const int32_t MAX_STEERING_STEP = 24000;
const uint32_t MAX_STEERING_SPEED = 0x00010000; // 自動運転モード時の最大操舵速度

// 手動運転モード（Gamepad）用の操舵速度を定義
const uint32_t MANUAL_STEERING_SPEED = 0x00002800; // 手動運転モード時の操舵速度 (最大より遅く設定)
const double THROTTLE_POWER_CURVE = 3.0;
const std::chrono::milliseconds LOOP_INTERVAL(50);

const double PI = 3.141592653589793;
const double EARTH_RADIUS_M = 6378137.0; // 地球の半径（メートル）
const double BASE_LAT = 35.5465763;      // 原点緯度
const double BASE_LON = 139.6720388;     // 原点経度

// --- Thread-safe Global Variables ---
std::atomic<double> g_steering_input_serial(0.0);
std::atomic<double> g_vehicle_speed_kmh(0.0);
std::atomic<bool> g_reverse_active(false);
std::atomic<bool> g_running(true);
std::atomic<double> g_actual_voltage_pot(POT_MIN_V);
std::atomic<bool> g_is_recording(false);
std::atomic<bool> g_reconnect_requested(false);
std::atomic<bool> g_shared_memory_control(false);
std::atomic<double> g_gnss_lat(0.0);
std::atomic<double> g_gnss_lon(0.0);
std::atomic<double> g_gnss_accuracy(99.9);
std::atomic<double> g_gnss_x(0.0);
std::atomic<double> g_gnss_y(0.0);
std::atomic<bool> g_keyboard_throttle_mode(false);
std::atomic<double> g_throttle_input_norm(0.0);
std::atomic<double> g_brake_input_norm(0.0);

// --- ブレーキ制御の診断用（ダッシュボード表示） ---
// 実際にブレーキアクチュエータを動かしているのは sm[224](目標ブレーキ強度) であり、
// ゲームパッドのトリガ(g_brake_input_norm)ではない。原因切り分けのため実値を表示する。
std::atomic<double> g_brake_cmd(0.0);            // 実際に使われたブレーキ指令 (sm[224] をクランプした値)
std::atomic<double> g_brake_target_v(0.0);       // 目標ポテンショ電圧
std::atomic<double> g_brake_motor_v(MOTOR_STOP_V); // ブレーキモーターへの出力(DAC1)

// --- DFP/XInput 優先権管理 ---
enum class InputDevicePriority {
    None,
    XInput,
    DirectInput
};
std::atomic<InputDevicePriority> g_input_priority(InputDevicePriority::None);


// --- Device Handles & Status Flags ---
HANDLE hSerialSteering = INVALID_HANDLE_VALUE;
HANDLE hSerialSpeed = INVALID_HANDLE_VALUE;
HANDLE hSerialGNSS = INVALID_HANDLE_VALUE;
LJ_HANDLE ljHandle = 0;
std::atomic<bool> steering_ok(false);
std::atomic<bool> m5atom_ok(false);
std::atomic<bool> gnss_ok(false);
std::atomic<bool> labjack_ok(false);
std::atomic<bool> joystick_ok(false); // XInput Gamepad

// --- DFP (DirectInput) 用ハンドル ---
LPDIRECTINPUT8       g_pDI = nullptr;
LPDIRECTINPUTDEVICE8 g_pDFP = nullptr;
std::atomic<bool> g_dfp_ok(false); // DFP Gamepad

// --- 共有メモリ初期化/終了関数 ---
bool init_shared_memory() {
    hFM_SharedData = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, L"SHAREDATA");
    if (hFM_SharedData == NULL) {
        std::cerr << "[WARN] Could not open shared memory 'SHAREDATA'. Creating new one." << std::endl;
        hFM_SharedData = CreateFileMapping(
            INVALID_HANDLE_VALUE,
            NULL,
            PAGE_READWRITE,
            0,
            sizeof(double) * 2000,
            L"SHAREDATA");
    }

    if (hFM_SharedData == NULL) {
        std::cerr << "[FATAL] Could not create or open shared memory 'SHAREDATA'. Error: " << GetLastError() << std::endl;
        return false;
    }

    sm = (double*)MapViewOfFile(hFM_SharedData, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(double) * 2000);
    if (sm == nullptr) {
        std::cerr << "[FATAL] Could not map view of file 'SHAREDATA'. Error: " << GetLastError() << std::endl;
        CloseHandle(hFM_SharedData);
        return false;
    }

    std::cout << "[INFO] Shared memory 'SHAREDATA' connected successfully." << std::endl;
    return true;
}

void shutdown_shared_memory() {
    if (sm != nullptr) {
        UnmapViewOfFile(sm);
        sm = nullptr;
    }
    if (hFM_SharedData != nullptr) {
        CloseHandle(hFM_SharedData);
        hFM_SharedData = nullptr;
    }
    std::cout << "[INFO] Shared memory disconnected." << std::endl;
}

// --- ハードを動かすための下層レイヤ ---
uint16_t calculate_crc16_modbus(const std::vector<unsigned char>& data) { uint16_t crc = 0xFFFF; for (unsigned char byte : data) { crc ^= byte; for (int i = 0; i < 8; ++i) { if (crc & 1) { crc = (crc >> 1) ^ 0xA001; } else { crc >>= 1; } } } return crc; }
void pack_be(std::vector<unsigned char>& buffer, uint16_t value) { buffer.push_back((value >> 8) & 0xFF); buffer.push_back(value & 0xFF); }
void pack_be(std::vector<unsigned char>& buffer, int32_t value) { buffer.push_back((value >> 24) & 0xFF); buffer.push_back((value >> 16) & 0xFF); buffer.push_back((value >> 8) & 0xFF); buffer.push_back(value & 0xFF); }
void pack_be(std::vector<unsigned char>& buffer, uint32_t value) { buffer.push_back((value >> 24) & 0xFF); buffer.push_back((value >> 16) & 0xFF); buffer.push_back((value >> 8) & 0xFF); buffer.push_back(value & 0xFF); }
void pack_le(std::vector<unsigned char>& buffer, uint16_t value) { buffer.push_back(value & 0xFF); buffer.push_back((value >> 8) & 0xFF); }
std::vector<unsigned char> create_motor_command(double jx, uint32_t speed, int32_t max_step) { int32_t motorstep2 = static_cast<int32_t>(max_step * (-jx)); std::vector<unsigned char> payload = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }; pack_be(payload, motorstep2); pack_be(payload, speed); pack_be(payload, (uint32_t)0x00061A80); pack_be(payload, (uint32_t)0x00061A80); payload.insert(payload.end(), { 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01 }); std::vector<unsigned char> crc_source_data = { 0x02, 0x10 }; pack_be(crc_source_data, (uint16_t)0x0058); pack_be(crc_source_data, (uint16_t)0x0010); crc_source_data.push_back(0x20); crc_source_data.insert(crc_source_data.end(), payload.begin(), payload.end()); uint16_t crc = calculate_crc16_modbus(crc_source_data); std::vector<unsigned char> command_frame = crc_source_data; pack_le(command_frame, crc); return command_frame; }
double map_value(double x, double in_min, double in_max, double out_min, double out_max) { if ((in_max - in_min) == 0) return out_min; return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
void ErrorHandler(LJ_ERROR lngErrorcode, long lngLineNumber, const std::string& context) { if (lngErrorcode != LJE_NOERROR) { char err[256]; ErrorToString(lngErrorcode, err); std::cerr << "\n[FATAL] LabJack Error #" << lngErrorcode << ": \"" << err << "\"" << " in " << context << " at line " << lngLineNumber << std::endl; g_running = false; std::this_thread::sleep_for(std::chrono::milliseconds(200)); exit(lngErrorcode); } }
void set_dac_voltage(LJ_HANDLE ljHandle, long dac_num, double voltage) { if (!labjack_ok.load()) return; double safe_voltage = std::max(0.0, std::min(5.0, voltage)); ErrorHandler(eDAC(ljHandle, dac_num, safe_voltage, 0, 0, 0), __LINE__, "eDAC"); }
bool init_serial(HANDLE& hSerial, LPCWSTR port_name, int baudrate, bool is_steering = true) {
    hSerial = CreateFileW(port_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) { std::wcerr << L"[WARN] Serial: Could not open port " << port_name << L". Code: " << GetLastError() << std::endl; return false; }
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) { std::cerr << "[WARN] Serial: Could not get comm state." << std::endl; CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false; }
    dcbSerialParams.BaudRate = baudrate; dcbSerialParams.ByteSize = 8; dcbSerialParams.StopBits = ONESTOPBIT; dcbSerialParams.Parity = is_steering ? EVENPARITY : NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams)) { std::cerr << "[WARN] Serial: Could not set comm state." << std::endl; CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false; }
    COMMTIMEOUTS timeouts = { 0 };
    if (is_steering) { timeouts.WriteTotalTimeoutConstant = 50; timeouts.WriteTotalTimeoutMultiplier = 10; }
    else { timeouts.ReadIntervalTimeout = 50; timeouts.ReadTotalTimeoutConstant = 50; timeouts.ReadTotalTimeoutMultiplier = 10; timeouts.WriteTotalTimeoutConstant = 50; timeouts.WriteTotalTimeoutMultiplier = 10; }
    if (!SetCommTimeouts(hSerial, &timeouts)) { std::cerr << "[WARN] Serial: Could not set comm timeouts." << std::endl; CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false; }
    return true;
}
void write_serial_steering(const std::vector<unsigned char>& data) { if (hSerialSteering == INVALID_HANDLE_VALUE) return; DWORD bytes_written; if (!WriteFile(hSerialSteering, data.data(), static_cast<DWORD>(data.size()), &bytes_written, NULL)) { std::cerr << "\n[ERROR] Could not write to steering serial port." << std::endl; } }
void write_serial_command(const std::string& command) { if (hSerialSpeed == INVALID_HANDLE_VALUE) return; DWORD bytes_written; if (!WriteFile(hSerialSpeed, command.c_str(), static_cast<DWORD>(command.length()), &bytes_written, NULL)) { std::cerr << "\n[ERROR] Could not write command to M5Atom serial port." << std::endl; } }
void close_serial(HANDLE& hSerial) { if (hSerial != INVALID_HANDLE_VALUE) { CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; } }

std::vector<std::string> split_string(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// --- ハンコン (DFP) 初期化関連 ---
BOOL CALLBACK EnumJoysticksCallback(LPCDIDEVICEINSTANCE lpddi, LPVOID pvRef) {
    // "Driving Force Pro" を見つける
    if (wcsstr(lpddi->tszProductName, L"Driving Force Pro") != NULL) {
        if (FAILED(g_pDI->CreateDevice(lpddi->guidInstance, &g_pDFP, NULL))) {
            std::cerr << "[WARN] DInput: Failed to create device for DFP." << std::endl;
            return DIENUM_CONTINUE;
        }
        return DIENUM_STOP;
    }
    return DIENUM_CONTINUE;
}

bool init_directinput() {
    HWND hwnd = GetConsoleWindow();
    if (!hwnd) {
        std::cerr << "[WARN] DInput: Could not get console window handle." << std::endl;
        return false;
    }

    if (FAILED(DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8, (VOID**)&g_pDI, NULL))) {
        std::cerr << "[WARN] DInput: Failed to create DirectInput object." << std::endl;
        return false;
    }

    g_pDFP = nullptr;
    g_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumJoysticksCallback, NULL, DIEDFL_ATTACHEDONLY);

    if (g_pDFP == nullptr) {
        std::cerr << "[INFO] DInput: Driving Force Pro not found." << std::endl;
        g_pDI->Release();
        g_pDI = nullptr;
        return false;
    }

    if (FAILED(g_pDFP->SetDataFormat(&c_dfDIJoystick2))) {
        std::cerr << "[WARN] DInput: Failed to set data format for DFP." << std::endl;
        g_pDFP->Release();
        g_pDFP = nullptr;
        g_pDI->Release();
        g_pDI = nullptr;
        return false;
    }

    if (FAILED(g_pDFP->SetCooperativeLevel(hwnd, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE))) {
        std::cerr << "[WARN] DInput: Failed to set cooperative level for DFP." << std::endl;
        g_pDFP->Release();
        g_pDFP = nullptr;
        g_pDI->Release();
        g_pDI = nullptr;
        return false;
    }

    // DFPの軸範囲を設定 (オプションだが推奨)
    DIPROPRANGE dipr;
    dipr.diph.dwSize = sizeof(DIPROPRANGE);
    dipr.diph.dwHeaderSize = sizeof(DIPROPHEADER);
    dipr.diph.dwObj = DIJOFS_X; // ステアリング軸
    dipr.diph.dwHow = DIPH_BYOFFSET;
    dipr.lMin = -32767;
    dipr.lMax = 32767;
    g_pDFP->SetProperty(DIPROP_RANGE, &dipr.diph);

    // デッドゾーンを0に設定
    DIPROPDWORD dipdw;
    dipdw.diph.dwSize = sizeof(DIPROPDWORD);
    dipdw.diph.dwHeaderSize = sizeof(DIPROPHEADER);
    dipdw.diph.dwObj = DIJOFS_X;
    dipdw.diph.dwHow = DIPH_BYOFFSET;
    dipdw.dwData = 0; // 0%
    g_pDFP->SetProperty(DIPROP_DEADZONE, &dipdw.diph);


    if (FAILED(g_pDFP->Acquire())) {
        std::cerr << "[WARN] DInput: Failed to acquire DFP." << std::endl;
    }

    g_dfp_ok = true;
    return true;
}

void shutdown_directinput() {
    if (g_pDFP) {
        g_pDFP->Unacquire();
        g_pDFP->Release();
        g_pDFP = nullptr;
    }
    if (g_pDI) {
        g_pDI->Release();
        g_pDI = nullptr;
    }
    g_dfp_ok = false;
    g_input_priority = InputDevicePriority::None;
}


void reinitialize_devices() {
    std::cout << "\n[INFO] Attempting to (re)initialize devices..." << std::endl;

    if (steering_ok.load()) { close_serial(hSerialSteering); std::cout << "  - Closed existing Steering port." << std::endl; }
    if (m5atom_ok.load()) { close_serial(hSerialSpeed); std::cout << "  - Closed existing M5Atom port." << std::endl; }
    if (gnss_ok.load()) { close_serial(hSerialGNSS); std::cout << "  - Closed existing GNSS port." << std::endl; }
    if (labjack_ok.load()) {
        set_dac_voltage(ljHandle, 1, MOTOR_STOP_V);
        set_dac_voltage(ljHandle, 0, RT_UNPRESSED_V);
        Close();
        ljHandle = 0;
        std::cout << "  - Closed existing LabJack connection." << std::endl;
    }
    //  DFP 切断処理
    if (g_dfp_ok.load()) {
        shutdown_directinput();
        std::cout << "  - Closed existing DirectInput (DFP) connection." << std::endl;
    }

    steering_ok = false;
    m5atom_ok = false;
    gnss_ok = false;
    labjack_ok = false;
    joystick_ok = false;
    g_dfp_ok = false; // DFPステータスもリセット

    std::cout << "--- Scanning for devices ---" << std::endl;
    steering_ok = init_serial(hSerialSteering, SERIAL_PORT_STEERING, BAUDRATE_STEERING, true);
    if (steering_ok) std::wcout << L"  [OK] Steering motor port " << SERIAL_PORT_STEERING << L" opened." << std::endl;

    m5atom_ok = init_serial(hSerialSpeed, SERIAL_PORT_SPEED, BAUDRATE_SPEED, false);
    if (m5atom_ok) std::wcout << L"  [OK] M5Atom port " << SERIAL_PORT_SPEED << L" opened." << std::endl;

    gnss_ok = init_serial(hSerialGNSS, SERIAL_PORT_GNSS, BAUDRATE_GNSS, false);
    if (gnss_ok) std::wcout << L"  [OK] GNSS port " << SERIAL_PORT_GNSS << L" opened." << std::endl;

    if (OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", 1, &ljHandle) == LJE_NOERROR) {
        labjack_ok = true;
        std::cout << "  [OK] LabJack U3 connected." << std::endl;
    }
    else {
        labjack_ok = false;
        std::cerr << "  [FAIL] LabJack U3 not detected." << std::endl;
    }

    XINPUT_STATE state;
    ZeroMemory(&state, sizeof(XINPUT_STATE));
    if (XInputGetState(0, &state) == ERROR_SUCCESS) {
        joystick_ok = true;
        std::cout << "  [OK] XInput Gamepad detected." << std::endl;
    }
    else {
        joystick_ok = false;
        std::cerr << "  [FAIL] XInput Gamepad not found." << std::endl;
    }

    // --- DFP 初期化 ---
    if (init_directinput()) {
        g_dfp_ok = true;
        std::cout << "  [OK] DirectInput DFP detected." << std::endl;
    }
    else {
        g_dfp_ok = false;
        // 検出失敗は通常のことなので [FAIL] ではなく [INFO] とする
        std::cerr << "  [INFO] DirectInput DFP not found." << std::endl;
    }

    // 優先権をリセット
    g_input_priority = InputDevicePriority::None;

    std::cout << "----------------------------" << std::endl;
}

// --- ハンドル取得 ---
void poll_dfp() {
    if (!g_dfp_ok.load() || !g_pDFP) {
        return;
    }

    static DIJOYSTATE2 last_dfp_state = { 0 };
    DIJOYSTATE2 js;

    HRESULT hr = g_pDFP->Poll();
    if (FAILED(hr)) {
        if (hr == DIERR_INPUTLOST || hr == DIERR_NOTACQUIRED) {
            if (FAILED(g_pDFP->Acquire())) {
                g_dfp_ok = false;
                std::cerr << "\n[WARN] DInput: DFP connection lost and failed to re-acquire." << std::endl;
                return;
            }
        }
        else {
            g_dfp_ok = false;
            std::cerr << "\n[WARN] DInput: DFP poll failed." << std::endl;
            return;
        }
    }

    hr = g_pDFP->GetDeviceState(sizeof(DIJOYSTATE2), &js);
    if (FAILED(hr)) {
        g_dfp_ok = false;
        std::cerr << "\n[WARN] DInput: DFP GetDeviceState failed." << std::endl;
        return;
    }

    // --- ボタン操作 (常時有効) ---
    // Y (△) ボタン -> 共有メモリモード切替 (Button 0)
    if ((js.rgbButtons[0] & 0x80) && !(last_dfp_state.rgbButtons[0] & 0x80)) {
        g_shared_memory_control = !g_shared_memory_control.load();
        if (g_shared_memory_control.load()) {
            std::cout << "\n[INFO] (DFP) Switched to Shared Memory Control Mode." << std::endl;
        }
        else {
            std::cout << "\n[INFO] (DFP) Switched to Gamepad/DFP Control Mode." << std::endl;
        }
    }

    // A (×) ボタン -> リバース切替 (Button 2)
    if ((js.rgbButtons[2] & 0x80) && !(last_dfp_state.rgbButtons[2] & 0x80)) {
        g_reverse_active = !g_reverse_active.load();
        if (m5atom_ok.load()) {
            write_serial_command(g_reverse_active.load() ? "R" : "N");
        }
    }



    last_dfp_state = js;

    // --- 軸操作 (優先権の確認と設定) ---
    long steer_axis = js.lX; // ステアリング
    bool dpad_up_accel = (js.rgdwPOV[0] == 0); // 十字キー上 (アクセル)
    bool dpad_down_brake = (js.rgdwPOV[0] == 18000); // 十字キー下 (ブレーキ)

    // DFPの軸入力があったか (デッドゾーン 1000/32767)
    bool dfp_axis_moved = (abs(steer_axis) > 1000 || dpad_up_accel || dpad_down_brake);

    if (dfp_axis_moved && g_input_priority.load() != InputDevicePriority::DirectInput) {
        g_input_priority = InputDevicePriority::DirectInput;
        std::cout << "\n[INFO] Switched to Driving Force Pro for axis input." << std::endl;
    }

    // --- 軸入力の適用 (DFPが優先権を持つ場合) ---
    if (g_input_priority.load() == InputDevicePriority::DirectInput && !g_shared_memory_control.load()) {
        // ステアリング
        double norm_x = steer_axis / 32767.0;
        g_steering_input_serial = norm_x;

    }
}

// --- Xboxコントローラ ---
void poll_gamepad() {
    XINPUT_STATE state;
    ZeroMemory(&state, sizeof(XINPUT_STATE));

    DWORD result = XInputGetState(0, &state);

    double throttle_norm = state.Gamepad.bRightTrigger / 255.0;
    double brake_norm = state.Gamepad.bLeftTrigger / 255.0;


    g_throttle_input_norm = throttle_norm;
    g_brake_input_norm = brake_norm;


    if (result == ERROR_SUCCESS) {
        joystick_ok = true;

        static WORD last_buttons = 0;
        WORD current_buttons = state.Gamepad.wButtons;

        // --- ボタン操作 (常時有効) ---
        // Yボタン -> 共有メモリモード切替
        if ((current_buttons & XINPUT_GAMEPAD_Y) && !(last_buttons & XINPUT_GAMEPAD_Y)) {
            g_shared_memory_control = !g_shared_memory_control.load();
            if (g_shared_memory_control.load()) {
                std::cout << "\n[INFO] (XInput) Switched to Shared Memory Control Mode." << std::endl;
            }
            else {
                std::cout << "\n[INFO] (XInput) Switched to Gamepad/DFP Control Mode." << std::endl;
            }
        }

        // Aボタン -> リバース切替
        if ((current_buttons & XINPUT_GAMEPAD_A) && !(last_buttons & XINPUT_GAMEPAD_A)) {
            g_reverse_active = !g_reverse_active.load();
            if (m5atom_ok.load()) {
                write_serial_command(g_reverse_active.load() ? "R" : "N");
            }
        }



        last_buttons = current_buttons;

        // --- 軸操作 (優先権の確認と設定) ---
        float lx = state.Gamepad.sThumbLX;
        byte rt = state.Gamepad.bRightTrigger;
        byte lt = state.Gamepad.bLeftTrigger;

        // XInputの軸入力があったか
        bool xinput_axis_moved = (abs(lx) > XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE || rt > 20 || lt > 20);

        if (xinput_axis_moved && g_input_priority.load() != InputDevicePriority::XInput) {
            g_input_priority = InputDevicePriority::XInput;
            std::cout << "\n[INFO] Switched to XInput Gamepad for axis input." << std::endl;
        }

        // --- 軸入力の適用 (XInputが優先権を持つ場合) ---
        if (!g_shared_memory_control.load() && g_input_priority.load() == InputDevicePriority::XInput) {
            // ステアリング
            double norm_x = 0.0;
            if (abs(lx) > XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) {
                norm_x = lx / 32767.0;
            }
            g_steering_input_serial = norm_x;
        }
        else if (g_input_priority.load() != InputDevicePriority::DirectInput) {
            // どのデバイスも優先権を持っていない場合、軸をニュートラルに戻す
            g_steering_input_serial = 0.0;
        }
    }
    else {
        // XInputコントローラーが切断された
        if (joystick_ok.load()) {
            std::cerr << "\n[WARN] XInput Gamepad disconnected." << std::endl;
        }
        joystick_ok = false;

        // 優先権を持っていた場合はリセット
        if (g_input_priority.load() == InputDevicePriority::XInput) {
            g_input_priority = InputDevicePriority::None;
            g_steering_input_serial = 0.0;
        }
    }
}

// --- M5Atom から速度取得
void read_speed_thread() {
    char buffer[256];
    std::string line_buffer;
    DWORD bytes_read;
    while (g_running) {
        if (m5atom_ok.load()) {
            if (ReadFile(hSerialSpeed, buffer, 1, &bytes_read, NULL) && bytes_read > 0) {
                if (buffer[0] == '\n') {
                    if (!line_buffer.empty()) {
                        try { g_vehicle_speed_kmh = std::stod(line_buffer); }
                        catch (const std::exception&) {}
                        line_buffer.clear();
                    }
                }
                else if (buffer[0] != '\r') { line_buffer += buffer[0]; }
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

void gnss_reader_thread() {
    char buffer[1024];
    std::string line_buffer;
    DWORD bytes_read;
    while (g_running) {
        if (gnss_ok.load()) {
            if (ReadFile(hSerialGNSS, buffer, 1, &bytes_read, NULL) && bytes_read > 0) {
                if (buffer[0] == '\n') {
                    if (!line_buffer.empty() && line_buffer.rfind("#BESTPOSA", 0) == 0) {
                        try {
                            auto main_parts = split_string(line_buffer, ';');
                            if (main_parts.size() > 1) {
                                auto header_parts = split_string(main_parts[0], ',');
                                auto data_parts = split_string(main_parts[1], ',');

                                if (data_parts.size() >= 10 && header_parts[0] == "#BESTPOSA" && data_parts[0] == "SOL_COMPUTED" && data_parts[1] == "NARROW_INT") {
                                    double lat = std::stod(data_parts[2]);
                                    double lon = std::stod(data_parts[3]);
                                    double lat_std = std::stod(data_parts[7]);
                                    double lon_std = std::stod(data_parts[8]);

                                    g_gnss_lat = lat;
                                    g_gnss_lon = lon;
                                    g_gnss_accuracy = (lat_std + lon_std) / 2.0;

                                    double x = EARTH_RADIUS_M * (lon - BASE_LON) * PI / 180.0 * std::cos(BASE_LAT * PI / 180.0);
                                    double y = EARTH_RADIUS_M * (lat - BASE_LAT) * PI / 180.0;

                                    g_gnss_x = x;
                                    g_gnss_y = y;

                                }
                                else {
                                    g_gnss_accuracy = 99.9;
                                }
                            }
                        }
                        catch (const std::exception&) {}
                    }
                    line_buffer.clear();
                }
                else if (buffer[0] != '\r') {
                    line_buffer += buffer[0];
                }
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}


// --- メイン制御スレッド ---
void control_thread_func() {
    reinitialize_devices();
    std::thread speed_reader(read_speed_thread);
    std::thread gnss_thread(gnss_reader_thread);
    auto last_record_time = std::chrono::steady_clock::now();

    while (g_running) {
        auto start_time = std::chrono::high_resolution_clock::now();

        if (g_reconnect_requested.load()) {
            reinitialize_devices();
            g_reconnect_requested = false;
        }

        // 両方のデバイスをポーリング
        poll_gamepad();
        poll_dfp();


        static double integral_term = 0.0;
        double steer_norm = 0.0;
        double throttle_norm = 0.0;
        double brake_norm = 1.0;//デフォルトはブレーキ

        if (sm != nullptr) {
            steer_norm = sm[212] / 1000.0; // -1〜1
            throttle_norm = sm[223];       // 0〜1
            brake_norm = sm[224];          // 0〜1
            sm[300] = g_steering_input_serial.load();  // -1〜1
            sm[301] = g_throttle_input_norm.load();
            sm[302] = g_brake_input_norm.load();

        }

        double throttle_to_set = map_value(throttle_norm, 0.0, 1.0, RT_PRESSED_MIN_V, RT_PRESSED_MAX_V);

        if (steering_ok.load()) {
            uint32_t current_steering_speed = MAX_STEERING_SPEED;
            // 操舵角は常に最大を使用
            int32_t current_max_steering_step = MAX_STEERING_STEP;

            //double jx_value = g_steering_input_serial.load();
            double jx_value = steer_norm;
            // 操舵の指数関数カーブ(POWER_CURVE)とデッドゾーン(DEAD_ZONE)処理を削除。
            // ジョイスティックの入力をそのまま(線形に)操舵指令値(processed_jx)とします。
            double processed_jx = jx_value;

            auto motor_command = create_motor_command(processed_jx, current_steering_speed, current_max_steering_step);
            write_serial_steering(motor_command);
        }

        if (labjack_ok.load()) {
            double current_voltage_ain0 = 0.0;
            eAIN(ljHandle, 0, 31, &current_voltage_ain0, 0, 0, 0, 0, 0, 0);
            g_actual_voltage_pot = current_voltage_ain0;

            // ブレーキ指令を 0〜1 にクランプ（共有メモリの値が範囲外でも安全にする）
            double brake_cmd = std::max(0.0, std::min(1.0, brake_norm));

            // デフォルトはモーター停止
            double dac1_output_v = MOTOR_STOP_V;

            double target_voltage_pot_local = map_value(brake_cmd, 0.0, 1.0, POT_MIN_V, POT_MAX_V);

            // ブレーキ指令がしきい値を超えたときだけ位置制御(bang-bang)を行う。
            // 何も操作していない状態(brake_cmd≒0)では到達不能な目標を追い続けて
            // モーターが端で突っ張りカタカタ鳴るため、その場合は停止のままにする。
            if (brake_cmd > BRAKE_RELEASE_THRESHOLD) {
                double error = target_voltage_pot_local - current_voltage_ain0;
                if (error > TOLERANCE_V) dac1_output_v = MOTOR_FORWARD_V;
                else if (error < -TOLERANCE_V) dac1_output_v = MOTOR_REVERSE_V;
            }

            // 診断用に実値を公開
            g_brake_cmd = brake_cmd;
            g_brake_target_v = target_voltage_pot_local;
            g_brake_motor_v = dac1_output_v;

            set_dac_voltage(ljHandle, 1, dac1_output_v);
            set_dac_voltage(ljHandle, 0, throttle_to_set);
        }


        if (sm != nullptr) {
            // デバイスステータスを共有メモリに書き込む
            sm[1000] = g_shared_memory_control.load() ? 1.0 : 0.0; // 制御モード (1: SharedMem, 0: Gamepad)
            sm[1001] = steering_ok.load() ? 1.0 : 0.0;         // ステアリングOK
            sm[1002] = m5atom_ok.load() ? 1.0 : 0.0;          // M5Atom (速度計) OK
            sm[1003] = gnss_ok.load() ? 1.0 : 0.0;            // GNSS OK
            sm[1004] = labjack_ok.load() ? 1.0 : 0.0;         // LabJack OK
            sm[1005] = joystick_ok.load() ? 1.0 : 0.0;        // ジョイスティック OK
            sm[1006] = g_dfp_ok.load() ? 1.0 : 0.0;           // DFP (DirectInput) OK

            sm[64] = g_vehicle_speed_kmh.load();
            sm[41] = map_value(g_actual_voltage_pot.load(), POT_MIN_V, POT_MAX_V, 0.0, 1.0);
            sm[505] = g_gnss_x.load();
            sm[506] = g_gnss_y.load();
            sm[508] = g_gnss_accuracy.load();
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
        if (elapsed < LOOP_INTERVAL) {
            std::this_thread::sleep_for(LOOP_INTERVAL - elapsed);
        }
    }

    if (m5atom_ok.load()) write_serial_command("N");
    if (speed_reader.joinable()) speed_reader.join();
    if (gnss_thread.joinable()) gnss_thread.join();

    std::cout << "\n--- Shutting down devices ---" << std::endl;
    if (labjack_ok.load()) { set_dac_voltage(ljHandle, 1, MOTOR_STOP_V); set_dac_voltage(ljHandle, 0, RT_UNPRESSED_V); Close(); std::cout << "  - LabJack closed." << std::endl; }
    if (steering_ok.load()) { close_serial(hSerialSteering); std::cout << "  - Steering port closed." << std::endl; }
    if (m5atom_ok.load()) { close_serial(hSerialSpeed); std::cout << "  - M5Atom port closed." << std::endl; }
    if (gnss_ok.load()) { close_serial(hSerialGNSS); std::cout << "  - GNSS port closed." << std::endl; }
    // ★★★ DFP 終了処理 ★★★
    if (g_dfp_ok.load()) { shutdown_directinput(); std::cout << "  - DirectInput (DFP) closed." << std::endl; }
    // ★★★★★★★★★★★★★★★★
    std::cout << "-----------------------------" << std::endl;
}

// --- 表示用ヘルパー ---------------------------------------------------------
// ラベルと数値を「同じ行・同じ出力文」で一体化して描画するための整形関数群。
// 値が短くなっても前フレームの文字が残らないよう、右側を空白で埋めて固定幅にする。

// 数値を固定幅(width)・小数点以下(precision)桁の文字列へ整形
static std::string format_field(double value, int precision, int width) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    std::string s = oss.str();
    if (static_cast<int>(s.size()) < width)
        s += std::string(width - static_cast<int>(s.size()), ' ');
    return s;
}

// 文字列(ステータス等)を固定幅(width)へ整形
static std::string format_field(const std::string& text, int width) {
    std::string s = text;
    if (static_cast<int>(s.size()) < width)
        s += std::string(width - static_cast<int>(s.size()), ' ');
    return s;
}

// ダッシュボードを丸ごと1回で描画する。
// 画面はクリアせず、カーソルを左上(0,0)に戻して上書きするのでチラつかない。
// ラベルと数値を同じ行にまとめて出力するため、
// 「座標で後から数値を流し込んで位置がズレる」問題が起きない。
void draw_dashboard() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    COORD home = { 0, 0 };
    SetConsoleCursorPosition(h, home);

    const double brk = map_value(g_actual_voltage_pot.load(), POT_MIN_V, POT_MAX_V, 0, 100);

    std::ostringstream out;
    out << "\n";
    out << "=========== VEHICLE DASHBOARD ===========\n";
    out << "\n";
    out << "[SYSTEM]\n";
    out << "  Steering  : [" << format_field(steering_ok.load() ? "OK" : "FAIL", 8) << "]\n";
    out << "  GNSS      : [" << format_field(gnss_ok.load()     ? "OK" : "FAIL", 8) << "]\n";
    out << "  LabJack   : [" << format_field(labjack_ok.load()  ? "OK" : "FAIL", 8) << "]\n";
    out << "\n";
    out << "[INPUT]\n";
    out << "  Steering  : [" << format_field(g_steering_input_serial.load(), 3, 8) << "]\n";
    out << "  Throttle  : [" << format_field(g_throttle_input_norm.load(),   3, 8) << "]\n";
    out << "  Brake     : [" << format_field(g_brake_input_norm.load(),      3, 8) << "]\n";
    out << "\n";
    out << "[VEHICLE]\n";
    out << "  Speed     : [" << format_field(g_vehicle_speed_kmh.load(), 2, 8) << "] km/h\n";
    out << "  BrakePos  : [" << format_field(brk,                        1, 8) << "] %\n";
    out << "\n";
    // ブレーキ制御の実値（カタカタ原因の切り分け用）
    // Cmd sm224 = 実際にアクチュエータを動かす指令。何も操作していないのにここが0でなければ
    //             原因は上位プロセス(PathFollower)/共有メモリ側。MotorOut が常時 FWD/REV なら駆動中。
    const double motor_v = g_brake_motor_v.load();
    const char* motor_state =
        (motor_v > MOTOR_STOP_V + 0.1) ? "FWD(pull)" :
        (motor_v < MOTOR_STOP_V - 0.1) ? "REV(release)" : "STOP";
    out << "[BRAKE CTRL]\n";
    out << "  Cmd sm224 : [" << format_field(g_brake_cmd.load(),      3, 8) << "]\n";
    out << "  TargetV   : [" << format_field(g_brake_target_v.load(), 3, 8) << "] V\n";
    out << "  ActualV   : [" << format_field(g_actual_voltage_pot.load(), 3, 8) << "] V\n";
    out << "  MotorOut  : [" << format_field(motor_state,            12) << "]\n";
    out << "\n";
    out << "[POSITION]\n";
    out << "  X         : [" << format_field(g_gnss_x.load(), 3, 8) << "] m\n";
    out << "  Y         : [" << format_field(g_gnss_y.load(), 3, 8) << "] m\n";
    out << "\n";
    out << "========================================\n";

    std::cout << out.str() << std::flush;
}


// --- コンソールベースのmain関数 ---
int main() {

    if (!init_shared_memory()) return -1;

    std::thread control_thread(control_thread_func);

    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

    // カーソルの点滅が表示に被らないように非表示にする
    CONSOLE_CURSOR_INFO cci;
    if (GetConsoleCursorInfo(hConsole, &cci)) {
        cci.bVisible = FALSE;
        SetConsoleCursorInfo(hConsole, &cci);
    }

    system("cls"); // 最初に一度だけクリア

    while (g_running) {
        // ラベルと数値を一体化したダッシュボードを毎フレーム上書き描画する
        draw_dashboard();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (control_thread.joinable()) control_thread.join();

    shutdown_directinput();
    shutdown_shared_memory();

    return 0;
}
