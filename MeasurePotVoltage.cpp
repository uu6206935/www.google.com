// =============================================================
//  ポテンショメータ電圧 測定ツール (LabJack U3 / Windows / C++)
// -------------------------------------------------------------
//  目的: ブレーキの校正用に AIN0(ポテンショメータ) の電圧を実測する。
//
//  ★安全設計★
//   - モーター方向指令 DAC1 には「停止電圧 2.5V」しか出しません。
//   - 駆動電圧(正転5.0V / 逆転0.0V)は一切出しません。
//   - よってこのプログラムを実行してもブレーキモーターは動きません。
//   - 制御ループ(追従制御)は入っていないので、暴走の余地がありません。
//
//  使い方:
//   1) このプログラムをビルドして起動する。
//   2) ブレーキ全開放のまま AIN0 の値を読む  → これが POT_MIN_V の正しい値。
//   3) (機構が手で動かせる場合) ブレーキを手で最大まで掛けて max を読む
//      → これが POT_MAX_V の正しい値。約0.75V出れば正常、出なければ断線/電源喪失を疑う。
//   4) 終了は Ctrl+C もしくはウィンドウを閉じる(DAC1は2.5V=停止のまま残ります)。
// =============================================================
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <windows.h>
#include "LabJackUD.h"
#pragma comment(lib, "LabJackUD.lib")

// --- 既存プログラム(Bike_BrakeThrottleC+)と同じ安全値 ---
const double MOTOR_STOP_V    = 2.5; // DAC1: モーター停止。これしか出さない。
const double THROTTLE_IDLE_V = 0.8; // DAC0: スロットルをアイドル(離した状態)に。

int main() {
    LJ_HANDLE lj = 0;

    LJ_ERROR e = OpenLabJack(LJ_dtU3, LJ_ctUSB, "1", 1, &lj);
    if (e != LJE_NOERROR) {
        char err[256]; ErrorToString(e, err);
        std::cerr << "[FATAL] LabJack U3 を開けません: " << err << std::endl;
        std::cerr << "USB接続とドライバ(LabJackUD)を確認してください。" << std::endl;
        std::cout << "Enterで終了..." << std::endl; std::cin.get();
        return 1;
    }
    std::cout << "[OK] LabJack U3 接続成功" << std::endl;

    // ★安全初期化: モーター停止(2.5V) と スロットルアイドル(0.8V) を出す。
    //   駆動電圧は出さないので、ここでモーターが動くことはありません。
    eDAC(lj, 1, MOTOR_STOP_V, 0, 0, 0);    // DAC1 = モーター方向指令 = 停止
    eDAC(lj, 0, THROTTLE_IDLE_V, 0, 0, 0); // DAC0 = スロットル = アイドル

    std::cout << "[INFO] DAC1=2.5V(モーター停止) を出力。モーターは駆動しません。\n" << std::endl;
    std::cout << "AIN0(ポテンショメータ) を測定します。Ctrl+C / ウィンドウを閉じて終了。\n" << std::endl;
    std::cout << "  全開放のときの値 -> POT_MIN_V に入れる値" << std::endl;
    std::cout << "  全制動のときの値 -> POT_MAX_V に入れる値(約0.75V出れば正常)\n" << std::endl;

    double vmin = 1e9, vmax = -1e9;
    while (true) {
        double v = 0.0;
        // AIN0 を単線(single-ended, 負入力=31)で読み取り。既存プログラムと同じ呼び方。
        e = eAIN(lj, 0, 31, &v, 0, 0, 0, 0, 0, 0);
        if (e == LJE_NOERROR) {
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
            std::cout << "\rAIN0 = " << std::fixed << std::setprecision(4) << std::setw(8) << v
                      << " V    |   min " << std::setw(8) << vmin
                      << "    max " << std::setw(8) << vmax << "     " << std::flush;
        }
        else {
            char err[256]; ErrorToString(e, err);
            std::cerr << "\n[WARN] eAIN 失敗: " << err << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // (通常ここには到達しません。Ctrl+C 終了時も DAC1 は 2.5V=停止 のまま残ります)
    return 0;
}
