//--WiFi接続設定の見本
//  このファイルをコピーして wifi_config.h という名前で保存し、環境に合わせて変更する
//  wifi_config.h はパスワードを含むため .gitignore でGit管理外にしている
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#define WIFI_SSID   "YOUR_SSID"        //WiFiのSSID
#define WIFI_PASS   "YOUR_PASSWORD"    //WiFiのパスワード
#define AGENT_IP    "192.168.11.2"     //micro_ros_agentを動かすPCのIPアドレス
#define AGENT_PORT  8888               //agentのUDPポート (全台共通でよい)

#endif
