##機能追加
- 複数のロボットを一つのROSネットワークに接続するための設定変更

## 重要な変更履歴

### v1.4 (2026-10-13) オドメトリの向きの修正
- `odom`・`/tf` のクォータニオンを `z=sin(phi)`, `w=cos(phi)` から `z=sin(phi/2)`, `w=cos(phi/2)` に修正
- 修正前はRVizなどでロボットの向きが実際の2倍の角度で表示されていた (位置 px, py は修正前から正しい)
- 確認: ロボットを90°回転させ、RVizの向きも90°になること

### v1.3 (2026-10-13) WiFi設定の分離・GitHub公開
- SSID・パスワード・agentのIP/ポートを `wifi_config.h` に分離 (Git管理外)
- 新しい環境では `wifi_config.example.h` をコピーして `wifi_config.h` を作る (ないとコンパイルエラー)
- GitHub公開のため、過去のコミットからもパスワードを削除 (履歴を書き換えたのでコミットIDが変わった)

### v1.2 (2026-10-12) 配信周期の変更
- `odom`・`/tf` を 20Hz (50ms)、`whValue`・`joint_states` を 5Hz (200ms) に変更 (従来は全トピック100Hz)
- 周期は `ros2robocar.h` の `ODOM_PUB_PERIOD_MS` / `STATE_PUB_PERIOD_MS` で設定
- 注意: `timer_callback` を `odom_timer_callback` / `state_timer_callback` に分割した
- 確認: `ros2 topic hz /robot_XXXXXX/odom` で約20Hz

### v1.1 (2026-10-12) WiFi省電力モードOFF
- `WiFi.setSleep(false)` に加え `esp_wifi_set_ps(WIFI_PS_NONE)` で省電力を確実にOFF
- 接続前・接続後・再接続時に設定。起動ログに `PowerSave: OFF` と表示される

### v1.0 (2026-10-12) Git管理開始
- WiFi/USB切替、複数ロボット対応(MACアドレスから名前空間生成)、agent自動再接続を含む状態
