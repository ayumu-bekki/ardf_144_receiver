# ARDF 144MHz Receiver

ESP32-C6ベースの144MHz帯ARDF (Amateur Radio Direction Finding) 用受信機プロジェクトです。
ハードウェアからファームウェア、ケース、アンテナ設計データまでを含んでいます。

## 概要

本プロジェクトはスーパーヘテロダイン方式を採用した144MHz帯ARDF競技用受信機です。
各種制御はXIAO ESP32C6で行い、局部発振器にSi5351Aを使用しています。

### 主な特徴・構成要素
- **XIAO ESP32C6**: メインコントローラ
- **Si5351A**: クロックジェネレータ（局部発振器 LO）
- **MCP4018**: デジタルポテンショメータによる電子ボリューム制御（Aカーブ対応）
- **SSD1306**: 128x32 OLEDディスプレイ搭載（周波数、プリアンプ状態、ボリュームなどを表示）
- カスタムPCB設計 (KiCad)
- 3Dプリント対応専用ケース設計 (FreeCAD / STEP)
- 144MHz帯3エレメント八木・宇田アンテナ設計 (FreeCAD)

## ディレクトリ構成

- `app/ardf_receiver/`: ESP-IDF ファームウェアソースコード (C++17)
- `antenna/`: 144MHz帯3エレメント八木アンテナ設計データ (FreeCAD)
- `case/`: 3Dプリント用ケース設計データ (FreeCAD / STEP)
- `pcb/`: カスタムプリント基板設計データおよびガーバーデータ (KiCad)

## 開発環境 (ファームウェア)

- **フレームワーク**: ESP-IDF (v6.0)
- **言語**: C++17

### ビルドと書き込み

```bash
cd app/ardf_receiver
# ESP-IDF環境のセットアップ (環境に応じて実行)
. $IDF_PATH/export.sh

# ビルド
idf.py build

# デバイスへの書き込みとシリアルモニタの起動
idf.py flash monitor
```

## ライセンス

本プロジェクトは [Apache License 2.0](LICENSE) のもとで公開されています。
Copyright (c) 2026 Ayumu Bekki
