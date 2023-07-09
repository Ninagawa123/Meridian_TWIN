
# IcsClass for Arduino
 ---
## Overview
近藤科学製ICS機器をArduinoのSerial(HardwareSerial)から動かすためのライブラリです。
This library is for connecting a Kondo Kagaku ICS device from an Arudino serial(HardwareSerial) port.

## Description
近藤科学製サーボモータや受信機等ICS機器をArduinoで動かすためのライブラリです。  
This library is for connecting a Kondo Kagaku servo motor, receiver or other ICS device using Arudino.

サーボモータの位置制御(ポジション)コマンドやストレッチ等のパラメータ変更がArduinoからできるようになります。
It enables servo motor position commands and parameter changes like for “stretch” to be performed from Arudino.

受信機(KRR)に接続することで、送信機(KRC)からのデータも取得できます。
By connecting to a receiver (KRR), data can also be acquired from a transmitter (KRC).

ArduinoからはSerial(HardwareSerial)を用いて通信をします。
Communications can be performed from Arudino using a serial port (HardwareSerial).


## Requirement
Arduino Uno (Serial)
Arduino Meag (Serial1 ~ Serial3)

ArduinoからアクセスできるIcsHardSerialClassを使うにはIcsClassのIcsBaceClassをリンクできるようにしてください。
To use IcsHardSerialClass, which can be accessed from Arudino, make sure IcsClass’s IcsBaseClass can be linked.

他のマイコンを使いたい場合は、IcsBaceClassを派生させると便利です。
To use another computer, deriving IcsBaseClass is a convenient approach.


## Usage
配布フォルダのマニュアルをご覧ください。
For details, refer to the manual (PDF) in the folder provided.

## Licence
Copyright 2018 Kondo Kagaku co.,ltd.
[MIT](http://opensource.org/licenses/mit-license.php)
see to MIT_Licence.txt


## Author
近藤科学株式会社
Kondo Kagaku co.,ltd.
T.Nobuhara
近藤科学ホームページ:(<http://kondo-robot.com/>)
