### bugfixed 20240325
esp32側のpratformio.ini内、lib_depsのhideakitai/ESP32DMASPI@0.1.2が自動では検出できなくなったため、  
hideakitai/ESP32DMASPI@0.3.0 にアップデートしました。  
  
# Update 20230710  v1.0.0

ライブラリ対応版として大幅なアップデートを行いました.(前回までのバージョンはoldディレクトリにzipで格納しています.)      
メインが整理され, 改造の見通しが立ちやすくなりました.  
また, マイコンごとにまちまちだった変数名の書式も統一しました.  
変数のルールやライブラリ関数については, 下記に情報を集約していきます.  
https://ninagawa123.github.io/Meridian_info/  

# Meridian_TWIN
  
Meridian計画はヒューマノイドの制御システムについてのオープンソースプロジェクトです.  
ホビーロボットのデジタルツイン化を簡単に実装することができ, PC上のシミュレーション空間とロボット実機をWIFI経由で10msの更新頻度でデータリンクします.  
    
システムの中核はMeridim配列というロボットの状態データを格納できる軽量で汎用的なデータ配列です.  
このデータ配列がデバイス間を高速に廻ることで, リアルタイムな状態データを共有を可能にします.  
Meridim配列を中間プロトコルとして既存のシステムの間に挟むことで, 複数社のコマンドサーボやセンサ, Unityなどの開発環境, ROSで使用可能な多岐にわたるシミュレーターなどを自由に繋ぎ合わせることができます.  
  
当リポジトリで取り扱う ”Meridian_TWIN" はESP32とTeensy4.0を併用するタイプで, 対応ボードはMeridian Board Type.Kとなります.  
また, ESP32DeckitC単体で動作する簡易バージョンのMeridian_LITE(対応ボードはMeridian Board -LITE-)も開発済みです.  
Meridianは今後も用途に応じて様々なハードウェア, ソフトウェアに対応させていく予定です.  
  
[![sync](https://img.youtube.com/vi/4ymSV_Dot-U/0.jpg)](https://www.youtube.com/watch?v=4ymSV_Dot-U)  
100Hzデータリンクのデモ動画
  
[![dance](https://img.youtube.com/vi/Wfc9j4Pmr3E/0.jpg)](https://www.youtube.com/watch?v=Wfc9j4Pmr3E)  
100Hzダンスのデモ動画
  
# 開発資料  
  
プログラムのフローや配列の定義, ボードのピンアサインについては「document」ディレクトリの中に資料としてまとめています.  
専用ボードの回路図も公開しており, 自作したりブレットボードで再現することが可能です.  
動作可能なサンプルプログラムも当リポジトリ内で公開しています.  
  
Meridianの概要や変数やライブラリ関数について, 下記に集約中です.  
https://ninagawa123.github.io/Meridian_info/

また, 全体の仕組みや開発進捗は以下のnoteにまとめています.  
https://note.com/ninagawa123/n/ncfde7a6fc835  
  
  
# System composition  
  
Meridian_TWINは, ハードウェアとして通信用のESP32DevKitC, 制御用のTeensy4.0, それらを連結する専用ボードのMeiridian Boardで構成されます.  
デモは近藤科学のKRSサーボ(通信速度1.25Mbps)に対応しており, Meiridian Board Type.KはKHR-3HV用に搭載することができます.  
PC側はROS1のmelodic/noeticに対応しており, 現在Rvizでの表示が可能です. またUnity(Mac/Win版)でもヒューマノイドの姿勢をリアルタイムに反映させることができます.  
  
<img width="500" alt="TypeK" src="https://user-images.githubusercontent.com/8329123/180233435-4ed5fcb0-a2c6-4e73-94b1-b842ffb79af4.png">  
  
  
# 開発環境  
- PlatformIO (Teensy4.0のプラットフォームバージョンは3.5.0対応)  
- Teensyduino(Teensy Loader 1.54, PlatformIOと併用)  
- Teensy4.0  
- ESP32DevkitC  
- MPU6050(GY-521)  
- Meridian Board Type.K
  
  
# Installation  
Teensy4.0, ESP32DevKitCにそれぞれのファイルを書き込みます.  
以下の説明の理解にはPlatformIOやTeensy4.0, ESP32の扱いについてのごく初歩的な知識が必要です.
PlatformIOを初めて使うような方向けの導入手順については後日まとめる予定です.  
普段Arduino IDEを使っている方のためのPlatformIOの導入Tipsについては下記にまとめました.  
https://qiita.com/Ninagawa_Izumi/items/6f58d9dbfdfe99be9c13  
    
  
# ファイルの準備  
当リポジトリ右上の「\<\>code」ボタンより「Download ZIP」を選択し,  
ファイルをお手元のPCの適切なディレクトリに展開します.  
(もちろんgit cloneなど他の手順でも構いません.)  

## Teensy4.0の準備  
#### PlatformIOでTeensy4.0用のプロジェクトファイルを開く  
PlatformIOのファイルメニューより「フォルダーを開く」とし, 先ほど展開したファイルの中から「Meridian_TWIN_for_Teensy40」のディレクトリを選択します.  
(Meridian_TWIN_for_ESP32と間違えないようご注意ください.)  

主なファイル構成は下記になります.  
  
<p>MERIDIAN_TWIN_FOR_TEENSY40  <br>
&nbsp;├lib  <br>
&nbsp;│&nbsp;└ ICSClass_V210 (サーボ用ライブラリ)  <br>
&nbsp;├ src  <br>
&nbsp;│&nbsp;├ config.h (Meridian設定ファイル)  <br>
&nbsp;│&nbsp;├ main.cpp (Meridian本体)  <br>
&nbsp;│&nbsp;└ main.h  (Meridianヘッダファイル)  <br>
&nbsp;└ platfomio.ini (ボード設定ファイル)  </p>

#### Teensy4.0に導入されるライブラリ
下記のライブラリはファイルを開く際に自動的に導入されます.  
- **Meridian by Ninagawa123** 
- **TsyDMASPI by hideakitai** 
- **MPU6050 by Electronic Cats** 
- **Adafruit_BNO055** 
- **Adafruit BusIO** 
- **Adafruit Unified Sensor** 
- **IcsClass_V210** (詳細は下記)

##### IcsClass_V210の導入について
近藤科学のICSサーボのためのライブラリもMITライセンスに基づき同梱していますが,  
最新版については下記をご参照ください.  
https://kondo-robot.com/faq/ics-library-a2  
  
#### サーボのマウントを設定する
Teensy4.0用のソースコードの「src/config.h」を開き,155行目ごろから始まるサーボ設定のところで,各サーボのマウントありなしを変更します.   
接続しているサーボIDは1に, 接続していないIDは0に設定します.  
サーボのマウント設定により, KHR-3HVのフルセットがなくてもICSサーボが最低１つあればデモをテストすることができます.  
  
#### ロボットの姿勢とサーボを設定する  
接続するKRSサーボの通信速度設定をすべて**1.25Mbps**に変更します.  
  
また, サーボの0度状態を下記の姿勢に, サーボの＋回転方向も下図の矢印方向に合わせます.  
左半身および体の中心は下図に順次つつ, 右半身については左半身のミラー方向に回転に合わせます.  
サーボの回転方向は, サーボの内部の設定変更が望ましいですが, 「src/config.h」でも変更できます.  
(ESP32用のファイルにも同名のconfig.hが存在しますのでご注意ください.)  
<img width="600" alt="motorccw" src="https://user-images.githubusercontent.com/8329123/147812253-e6cbe388-f70a-445f-80c0-b4cd899aa15a.png">
  
#### サーボを接続する
<img width="600" src="https://raw.githubusercontent.com/Ninagawa123/Meridian_TWIN/main/documents/TypeK_pinassign.png">  
こちらのピンアサインを参考に, サーボを接続します.  
  
#### センサーを接続する  
MPU/AHRSセンサをMeridianボードのI2Cピンに接続します.  
今のところキャリブレーション済みのMPU6050(GY-521)のみ対応しています.  
センサーがない場合は, Teensy4.0のソースコード設定でセンサの接続をオフにすることができます.  
  
#### Teensy4.0にソースコードを書き込む
Teensy4.0とPCをUSBケーブルで接続し, PlatformIOの下にある「チェックマーク」のボタンを押して内容をビルドし,[SUCCESS]が表示されることを確認します. その後, 「→」ボタンを押してTeensy4.0にコードを書き込みます.(ボードは自動的に認識されます.)  
センサーやリモコンなどの機器の接続について, 「src/config.h」にて詳細に設定できます.    
  
## ESP32DevkitCの準備

#### PlatformIOでESP32用のプロジェクトファイルを開く  
PlatformIOのファイルメニューより「フォルダーを開く」とし, 先ほど展開したファイルの中から「Meridian_TWIN_for_ESP32」のディレクトリを選択します.  
(Meridian_TWIN_for_Teensy40と間違えないよう注意.)  

主なファイル構成は下記になります.  
  
<p>MERIDIAN_TWIN_FOR_ESP  <br>
  ├ lib  <br>
  │  └ ESP32Wiimote (wiiリモコン用ライブラリ)  <br>
  ├ src  <br>
  │  ├ config.h (Meridian設定ファイル)  <br>
  │  ├ main.cpp (Meridian本体)  <br>
  │  └ main.h  (Meridianヘッダファイル)  <br>
  └ platfomio.ini (ボード設定ファイル)  </p>

#### ESP32に導入されるライブラリ  
下記のライブラリはファイルを開く際に自動的に導入されます.  
- **Meridian@^0.1.0 by Ninagawa123**  
- **ESP32DMASPI@0.1.2 by hideakitai**  
  
PlatformIOで"MeridianTWIN_ESP32"等の名前で新規プロジェクトを作成し, BoardはEspressif ESP32 Dev Module, FrameWorkにはArduinoを選択します.  
    
#### 追加のライブラリを導入し、修正する  
- **PS4Controller.h**  
PS4リモコン用のライブラリを追加導入します.(https://github.com/aed3/PS4-esp32)  
libのインポートなどにルールがあり、下記にまとめました.  
またPS4ライブラリをESP32用に修正する方法もまとめています  
https://qiita.com/Ninagawa_Izumi/items/d8966092fe2abd3cba79  

##### 接続先のPCのIPアドレスを調べる
windowsのコマンドプロンプトを開き,  
$ ipconfig （Ubuntuの場合は$ ip a もしくは $ ifconfig）  
と入力しコマンド実行します.  
IPv4アドレスが表示されます(192.168.1.xxなど)  
Macの場合は画面右上のwifiマークから”ネットワーク”環境設定...で表示されます.  
  
##### WIFIを設定する  
「src/config.h」 /\* Wifiアクセスポイントの設定 \*/ のところで,  
接続したいWIFIのアクセスポイントのSSIDとパスワードを入力します.  
アクセスポイントは5GHzではなく**2.4GHz**に対応している必要があります.  
また, 先ほど調べた接続先のPCのIPアドレスも記入します.  
  
#### ESP32書き込み用のCP210ドライバを導入する  
すでにお手元でESP32 DevkitCに書き込みを行ったことのあるPCであれば問題ないですが、
初めての場合、「CP210x USB - UART ブリッジ VCP ドライバ」が必要になる場合があります。
未導入の方は下記サイトより適切なものをインストールをしてください。
https://jp.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads
  
#### ESP32にソースコードを書き込む  
ここで一度, 更新したファイルを**セーブしESP32に書き込みます**.  
ESP32とPCをUSBケーブルで接続し, PlatformIOの下にある「チェックマーク」のボタンを押して内容をビルドし,[SUCCESS]が表示されることを確認します. その後, 「→」ボタンを押してESP32にコードを書き込みます.(ボードは自動的に認識されます.)  
##### ESP32のアップロードがうまくいかない場合  
アップロードが失敗する場合でも, 何度か行うことで成功する場合があるので試してみてください.  
アップロード開始時にESP32DeckitCのENボタンを押すことでアップロードがうまくいく場合もあります.  
また, ESP32DeckitCのENとGNDの間に10uFのセラミックコンデンサを入れると、ENボタンを押さずとも書き込みができるようになる場合があります.

#### ESP32のIPアドレスを調べる  
PlatformIOで画面下のコンセントアイコンからシリアルモニタを開き, ESP32DevKitC本体のENボタンを押します.  
wifi接続に成功すると  
> Hello, This is Meridian_TWIN_for_ESP32_20230710.  
> WiFi connected to  => xxxxxxx  
> PC's IP address is  => 192.168.1.xxx  
> ESP32's IP address is  => 192.168.1.xxx  
> ESP32's Bluetooth Mac Address is => xx:xx:xx:xx:xx:xx  
  
と表示され, 「ESP32's IP address =>」にESP32本体のIPアドレスが表示されます.このxxの番号をメモしておきます.  
  
#### ESP32にソースコードを書き込む  
PS4リモコンを使用する場合は, 「src/config.h」にESP32のBluetooth Mac Addressを記入します.  
更新したファイルをセーブし, ESP32に書き込みます.  
　　
#### platformio.ini  
platformio.iniでは以下の設定を行なっています.
- platformのバージョン指定
- PCとのSerial通信速度設定を115200に指定
- ライブラリの指定
- OTA（無線経由のプログラム書き込み機能）の無効化によるパーティション拡張
  
#### 設定の確認    
他にも, 接続するリモコンやシリアルモニタなどについての設定が可能です.  
Bluetoothリモコンを接続しない場合は必ずMOUNT_JOYPADを0に設定してください.(リモコンの受信待ちでMeridianの通信速度が大幅に低下します.)  
  
#### 各種設定の確認  
Teensy40, ESP32両方の「src/config.h」内のコメントを参考に適宜変更してください.  
Teensy40は主にサーボやセンサーなどのハードウェア接続の設定や制御システムの基本設定,  
ESP32は主に通信系のWifiとBluetoothリモコンの設定になります.  
  
これでMeridian Board側の設定は完了です.  
  
  
#  Meridian consoleを実行する  
Meridianで受け取るデータを表示できるコンソールを用意しました.python3が使える環境で実行可能です.  
https://github.com/Ninagawa123/Meridian_console  
![meridian_console](https://raw.githubusercontent.com/Ninagawa123/Meridian_console/main/image/console_img.jpg)  
  
#  Unity版デモを実行する  
  
Meridian_TWINとUnityを連携させることができます.  
下記のリポジトリの内容をお試しください.  
[https://github.com/Ninagawa123/Meridian_Unity/tree/main](https://github.com/Ninagawa123/Meridian_Unity/tree/main)  
  
<img width="500" alt="Meridian_Unity" src="https://github.com/Ninagawa123/Meridian_TWIN/assets/8329123/5b486e83-40b8-4556-8a98-8d0ac643effd">

  
# ROS版デモを実行する
  
### ROS noeticの導入
お手持ちの環境にROSを導入してください.  
以下の公式のインストール方法をご参照ください.
http://wiki.ros.org/ja/noetic/Installation/Ubuntu
  
また、Raspberry pi4でROS-noeticを導入する手順については下記にまとめました.
https://qiita.com/Ninagawa_Izumi/items/e84e9841f7a048832fcc  
  
### URDFの表示テスト
https://github.com/Ninagawa123/roid1
まず, こちらのREADMEにしたがってRvizでロボットを表示できるか確認します.  
  

### ROS, rviz, meridian_demoを実行する
１つ目のターミナルを開き,  
$ roscore  
  
２つ目のターミナルを開き,  
$ roslaunch roid1_urdf display_meridian_demo.launch  
*この時点ではロボットはベースとなる腰部分しか表示されません*  
  
３つ目のターミナルを開き,  
$ CD ~/(Meridian_console.pyのあるディレクトリ)  
$ python Meridian_console.py  
  
MeridianBoardの電源を入れ接続が確立すると, Meridian consoleの画面のデータが小さく変動し続けます.  
ここでMeridian consoleの「->ROS1」にチェックを入れるとロボットRoid.1の姿が現れ,  ロボットのサーボ位置が画面の表示に反映されます.  
そのまま（他のチェックボックスが空の状態）で, ロボットのサーボを手で動かした時にロボットにも反映されます.  
  
また, 「DEMO」「Enable」にチェックを入れると, 画面内のロボットがサインカーブで構成されたダンスのデモを行います.  
ここでさらに「Power」にもチェックを入れると, ロボットのサーボにパワーが入り, 画面と同じ動きを実機で再現します.  
  
  
# トラブルシューティング
  
### ROS版デモ実行時のトラブルシューティング
Error: Cannot assign requested address となる  
*→ おそらくアドレス番号が「192.168.x.xx」などのまま書き変わっていません.「ESP32のIPアドレスを調べる」「PCやラズパイ自身のIPアドレスを調べる」の項目を参考に,  
Meridian consoleのアドレスを更新してください.*  
  
### ボードが動いていない時のトラブルシューティング  
動作テストとしてUSB給電のみで使っている場合に動作しない場合があります.  
その場合, ESP32側にUSB給電することで動く場合があります.  
それでも動かない場合は電源供給付きのUSBハブを利用するか, Meridianボードに電源を接続することでアンペアを確保してください.  
Meridianボードの電源入力にバッテリーや安定化電源で電力を供給することでも安定的に動きます.  
  
### 既知の課題  
  
####  Meridian Board(2023.07.10)
- 各経路で, 0.1%以下の通信エラーが発生します.Meridianのシステムでは通信エラー時はすぐに次のデータを使用することでこのエラーをフォローしています.
- PS4コントローラの接続時, データスキップが5%~15%ほど生じます.
- Wiiコントローラの接続時はデータのスキップはほとんど発生しません.

####  Meridian Board Type.K のフリーピン結線時の注意  
Meridian Board Type.Kには未接続のピン穴を複数設けてあり, 背面からマイコンの入出力と半田付けするすることでIOポートとして利用可能です.その際の注意点を以下にメモします.  
- ESP32のRX0, TX0はPCとのUSBシリアルで使用されています.  
- ESP32のGPIO6-11は内部フラッシュとの接続されておりIOとしては使用できないようです.  
- SPIの機器追加がうまくいかない場合は, 機器側の信号線をプルアップすることで動作が安定することがあります.  
  
####  ICSサーボの取得値のゆれ  
近藤科学のICS通信では, 取得するデータが常時小刻みに揺れています.
このためUnityやRviz,Meridian Consoleで表示した際にも, ピクピクと揺れます.  
(表示側である程度の揺れを吸収する場合もあります.)  

####  Meridian Board でサーボのデータが途切れ途切れになる  
USBバスパワーのみで動作させている場合など, サーボへの電力供給が足りていない場合, サーボがリセットを繰り返すことにより返信データが途切れやすくなります. ボードに正しく電源を接続してください.  

####  Meridian Board でサーボが全く反応しない  
サーボ接続時にコネクタの方向を間違えることで, 半二重回路が破損します.  
万が一回路が破損した場合には,3系統の回路で代用するか,修理用のキット(BOOTHで販売)で回路を交換してください.    

####  9軸センサのBNO055がうまく動作しない  
Teensy4.0との相性問題で,通信中に正しいデータが取得できなくなります.改善方法を探っています.  
