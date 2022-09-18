# Meridian_TWIN
  
Meridian計画はヒューマノイドの制御システムについてのオープンソースプロジェクトです.  
ホビーロボットのデジタルツイン化を簡単に実装することができ, PC上のシミュレーション空間とロボット実機をWIFI経由で10msの更新頻度でデータリンクします.  
    
システムの中核はMeridim配列というロボットの状態データを格納できる軽量で汎用的なデータ配列です.  
このデータ配列がデバイス間を高速に廻ることで, リアルタイムな状態データを共有を可能にします.  
Meridim配列を中間プロトコルとして既存のシステムの間に挟むことで, 複数社のコマンドサーボやセンサ, Unityなどの開発環境, ROSで使用可能な多岐にわたるシミュレーターなどを自由に繋ぎ合わせることができます.  
  
当リポジトリで取り扱う ”Meridian_TWIN" はESP32とTeensy4.0を併用するタイプで, 対応ボードはMeridian Board Type.Kとなります.  
また, ESP32DeckitC単体で動作する簡易バージョンのMeridian_LITE（対応ボードはMeridian Board -LITE-）も開発済みです.  
Meridianは今後も用途に応じて様々なハードウェア, ソフトウェアに対応させていく予定です.  
  
  
# 開発資料  
  
プログラムのフローや配列の定義, ボードのピンアサインについては「document」ディレクトリの中に資料としてまとめています.  
専用ボードの回路図も公開しており, 自作したりブレットボードで再現することが可能です.  
動作可能なサンプルプログラムも当リポジトリ内で公開しています.  
  
全体の仕組みや概要は以下のnoteにまとめています.  
https://note.com/ninagawa123/n/ncfde7a6fc835  
  
  
# System composition  
  
Meridian_TWINは, ハードウェアとして通信用のESP32DevKitC, 制御用のTeensy4.0, それらを連結する専用ボードのMeiridian Boardで構成されます.  
デモは近藤科学のKRSサーボ(通信速度1.25Mbps）に対応しており, Meiridian Board Type.KはKHR-3HV用に搭載することができます.  
PC側はROS1のmelodic/noeticに対応しており, 現在Rvizでの表示が可能です. またUnity(Mac/Win版）でもヒューマノイドの姿勢をリアルタイムに反映させることができます.  
  
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
  
  
## Teensy4.0の準備  
#### PlatformIOでTeensy4.0用のプロジェクトファイルを作成する
PlatformIOで"MeridianTWIN_Teensy40"等の名前で新規プロジェクトを作成し, BoardはTeensy4.0, FrameWorkにはArduinoを選択します.  
  
<img width="400" alt="SS 2373" src="https://user-images.githubusercontent.com/8329123/180238336-6cb5456f-cd9d-43a8-813c-5e792fdfd662.png">  
  
#### ライブラリを導入する
- **TsyDMASPI by hideakitai** (PlatformIO上でインストール可)
- **MPU6050 by Electronic Cats** (PlatformIO上でインストール可)
- **Adafruit BusIO** (PlatformIO上でインストール可)
- **Adafruit Unified Sensor** (PlatformIO上でインストール可)
- **IcsClass_V210** (ICS_Library_for_Arduino_V2_1.zipをDL, 解凍してプロジェクトのLibディレクトリに配置)
  
#### サーボのマウントを設定する
Teensy4.0ソースコードの サーボ設定 の項目で, 各サーボのマウントありなしを変更できます.  
接続しているサーボIDは1に, 接続していないIDは0に設定します.  
サーボのマウント設定により, KHR-3HVのフルセットがなくてもICSサーボが最低１つあればデモをテストすることができます.  
サーボ設定に対し１箇所でも接続されていない箇所があると動作しません.  
  
#### ロボットの姿勢とサーボを設定する  
接続するKRSサーボの通信速度設定をすべて**1.25Mbps**に変更します.  
  
また, サーボの0度状態を下記の姿勢に, サーボの＋回転方向も下図の矢印方向に合わせます.  
左半身および体の中心は下図に順次つつ, 右半身については左半身のミラー方向に回転に合わせます.  
サーボの回転方向は, サーボの内部の設定変更が望ましいですが, Teensy4.0ソースコードの サーボ設定 の項目でも変更できます.  
<img width="600" alt="motorccw" src="https://user-images.githubusercontent.com/8329123/147812253-e6cbe388-f70a-445f-80c0-b4cd899aa15a.png">
  
#### サーボを接続する
<img width="600" src="https://github.com/Ninagawa123/Meridian_TWIN/blob/main/documents/TypeK_pinassign.png">  
こちらのピンアサインを参考に, サーボを接続します.  
  
#### センサーを接続する  
MPU/AHRSセンサをMeridianボードのI2Cピンに接続します.  
今のところキャリブレーション済みのMPU6050(GY-521)のみ対応しています.  
センサーがない場合は, Teensy4.0のソースコード設定でセンサの接続をオフにすることができます.  
  
#### Teensy4.0にソースコードを書き込む
https://github.com/Ninagawa123/Meridian_TWIN/tree/main/Meridian_TWIN_for_Teensy40  
の**src/main.cpp**, **platformio.ini** をPlatformIOのプロジェクトファイルに上書きし, Teensy4.0に書き込みます.  
詳細な設定についてはソースコード内のコメントに記しています.  
  
  
## ESP32DevkitCの準備
PlatformIOで"MeridianTWIN_ESP32"等の名前で新規プロジェクトを作成し, BoardはEspressif ESP32 Dev Module, FrameWorkにはArduinoを選択します.  
  
#### ESP32にライブラリを導入し、修正する
- **ESP32DMASPI by hideakitai バージョン0.1.2**  
アリマーク→QUICK ACCESS→ PIO Home → Open → Libraries の検索窓で「ESP32DMASPI」  
「ESP32DMASPI by Hideaki Tai」を選び、バージョンは0.1.2とする（※0.2.0では動かない） 「Add to Project」でプロジェクトを選択し「Add」ボタンで導入.  
  
- **PS4Controller.h**  
libのインポートなどにルールがあり、下記にまとめました。  
またPS4ライブラリをESP32用に修正する方法もまとめています。  
https://qiita.com/Ninagawa_Izumi/items/d8966092fe2abd3cba79  
  
- **ESP32Wiimote.h**  
https://github.com/hrgraf/ESP32Wiimote  
を前述のPS4リモコンと同様に導入しますが, 階層構造が違うので変更します.  
解凍してできた「ESP32Wiimote-master」をlibに格納し,  
その「ESP32Wiimote-master」の中に「src」というディレクトリを作り,  
ESP32Wiimote.cpp, ESP32Wiimote.h, TinyWiimote.cpp, TinyWiimote.h  
を格納します.  
  
#### ソースコードを修正する
https://github.com/Ninagawa123/Meridian_TWIN/blob/main/Meridian_TWIN_for_ESP32/  
の**src/main.cpp**, **platformio.ini** をPlatformIOのプロジェクトファイルに上書きします.  
通信関係の設定を修正する必要があるので, 下記にて必要項目を調べ, 書き換えます.  
  
##### 接続先のPCのIPアドレスを調べる
windowsのコマンドプロンプトを開き,  
$ ipconfig （Ubuntuの場合は$ ip a もしくは $ ifconfig）  
と入力しコマンド実行します.  
IPv4アドレスが表示されます（192.168.1.xxなど)  
Macの場合は画面右上のwifiマークから”ネットワーク”環境設定...で表示されます.  
  
##### WIFIを設定する  
main.cppの /\* 頻繁に変更するであろう変数 #DEFINE \*/ のところで,  
接続したいWIFIのアクセスポイントのSSIDとパスワードを入力します.  
アクセスポイントは5GHzではなく**2.4GHz**に対応している必要があります.  
また, 先ほど調べた接続先のPCのIPアドレスも記入します.  
  
#### ESP32にソースコードを書き込む  
ここで一度, 更新したファイルを**セーブしESP32に書き込みます**.  
  
#### ESP32のIPアドレスを調べる  
PlatformIOで画面下のコンセントアイコンからシリアルモニタを開き, ESP32DevKitC本体のENボタンを押します.  
wifi接続に成功すると  
> Hello, This is Meridian_TWIN_for_ESP32_20220721.  
> WiFi connected to  => xxxxxxx  
> PC's IP address is  => 192.168.1.xxx  
> ESP32's IP address is  => 192.168.1.xxx  
> ESP32's Bluetooth Mac Address is => xx:xx:xx:xx:xx:xx  
  
と表示され, ESP32本体のIPアドレスが表示されます.このxxの番号をメモしておきます.  
  
#### ESP32にソースコードを書き込む  
ソースコードにESP32のBluetooth Mac Addressを記入します.  
更新したファイルをセーブし, ESP32に書き込みます.  
　　
#### platformio.ini  
上書きしたplatformio.iniでは以下の設定を行なっています.
- platformのバージョン指定
- PCとのSerial通信速度設定を50000に指定
- ライブラリの指定
- OTA（無線経由のプログラム書き込み機能）の無効化によるパーティション拡張
  
#### 各種設定の確認    
他にも, 接続するリモコンやシリアルモニタなどについての設定が可能です.  
各ソースコード内のコメントを参考に適宜変更してください.  
  
これでMeridian Board側の設定は完了です.  
  
  
#  Meridian consoleを実行する  
Meridianで受け取るデータを表示できるコンソールを用意しました.python3が使える環境で実行可能です.  
https://github.com/Ninagawa123/Meridian_console  
![meridian_console](https://user-images.githubusercontent.com/8329123/190897481-e073a30d-e475-40f3-bdf4-cfa3e188bf8f.jpg)
  
#  Unity版デモを実行する
（※Mac/Winで動作を確認.Winではファイアーウォールの設定が必要です.）  
  
###  UnityHubに登録して起動する  
https://github.com/Ninagawa123/Meridian_core/tree/main/Unity_demo  
フォルダ「Unity_demo」の中のMeridian_unity_demo_win_20220210.zipをDLし解凍します.  
UnityHubを開き, ProjectsのADDで解凍済みの「Meridian_unity_demo_win_20220210」フォルダを指定します.  
UnityHubに登録されたらプロジェクトを起動します.（Unityのバージョンは**2020.3.25f1(LTS)** です.） 
  
###  UnityのソースコードのIPアドレスを書き換える
画面下の「Project」→「Assets」→「Script」よりUdp_handler_sendをダブルクリックして開きます（VScodeなどが立ち上がります）  
ソースコード9行目のconst string HOST = "192.168.1.xx"; にESP32DevKitCのIPアドレスを記入し, セーブします.

###  ESP32のIPアドレスを書き換える
これまでの手順で設定済みの場合はそのままでOKです.
Meridian_core_for_ESP32_PathThrough.inoの93行目にUnityを使うPCのIPアドレスを入力します.

### Windowsの場合はファイアーウォールを設定する
Windowsスタートメニュー→「設定」→「更新とセキュリティ」→「Windowsセキュリティ」→「ファイアーウォールとネットワーク保護」→「詳細設定」→「受信の規則」の一覧から「Unity 2020.3.25f1 Editor」の「パブリック」となっているものを選択しダブルクリック.「接続を許可する」にチェックを入れOKする.

###  UnityとMeridianボードを起動する
Unityを起動した後, Meridianボードを起動します.
通信が成立していればロボットの関節にシンクロして画面の中のモデルが動きます.

画面の「Send」にチェックを入れるとUnityからロボットを操作することができます.
スライドバーに対応したサーボが動きます.またスライドバーの上のテキストボックスに直接数値を入力することができます.
数値の単位はdegreeとなります.

「Action」にチェックを入れるとUnityからロボットにサンプルのモーションを送信します.
左半身の各関節角度をsinカーブで増減させます.
このモーションはParamMaster.csの160行目-167行目で設定しているので, 適宜書き換えてお試しください.

  
# ROS版デモを実行する
  
### ROS noeticの導入
お手持ちの環境にROSを導入してください.  
Raspberry pi4でROS-noeticを導入する手順については下記にまとめました.
https://qiita.com/Ninagawa_Izumi/items/e84e9841f7a048832fcc  
  
### URDFの表示テスト
https://github.com/Ninagawa123/roid1_urdf  
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
さらに「->ESP32」にチェックを入れると, ロボットのサーボを手で動かした時にロボットにも反映されます.  
  
また, 「->ROS1」「<-DEMO」にチェックを入れると, 画面内のロボットがサインカーブで構成されたダンスのデモを行います.  
ここでさらに「ESP32<-」「Power」にもチェックを入れると, ロボットのサーボにパワーが入り, 画面と同じ動きを実機で再現します.  
  
  
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
  
####  Meridian Board(2022.07.21)
- 各経路で, 0.1%以下の通信エラーが発生します.Meridianのシステムでは通信エラー時はすぐに次のデータを使用することでこのエラーをフォローしています.
- PS4コントローラの接続時, データスキップが5%~10%ほど生じます.
- Wiiコントローラの接続時はデータのスキップはほとんど発生しませんが, 初回の接続が確立しない場合が多いです.

####  Meridian console版(2022.05.04)
Meridian Consoleで読み取った関節データをROS1に出力し, rosbagでjointstateを記録することは可能のようですが, rosbug playとした場合にMeridian Console経由で再生しようとするとカクツキが生じています.改善すべく, 原因を探っています. 

####  Meridian Board Type.K のフリーピン結線時の注意  
Meridian Board Type.Kには未接続のピン穴を複数設けてあり, 背面からマイコンの入出力と半田付けするすることでIOポートとして利用可能です.その際の注意点を以下にメモします.  
- ESP32のRX0, TX0はPCとのUSBシリアルで使用されています.  
- ESP32のGPIO6-11は内部フラッシュとの接続されておりIOとしては使用できないようです.  
