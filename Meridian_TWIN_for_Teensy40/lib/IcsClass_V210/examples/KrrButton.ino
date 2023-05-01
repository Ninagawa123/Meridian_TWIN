//
//  @file KrrButton.ino
//  @brief KRR get button sample 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2017/12/26
//
//  KRR(受信機)に送られてきているボタンデータを取得します
//  左側で押されたボタンをもとにID:0のサーボモータを動かします。
//  ICSの通信にはHardwareSerialを使います。
//

#include <IcsHardSerialClass.h>

const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 100;

IcsHardSerialClass krs(&Serial,EN_PIN,BAUDRATE,TIMEOUT);  //インスタンス＋ENピン(2番ピン)およびUARTの指定


void setup() {
  // put your setup code here, to run once:
  krs.begin();  //サーボモータの通信初期設定

  
}

void loop() {

   unsigned short buttonData;

   buttonData = krs.getKrrButton();
   if(buttonData != KRR_BUTTON_FALSE)   //ボタンデータが受信できていたら
   {
    switch(buttonData)
    {
      case KRR_BUTTON_UP : //↑ボタンだった時
      {
        krs.setPos(0,7500);
        break;
      }
      case KRR_BUTTON_DOWN : //↓ボタンだった時
      {
        krs.setFree(0);   //移動できないのでフリー
        break;
      }
      case KRR_BUTTON_RIGHT : //→ボタンだった時
      {
        krs.setPos(0,krs.degPos100(9000));
        break;
      }
      case KRR_BUTTON_LEFT  : //←ボタンだった時
      {
        krs.setPos(0,krs.degPos100(-9000));
        break;
      }
      case (KRR_BUTTON_UP | KRR_BUTTON_RIGHT) : //↑→ボタンだった時
      {
        krs.setPos(0,krs.degPos100(4500));
        break;
      }
      case (KRR_BUTTON_UP |KRR_BUTTON_LEFT) : //←↑ボタンだった時
      {
        krs.setPos(0,krs.degPos100(-4500));
        break;
      }
      case (KRR_BUTTON_DOWN |KRR_BUTTON_RIGHT) : //↓→ボタンだった時
      {
        krs.setPos(0,11500);
        break;
      }
      case (KRR_BUTTON_DOWN | KRR_BUTTON_LEFT) : //←↓ボタンだった時
      {
        krs.setPos(0,3500);
        break;
      }
      default:
      {
        //なにもしない
        break;
      }
      
    }   
   }
   else	//失敗していた時の処理
   {
     delay(500);
   }
   
  
    delay(10);    //KRR5は10ms以下の応答にはついていけないので待つ
}
