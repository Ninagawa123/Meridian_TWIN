
# IcsClass for Arduino
 ---
## Overview
�ߓ��Ȋw��ICS�@���Arduino��Serial(HardwareSerial)���瓮�������߂̃��C�u�����ł��B
This library is for connecting a Kondo Kagaku ICS device from an Arudino serial(HardwareSerial) port.

## Description
�ߓ��Ȋw���T�[�{���[�^���M�@��ICS�@���Arduino�œ��������߂̃��C�u�����ł��B  
This library is for connecting a Kondo Kagaku servo motor, receiver or other ICS device using Arudino.

�T�[�{���[�^�̈ʒu����(�|�W�V����)�R�}���h��X�g���b�`���̃p�����[�^�ύX��Arduino����ł���悤�ɂȂ�܂��B
It enables servo motor position commands and parameter changes like for �gstretch�h to be performed from Arudino.

��M�@(KRR)�ɐڑ����邱�ƂŁA���M�@(KRC)����̃f�[�^���擾�ł��܂��B
By connecting to a receiver (KRR), data can also be acquired from a transmitter (KRC).

Arduino�����Serial(HardwareSerial)��p���ĒʐM�����܂��B
Communications can be performed from Arudino using a serial port (HardwareSerial).


## Requirement
Arduino Uno (Serial)
Arduino Meag (Serial1 ~ Serial3)

Arduino����A�N�Z�X�ł���IcsHardSerialClass���g���ɂ�IcsClass��IcsBaceClass�������N�ł���悤�ɂ��Ă��������B
To use IcsHardSerialClass, which can be accessed from Arudino, make sure IcsClass�fs IcsBaseClass can be linked.

���̃}�C�R�����g�������ꍇ�́AIcsBaceClass��h��������ƕ֗��ł��B
To use another computer, deriving IcsBaseClass is a convenient approach.


## Usage
�z�z�t�H���_�̃}�j���A�����������������B
For details, refer to the manual (PDF) in the folder provided.

## Licence
Copyright 2018 Kondo Kagaku co.,ltd.
[MIT](http://opensource.org/licenses/mit-license.php)
see to MIT_Licence.txt


## Author
�ߓ��Ȋw�������
Kondo Kagaku co.,ltd.
T.Nobuhara
�ߓ��Ȋw�z�[���y�[�W:(<http://kondo-robot.com/>)
