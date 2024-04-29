# GOAL: Check the radio communication between two nRF boards

## Segger:
  * **UnZIP** then open `DotBot-firmware` file and open project called `nRF52840dk` with Segger
  
  * Tx: **build** then choose between `solution Radio test -> project 'tx_ble' and project 'tx_ieee_802154'` then **load**  on the first nRF (you can change the power line 100)
  * Rx: **build** then choose between `solution Radio test -> project 'rx_ble' and project 'rx_ieee_802154'` then **load**  on the second nRF
  * Blocker: Choose between `solution Radio test -> project 'blocker_tone', 'blocker_tone_constant', project 'blocker_ble' and project 'bocker_ieee_802154'` 
  * For `blocker_tone` choose line  52 between `t_start_ble` and `t_start_ieee` depending on the communication you want to block same line 53 for `t_end_ble` and `t_end_ieee`. You can also change the power line 88. This blocker is synchronize with tx
  * For `blocker_tone_constant` you can change the power line 32. This blocker is not synchronize with tx
  * For `blocker_ble` and `blocker_ieee8021524` choose line  56 between `t_start_ble` and `t_start_ieee` depending on the communication you want to block and change the size packet as mentionned in the comment line 31. You can also change the power line 92. This blocker is synchronize with tx
  * **build** then **load** it on the third nRF

  * **use** the LED to be sure that you have loaded the good code :  LED1 = rx / LED2 = tx / LED3 = Blocker and Blink normal = BLE / Blink slow = 802.15.4 / Blink fast = Tone (only for Blocker) / Always on tone constant (only for Blocker)
  * **Link** `pin 0.08` from the TX to `pin 0.07` from blocker don't need if you are running `blocker_tone_constant` 
  
  ![Démo](../doc/sphinx/_static/images/radio_test_setup.jpg)
    
 ## Python:
  * **install** serial package `pip install pyserial`
  * **install** pydotbot package `pip install pydotbot`
  * **find** the port of your nRF where the RX code is loaded
  * With a text file editor **change** the serialport_rx with the right one
  * **Launch** the script: `dist/scripts/radio_test/radio_test.py`
  * **Follow** the script indication
  * You can end the script by pressing CTRL+C and it will show you the data you ask for

  ![Python](../doc/sphinx/_static/images/radio_test_demo_python.png)

