Pretty much everything happens in main.c 

Signal conditioning phase consists of a voltage divider, diodes to clamp power to the rails, and an op-amp buffer to ensure proper current to the adc. 

I am interested in adding support for negative voltages through a dc bias to signals. I have yet to test the oscilliscopes capabilities thoroughly although it appears it's unable to accurately determine frequency past 150kHz or so.

The rightmost bread board is just a 555 for testing purposes. The middle one contains the signal condition phase and button inputs (which are still rather bouncy). 
The left most connects the STM32 to the screen, an ILI9488. 
The screen is rather slow to refresh, definetly not the best choice for this project. I regret powering the screen from the F411 as the power rails are rather inconsistent now (hence the need for reading Vrefint to infer true power voltage). I am interested in powering them separately in the future. 

The KiCad project folder for the PCB is the folder name Oscilliscope.

![IMG_3269](https://github.com/user-attachments/assets/2417fd9c-2287-414c-98d7-f8bc23083772)
![IMG_3268](https://github.com/user-attachments/assets/c0b5c635-a361-4ef4-89e5-ff83ad5119aa)

<img width="1218" height="818" alt="image" src="https://github.com/user-attachments/assets/0be5b9b4-5f8f-4d4c-a0bb-e1cc9cbf1648" />
<img width="1267" height="736" alt="image" src="https://github.com/user-attachments/assets/f1131e57-2128-499a-a03c-d595808a5e01" />


![IOC](https://github.com/user-attachments/assets/468c2dde-9f9d-4760-8e9b-34a5dcbabdb9)
![Clock](https://github.com/user-attachments/assets/519a8c44-f923-41b5-92d6-90d6a549c380)
