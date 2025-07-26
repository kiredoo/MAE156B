# Operating Procedure for the Machine  
[Final Report](https://docs.google.com/document/d/1663F3_ueJS9UGW88TEsQu9kWBfpubMaWPt4RremIK-0/edit?tab=t.0)  
The following procedure outlines the operating procedure. 

1. Connect all wires and Arduino Mega 2560 according to master electrical diagram A.7.1 on the final report.  

3. Compile and upload full_workflow.ide sketch in Arduino.ide software.  
4. Open serial monitor at 115200 baud.  
Immediately, this prompt will appear:

<img width="857" height="83" alt="Screenshot 2025-07-26 at 5 15 15 PM" src="https://github.com/user-attachments/assets/563b62ca-288e-49a3-8144-7efa49275248" />  

When you enter ‘s’ in the serial monitor, these messages will appear in sequence as the system executes the workflow. Note that ‘extending’ and ‘retracting’ refer  to the linear actuator gripping and releasing the cages. Steps 1, 2 and 3 are the grab, dump and translate stages of the cycle; steps 4, 5   and 6 are resetting back to the start for the next cycle.  

<img width="638" height="52" alt="Screenshot 2025-07-26 at 5 15 36 PM" src="https://github.com/user-attachments/assets/e20ce5c4-ddc1-45c1-b529-0ba33989b97e" />  

The system has finished a cycle.  

<img width="638" height="171" alt="Screenshot 2025-07-26 at 5 15 54 PM" src="https://github.com/user-attachments/assets/1c3f0fcb-7ddf-4313-98a8-728c6b600b6f" />  
