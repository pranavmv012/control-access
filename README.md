# controll-access
In this assignment, you are going to developthe embedded software for controlling access to a very sensitive siteby using the STM32L432 Nucleo-32board.To do so, you develope mbedded software that allows the authorized staff to access the site.Your code needs tofulfill the following criteria:Ten 4-digit authorization codes have been issued foraccessingto thesiteas shown in Table 1.
The software is going to prompt for the passcode before unlocking the facility’s door.
Your software displaysan appropriate message on screen forthe staffaccessingthe sitethrough serial port communicationvia the PuTTy software as an interface,
Your software prompts the staff to enter a 4 digit valueand checks if the 4 digit valueenteredis one of 10 passcodes stored in an array,Your software verifiesif the passcode is one out of the 10 passcode authorized to access the site,oIf so, your code 
Triggers an auditory signal excitingan speaker connectedvia the PA7port,Sends the “Access granted” text message to the PCthroughserial port,
Unlocksthe access door by turning on the RGB LED in green colourvia PB4port.oIfnot, your code 
Triggers another appropriate auditory feedback signal,Sends the “Access denied” text message sendingto the PCthroughserial port,Keepsthe door locked by turning the RGB-LED in red colour via the PB5port.
