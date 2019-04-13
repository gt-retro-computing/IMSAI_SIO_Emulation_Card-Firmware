# IMSAI SIO Emulation Card Software

This is the accompanying software that goes along side the IMSAI SIO emulation card.
This card aims to emulate the original IMSAI SIO-2 card, which includes a dual channel 
Serial interface each powered by an Intel 8251 chip. 

Currently, the revision 1, only supports a subset of the original card's feature. 
the goal is to use this as our temporary development serial card. The eventual goal is to
restore our original serial interface card. 

However, that does not mean we will abandon implementing more features on this card.
The next step (Planned) is to replace the discreet transistor based RS-232 logic level 
shifter with MAX232. And also this should add support for Hardware flow control (RTS/CTS).