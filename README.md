micropython-firmata
===================

firmata implementation (in python) for the micropython project and board.
As board not yet working neither is this implementation.
Waiting for h/w and stabilisation of i2c and Serial comms.

Implements 2.1.13 version
* has virtual pin mapping for STMF4 chips which have many more pins than initial reference Arduino platform.
* implements full protocol
    * allows for i2c comms, servo range, capability queries
* does not implement Shift ops