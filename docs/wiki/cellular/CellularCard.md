# Celluar Cards

## Overview

Generally cellular cards operate over USB (2.0/3.0 depending on speed).

There are also cellular cards for M.2 or mPCIE slots for laptops these usually
are just a PCIE USB card and the actual cellular module on the same board.
Note that for these form factors the SIM card slot is not on the module and
requires an adapter board to be able to actually insert a SIM.

Adapter boards usually adapt the mPCIE or M.2 to USB or to M.2/mPCIE + SIM slot.
Many adapter boards will have antennas integrated into them.

## Carrier Compatibility

Carrier compability depends on which frequency bands the module supports.

For example Sierra wireless provides a
[comparison](https://www.sierrawireless.com/products-and-solutions/embedded-solutions/networking-modules/)
of their modules showing which bands are supported. If it isn't explicitly
said which carriers are supported, wikipedia lists which frequency bands carriers
have on the carrier's page (E.g. [Rogers](https://en.wikipedia.org/wiki/Rogers_Wireless#Networks)).

## Choice

One consideration that was taken when choosing the card was the IO of the Jetson TX1.
The TX1 has very limited USB ports but still has 1 free PCIE slot which made the decision easier.
If we buy a PCIE based card it is easy to adapt it to USB in the future if the PCIE slot is needed for
another purpose.

Unfortunately there aren't very many manufactuers of these PCIE based cards as they are usually directly
integrated into laptops rather than being sold on their own.

Sierra Wireless seems to be the only OEM which directly sells to individuals.
The EM7565 is a good choice as it supports all carriers and is very fast at 600Mbps down and 150 Mbps up.
The module costs around $300 so its not a cheap purchase, but given the capabilities it seems worthwhile.

EM7565: https://ltefix.com/shop/modems/sierra-wireless-airprime-cards/sierra-wireless-em7565-cat-12-m-2-modem/
https://www.digikey.ca/product-detail/en/sierra-wireless/EM7565_1103520/1645-1036-ND/7801743

The module also needs a adapter to access the sim card.

M.2 to mPCIE + SIM: https://ltefix.com/shop/pcie-m-2/mini-pci-e/mini-pci-e-to-m-2-ngff-key-b-adapter-with-sim-card-slot/

Finally the mPCIE adapter needs to be converted to full size PCIE. One option is from Startech (Kinda expensive)

https://www.startech.com/ca/Cards-Adapters/Slot-Extension/PCI-Express-to-Mini-PCI-Express-Card-Adapter~PEX2MPEX

Alternatively some Wifi Cards already adapt mPCIE to PCIE and have antennas pre-attached for much cheaper:

https://www.amazon.ca/Wireless-Network-Ubit-Dual-Band-Express/dp/B07KWTVCL2

