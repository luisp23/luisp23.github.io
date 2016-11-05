---
layout: default
title: About
---


The purpose of this project is to implement automated control systems using Arduino micro-controllers to optimize plant growth in a greenhouse environment. We are currently developing three main control systems: irrigation control, temperature control, and humidity control. The goal is to allow these control systems to easily adapt to different plants' requirements. I am currently in charge of developing the irrigation control. Code, schematics, and other materials I am using can be found in my github page.

I designed an irrigation system that distributes water from one source to 8 different levels using 12V solenoid valves. These levels are the shelves inside our greenhouse. Using an Arduino micro-controller, we will control the flow of water to a specific level based on the plant's need. Each level will have its own solenoid valve, placed on the shelf, inside the greenhouse. Each of the solenoid valves will be attached to a central water distribution adapter. This adapter will take in water from one source and distribute it to each valve through plastic tubing. The water flow will be blocked by the valve (as it is normally closed) until the valve receives 12V, thus opening it. On the output side of the valve will be a drip irrigation system distributing water to different areas of the soil within the level. The valves will sit on each shelf, inside the greenhouse, from where wire will be routed out of the greenhouse to the box holding the electronics.




 The big idea of this system is that each level would hold different types of plants, thereby having different water requirements. My code can be easily modify to adjust for these requirements.
 The base requirement of each level is that it be watered every three hours, given that the soil is dry. To check whether the soil is dry, I use humidity sensors, attached to the soil at each level. When my code checks the moisture for a given level, it decides whether to turn on the valve or not. If it does, the valve opens for a given time period, adequately watering the plants. It then goes on to the next level and repeats. Once every level has been checked, it repeats the process every three hours, giving the soil time to dry up. This allows resources to be used more effectively. We also don't want to be continuously checking for the soil's moisture, as the current sensor can rapidly corrode doing this. Wires will be routed from inside the greenhouse to the electronics box.





The first step in this project was to design the valve controller board whose purpose is to control the 12V solenoid valves. The Arduino's 5V digital outputs will trigger the valve on or off (open or closed). As the Arduino's output provides neither the right voltage nor the required current, we will use a TIP120 transistor to control a 12V power source. I drew out the schematic, then designed a PCB layout for this circuit. I then etched the circuit out using the toner transfer and ferric acid method. Once this circuit was complete I was able to solder on the electronics.

Next, using a perforated board, I built an adapter that allowed for easy interfacing between the Arduino micro-controller, and the moisture sensor modules. This consisted of routing wire to specific male and female header pins, allowing me to 'plug and play'.

I also made a valve status indicator (PCB etching) that consists of an LED for each valve. This would be visible outside of the box holding the components.

I am currently working with a fellow mechanical engineering student who is designing some hardware for the project. He is designing the box that will hold these electronics, as well as the central water distribution adapter, and caps to cover the open electronics side of the solenoid valves (as we do not want water to fry our electronics). We will use our school's 3-D printers to 3-D print these parts and assemble them. These parts can also be found on my github.
