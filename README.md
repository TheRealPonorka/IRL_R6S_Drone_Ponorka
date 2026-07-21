# Tom Clancy's Rainbow Six: Siege - Recon Drone by Ponorka
This is a project is inspired by the fictional tool called Recon drone from game Tom Clancy's Rainbow Six: Siege.

I have built a similar drone, which was based on another github project https://github.com/hemrobotics/recon-drone and I was wondering: "Can this drone be really a fully operational 2 wheeled remote controlled drone?" This github repo should be the answer for this.

# WIP - Work In Progress
This repo is still in progress. Right now 21/07/2026 the PCB is ready and I am waiting for the supplies so I can build the prototype and start coding everything around it. Mostly this will be a vibe coding, as I am really a beginner in any coding other than PowerShell :D So a lot of AI coding will be involved in this project. Sorry to everyone, I am just an enthusiast gamer in this case and in my professional life a SysAdmin :D

Slowly but there will be updates here to add the details about this project.


# Make it but don't sell it!
Keep in mind that this is a hobby project and I am giving it available to everyone, who is interested to build this project. So far I am not planning to make money from this and I would be happy to keep this mindset in a same way for everyone.

Don't make a hustle from this project! If you are interested, but you can't build it by yourself, ask someone, give the person all the necessary HW and pay him for the worktime, but do not sell it!

Thanks for understanding.


# Now the "FUN" Part
Okay, so if you came to this place, you probably want to build your own Recon drone (or maybe just get an inspiration, I don't know :D ).
I will do my best to explain everything around so the build would be easier to complete :)

The schematics and Parts
Took me some time to get back to this knowledge, as I did not work on any electronics for several years. I figured out some basic stuffs, which were needed in addition to properly have all the stuffs working together.
As I wanted to add all the possible features what a recon drone have in R6S, I needed think over, what I want to achieve:
1. Self-balancing 2 wheeled robot
2. Camera for reconnaissance
3. Microphone to hear what is happening around (reconnaissance)
4. Addressable RGB LED strip with 6 LEDs to replicate the visuals of the drone

So, how to achieve that?
First we need a microcontroller (MCU) robust enough to handle all the stuffs. After reviewing all the possibilities and cost effectiveness, I decided to go with ESP32-S3 CAM DEVKITC.
For now this is sufficient for everything we need (later on I might rethink the schematics and PCB layout with fully self-built PCB using only the MCU and not the devkit).
For camera module I decided to go with OV3660, as this module has good enough resolution, while keeping the FPS high enough to have a smooth experience.
I have decided to add also an omnidirectional microphone module INMP441 to hear the surroundings.
The motors I have selected are N20 small geared motors operating on 6V with gear reduction 1:30 and built-in HAL encoder. This gives the drone enough torque and power to balance, while keeping the voltage on lower side (more on it later)
The motor driver is a DRV8833, there is not much to talk about it, it is a sweet spot for this build. Low cost, low power.
The balancing is solved by GY-521 board with MPU6050, slotted on the main board.
Now here comes the power consumption part. In my previous attempt to build this drone I was happy to handle everything with only one 18650 LiPo battery cell and I wanted to keep it that way. I have found a 18650 battery cell shield, which can handle massive current delivery at 5V (up to 3A). With this solution I can easily power the ESP32, the LED strip, the motor drivers and the HAL sensors. It might not be able to run for tens of hours, but this solution should be sufficient to handle power delivery for several hours.
Anyway, the charging is solved via USB-C connector, which can charge the batter in short time. And if you have a spare 18650 battery cell, you can easily replace it whenever you need inside chasis.
There was a need to lower voltage from the motors HAL sensors. As they are operating on 5V from the battery shield, I needed to add also 4-channel bi-polar level shifter, so we don't fry our MCU with 5V input :)
<img width="1710" height="973" alt="Schematics" src="https://github.com/user-attachments/assets/c875a841-196e-4694-a851-3f170e79b0f7" />

The PCB
After several iterations of how and where the parts should be placed, I finished with this version of 2-layer PCB. The entire PCB size is 49.50mm x 94.80mm.

Already ordered PCB from manufacturer
<img width="761" height="724" alt="PCB" src="https://github.com/user-attachments/assets/7d60eb30-bb9c-4ff4-902a-112311d45625" />

3D overview of the final board
<img width="886" height="838" alt="PCB_3D" src="https://github.com/user-attachments/assets/e2766597-4439-4e22-8c61-6b65dd3e7869" />

# The Code
This is where my knowledge is really behind, and if you will review my code later on, you might find a lot of issues with that.
Majority will be written by AI, as I am really just a beginner in such coding.
This will take some time after I assemble the hardware, but will post details about it.

# The Body
I have found refference details about the drone chasis itself in the past, when I built my first functional version, even tho it could not balance. I CAN'T GUARANTEE IN FINAL RELEASE, THAT IT WILL BE 100% ACCURATE!!!
I'm doing my best to model the drone as close to original as possible, but there might be some visual differences at the end.
