# Arduino Latency Meter

A simple DIY Arduino-based device to measure the input-to-action latency in 3D games.

![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/board_overview.jpg?raw=true)

---

## 1. How it works

### It works differently
* Unlike other common HW latency meters, this device doesn't measure a "click-to-photon" latency, but rather an "aim-to-photon" latency.
* The device presents itself to the PC as a mouse, and sends a mouse movement (not a click!) message.
* Once the message is sent, the device measures the time until the image on screen changes.

### Advantages over other latency metering devices
1. You don't need bullets to measure latency, so you never run out of ammo - you can (quickly) make as much measurements as you wish. As a result, the device can work autonomously to collect unlimited number of measurement samples, reaching a much higher accuracy.
2. The measurement doesn't rely on the muzzle flash animation, which (depending on developer) may be rendered with a delay, producing incorrect results.
3. You can measure latency in games that don't shoot guns (e.g. shoot arrows or explore)
4. You don't need to place the device in the middle of the screen every time. It is unobtrusive enough that you don't even have to take it off.
5. The device requires only one connected wire for operation.
6. The device can even estimate the effective framerate - something the other devices cannot do.
7. And of course, last but not least - **the device is open source, cheap, and easy to make**!

### So what about the "click-to-photon"?

* The device can also measure "click-to-photon" by appropriately modifying the software it's running, if one wanted to.

## 2. Building the device

### a. What you'll need

* Arduino Pro Micro board (e.g. https://www.amazon.com/dp/B01MTU9GOB - 3 for $22.49)
* A photoresistor (e.g. https://www.amazon.com/dp/B00M1PMHO4 - 20 for $5.79)
* Some shrink tubes (e.g. https://www.amazon.com/dp/B072PCQ2LW - a lot for $6.99)
* Some sticky velcro (e.g. https://www.amazon.com/dp/B000TGSPV6 - 4 for $3.47)
* Box of matches or a lighter (e.g. https://www.amazon.com/dp/B00CKFHYHU - 10 for $5.69)

You should be able to make 3 devices for $44.43, or just under $15 plus tax per device...

### b. Prepare the light sensor

* Cut a piece of shrink tube about 2 lengths of the sensor and and insert the sensor into it. Align the edges.
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/prepare_sensor_1.jpg?raw=true)

* Use a lighter (or matches) to shrink the shrink tube (don't apply the flame directly - just hold it near the tube). Bend the leads as shown - make sure that the longer lead is on the right.
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/prepare_sensor_2.jpg?raw=true)

### c. Solder the light sensor to the board

* Insert the light sensor into contacts marked 4 and 6 , bend and solder as shown. Pay attention that the longer lead is still on the right. Do not cut the leads just yet!
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/prepare_board_1.jpg?raw=true)

* Cut only the left lead. Add some solder to the stub to make it nicer. Bend the right lead onto the stub - this will serve as an activation button. The wire should not be touching the contact - there should be a tiny gap.
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/prepare_board_2.jpg?raw=true)

* Note: if you soldered the phototransistor incorrectly (i.e. in reverse) - don't worry, there is a "#define REVERSE_PHOTOTRANSISTOR" in the code that can solve the problem.
 
### d. Final touch

* Add a piece of the loops part of Velcro (the soft part) to the back of the board. I suggest to add a little piece of it to the USB connector as well - it's not really needed there, but it can serve as a reminder that the connector is fragile and to NOT apply torque force to it (when unplugging the cable) - otherwise it WILL break off...
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/prepare_board_3.jpg?raw=true)

## 3. Device placement

- The device should be placed at the top edge of the monitor, approximately in the middle. I mean, it is also possible to place the device in the corner (like I did in the photo at the top) - but it ended up being easier to find suitable measurement scenes if the device was placed in the middle - see a section called "choosing the right scene for measurement" below.
- Attach a piece of Velcro to the top of the monitor bezel to hold the device.
- **IMPORTANT!!!** Be careful with the USB connector on the board - it's rather flimsy and will break off if you try to apply torque force to it. This mostly happens when you try to unstick the device from the Velcro by pulling on the wire towards yourself. :) Consider yourself warned...
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/sensor_placement_1.jpg?raw=true)

## 4. Operating the software

### a. Programming the Arduino board

* Download Arduino IDE from here: https://www.arduino.cc/en/software
* Use it to open the script - ArduinoLatencyMeter.ino
* Connect the board, use Tools -> Port to select the correct port
* Select the "Arduino Micro" board from Tools -> Board -> Arduino AVR Boards -> Arduino Micro.
* A word of caution: this specific board can be _slightly bricked_ by selecting the board type incorrectly - but this shouldn't be possible with the current code. In any case, it can be _unbricked_ as well, but it's a bit of a hassle.

### b. Using the Arduino IDE to interact with the software

* Once the board is programmed, open the serial monitor via Tools -> Serial Monitor. You should see a printout that looks something like this:

![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/welcome_note.png?raw=true)

Here is what this means:
* The latency meter - speedwise - is equivalent to a high-speed camera running at about 55,000fps.
* To start the measurement with a delay (to have enough time to Alt-Tab to the target application) - enter the number of timeout seconds in the input edit box (the one just above the printout).
* An alternative (and more convenient) way to start and stop the measurement is to push the wire on the board to close the circuit. This way you don't need to Alt-Tab.
* It helps to have a secondary monitor - with the Arduino IDE running there - to see the measurements appear in real time.
* Any serial terminal can be used instead of the Arduino IDE - once the board has been programmed.

### c. Choosing the right scene for measurement

* To measure the latency, the device will be emulating mouse movements in the right-left directon, moving the mouse back and forth by a small amount.
* To make sure the device detects the image change, it is recommended to put the sensor next to a contrast vertical edge.
* See the "device placement" above - the sensor is pointing to the darker leaf, and after moving right it will be pointing to the brighter sky.
* Make sure the place where the sensor is pointing is more-or-less static - so the only reason for the image under the sensor to change would be the movement of the mouse.

### d. Interpreting the results
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/measurements_explanation.png?raw=true)

### e. Understanding the "estimated framerate"

Lets start with the obvious first: there is no direct way to measure the framerate by observing the screen, especially if we are only observing a single point on screen.
That is true for a mostly static image, or for an animation we have no control over. But in our case we DO have control over the animation - we are causing the image to move right and left.

As it turns out, the latency measurements are inherently "noisy" - i.e. they have a "built-in" measurement error (because physics), and the magnitude of that error depends on - you guessed it - the framerate!
So by analyzing the latency measurements and measuring how "noisy" they are (i.e. measuring the standard deviation), we can conclude what the framerate of the game must be.

A few caveats:
* It works pretty well under "ideal conditions" - meaning that the game is running fullscreen exclusive, the monitor is a VRR monitor, and the game's framerate is withing the VRR range of the monitor.
* Even so, the estimated framerate will be in most cases a few percent lower than the actual framerate - because our measurements are not the only source of randomness - there is also the game logic itself and things happening in the operating system.
This additional randomness increases the standard deviation of the measurements (a bit), and causes the estimated framerate to be a bit lower.
* Things start getting interesting when the framerate of the game is beyond the VRR range of the monitor. In this case the estimated "effective framerate" will be lower - even lower than the supported VRR range of the monitor,
and the average latency will become higher - which is only fair - you won't be getting a response as quickly as you could have been.
* If you ignore tearing, the "effective framerate" seems to be a good estimate of how "fast on average" your system is rendering.

### f. Also, it blinks!

To assist with verifying the measurement results using a high-speed camera (read: sanity check), a red LED lights up immediately after the USB command to move the mouse is sent, and is turned off as soon as the movement is detected.
![](https://github.com/EugeneFainstain/assets/blob/main/ArduinoLatencyMeter_photos/board_placement_closeup.jpg?raw=true)

### g. A final note...

To achieve the best results, the measurement is always done when going from darker patch to brighter patch (and not on the back stroke), because:

1. The speed of light response of the monitor pigment is not symmetrical going from dark to bright vs from bright to dark.
2. Some (many!) monitors have a flickering backlight, which makes only the "dark to bright" measurement direction possible.
