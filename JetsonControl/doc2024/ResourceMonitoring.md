### [< back](./GuideForDocumentation.md)
### ~We used ***htop*** to monitor and close running processes. It proved more useful than previous methods when checking system load, but it's important to note that htop only monitors CPU usage.

---

### ~To resolve lag issues, we used iftop to monitor bandwidth load with the command `iftop -i wlan0`. To further reduce bandwidth load, we used OpenCV's `resize` function on the video feed, which reduced the load and eliminated the lag.

---

### ~We also observed lag when moving further away from Jetson, which we solved by attaching *antennas for wi-fi* to improve signal reception. 