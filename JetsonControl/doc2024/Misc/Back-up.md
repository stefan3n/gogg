### [< back](../GuideForDocumentation.md)

# How to back up your linux system

## Windows
### Once you have installed eveything you need, it would be a good idea to back up the entire system. To do so, 2 SD cards are required (same size) and a Windows device. One SD card is the original, the card on which the system lives. The second card will serve as a back up. When inserting the card, put the system card in READ ONLY mode. The back up card needs to be formatted. To format the card you can use [rufus](https://rufus.ie/en/). The copy process is done by using [Balena Etcher](https://etcher.balena.io/). Select the ```Clone Drive``` option, select the target and source and flash the copy card.

### There is another application that let's you also create *.img* files from SD cards (but we haven't tested it). You can download it from here: [https://sourceforge.net/projects/win32diskimager/](https://sourceforge.net/projects/win32diskimager/).

## Linux
### You can use *dd* for everything, which comes preinstalled with most distributions. With it you can copy from SD to SD or even create an .img file.
### This allows you to also have the image stored on a PC.

### First run ```fdisk -l``` to list all storage devices and identify the names of each SD card (something like /dev/mmcblk0 or /dev/sdX). You will see multiple entries with similar names: mmcblk0p1, mmcvlk0p2 etc. These are partitions; you only need the whole drive: mmcblk0.

### Then run this command to copy an image from one place to another:
```Bash
sudo dd bs=4M if=<input_file> of=<output_file> status=progress
```
### Examples:
```Bash
# flashing an sd card
sudo dd bs=4M if=/home/toycar/backup.img of=/dev/mmcblk0 status=progress

# copying a SD card onto another SD card
sudo dd bs=4M if=/dev/sda3 of=/dev/mmcblk0 status=progress

# making an image of the sd card
sudo dd bs=4M if=/dev/mmcblk0 of=/home/toycar/backup.img status=progress
```

### More detailed info [here](https://raspberrytips.com/create-image-sd-card/?nab=3).