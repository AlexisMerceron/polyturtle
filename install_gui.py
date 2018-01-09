#!/usr/bin/env python
from os import system, popen, geteuid
from sys import exit
import subprocess
import curses

# Check if terminal is not too small
rows, columns = popen('stty size', 'r').read().split()
if int(rows) < 28 or int(columns) < 98:
    exit("Your terminal is too small !");

# Check if user is root
if not geteuid() == 0:
    exit("You need root permissions to do this !");

# Get code name OS
result = subprocess.Popen(['lsb_release','-a'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
out,err = result.communicate()
table = out.decode('utf8').replace("\t","\n").split("\n")
description = table[3]
codename = table[7]


# Init screen
screen = curses.initscr()

# Set new configruration
curses.resizeterm(23,98)
curses.cbreak()
curses.noecho()
screen.keypad(1)
curses.curs_set(0)

# Get size of terminal
height, width = screen.getmaxyx()

# Set display
screen.border(0)
screen.addstr( 1, int(( width - 77 ) / 2), "  ____   ____ ____  ___ ____ _____   ___ _   _ ____ _____  _    _     _      ")
screen.addstr( 2, int(( width - 77 ) / 2), " / ___| / ___|  _ \|_ _|  _ \_   _| |_ _| \ | / ___|_   _|/ \  | |   | |     ")
screen.addstr( 3, int(( width - 77 ) / 2), " \___ \| |   | |_) || || |_) || |    | ||  \| \___ \ | | / _ \ | |   | |     ")
screen.addstr( 4, int(( width - 77 ) / 2), "  ___) | |___|  _ < | ||  __/ | |    | || |\  |___) || |/ ___ \| |___| |___  ")
screen.addstr( 5, int(( width - 77 ) / 2), " |____/ \____|_| \_\___|_|    |_|   |___|_| \_|____/ |_/_/   \_\_____|_____| ")
screen.addstr( 7, int(( width - 90 ) / 2) - 1, "Your linux distrib : " + description) 
screen.addstr( 8, int(( width - 90 ) / 2) - 1, "Choice ROS distrib :")
screen.addstr( 9, int(( width - 90 ) / 2), "+--------------------------------+---------------------+---------------------------------+")
screen.addstr(10, int(( width - 90 ) / 2), "|             Name               |    Release date     |          Distrib Linux          |")
screen.addstr(11, int(( width - 90 ) / 2), "+--------------------------------+---------------------+---------------------------------+")
# Set choice
choice = [["| ROS Melodic Morenia            | May, 2018           | Zesty, Yakkety, Xenial, Stretch |","melodic"],
          ["| ROS Lunar Loggerhead           | May 23rd, 2017      | Xenial, Yakkety, Zesty, Stretch |","lunar"],
          ["| ROS Kinetic Kame (Recommended) | May 23rd, 2016      | Xenial, Wily, Jessie            |","kinetic"],
          ["| ROS Jade Turtle                | May 23rd, 2015      | Vivid, Utopic, Trusty           |","jade"],
          ["| ROS Indigo Igloo               | July 22nd, 2014     | Trusty, Saucy                   |","indigo"],
          ["| ROS Hydro Medusa               | September 4th, 2013 | Raring, Quantal, Precise        |","hydro"]]

# Param interaction choice distrib
firstChoice = 12
column = int(( width - 90 ) / 2)
defaultChoice = 2

# Display choice
cursor = defaultChoice
for i in range(0, len(choice)):
    screen.addstr(firstChoice + i, column, choice[i][0]) if i != defaultChoice else screen.addstr(firstChoice + i, column, choice[i][0],curses.A_REVERSE)
screen.addstr(firstChoice + len(choice), column, "+--------------------------------+---------------------+---------------------------------+")
screen.addstr(firstChoice + len(choice), int(( width - 90 ) / 2), "+" + "-"*len(choice[0][0].split("|")[1]))

# Display finish
screen.addstr(height - 3, 10, "< Cancel >")
screen.addstr(height - 3 , width - 10 - 6, "< OK >")

# Interaction choice
c = screen.getch()
while c != 10  and c != 9:
    screen.refresh()
    c = screen.getch()
    if c == curses.KEY_UP and cursor > 0:
        screen.addstr(cursor + firstChoice, column, choice[cursor][0])
        screen.addstr(cursor + firstChoice - 1, column, choice[cursor - 1][0],curses.A_REVERSE)
        cursor = cursor - 1
    elif c == curses.KEY_DOWN and cursor < len(choice) - 1:
        screen.addstr(cursor + firstChoice, column, choice[cursor][0])
        screen.addstr(cursor + firstChoice + 1, column, choice[cursor + 1][0], curses.A_REVERSE)
        cursor = cursor + 1

# Cancel Ok
c = 9
finish = 1
while c != 10:
    if c == 9:
        if finish == 0:
            screen.addstr(height - 3, width - 10 - 6, "< OK >", curses.A_REVERSE)
            screen.addstr(height - 3, 10, "< Cancel >")
            finish = 1
        else:
            screen.addstr(height - 3 , 10, "< Cancel >", curses.A_REVERSE)
            screen.addstr(height - 3 , width - 10 - 6, "< OK >")
            finish = 0
    c = screen.getch()

# Close screen
curses.nocbreak()
screen.keypad(0)
curses.echo()
curses.endwin()

# Install ROS
if finish == 1:
    system("./install_ros.sh " + choice[cursor][1] + " " + codename )
