#!/bin/bash
<< 'COMMENT'
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"
COMMENT


# reset memory
python3 -m Startups.reset_memory
echo '====== Shared Memory Reset ======'
echo ''

# delete all the existing screens if any
pkill screen

# read config
demo=`python3 -m Play.config`

# create a background screen named 'bruce'
screen -d -m -S bruce

# create a window named 'gamepad'
screen -S bruce -X screen -t gamepad
screen -S bruce -p gamepad -X stuff 'python3 -m Startups.run_gamepad^M'
echo '====== Gamepad Online ======'
echo ''

# create a window named 'simulation'
screen -S bruce -X screen -t simulation
screen -S bruce -p simulation -X stuff 'python3 -m Startups.run_simulation^M'
sleep 2.0s
echo '====== Simulation Online ======'
echo ''

# create a window named 'estimation'
screen -S bruce -X screen -t estimation
screen -S bruce -p estimation -X stuff 'python3 -m Startups.run_estimation^M'
sleep 1.0s
echo '====== Estimation Online ======'
echo ''

# create a window named 'low_level'
screen -S bruce -X screen -t low_level
if [ $demo = "True" ]
then
  # screen -S bruce -p low_level -X stuff 'python3 -m Play.Demo.low_level^M'
  screen -S bruce -p low_level -X stuff 'Play/Demo/low_level-x86_64^M'
  sleep 1.0s
  screen -S bruce -p low_level -X stuff "y^M"
else
  screen -S bruce -p low_level -X stuff 'python3 -m Play.Walking.low_level^M'
fi
sleep 0.5s
# read -p $'\e[31mATTENTION: Press ENTER to start low-level control!\e[0m'
sleep 0.5s
screen -S bruce -p low_level -X stuff "y^M"
sleep 0.5s
echo '====== Low-Level Controller Online ======'
echo ''

# create a window named 'high_level'
screen -S bruce -X screen -t high_level
if [ $demo = "True" ]
then
  # screen -S bruce -p high_level -X stuff 'python3 -m Play.Demo.high_level^M'
  screen -S bruce -p high_level -X stuff 'Play/Demo/high_level-x86_64^M'
  sleep 1.0s
  screen -S bruce -p high_level -X stuff "y^M"
else
  screen -S bruce -p high_level -X stuff 'python3 -m Play.Walking.high_level^M'
fi
sleep 0.5s
# read -p $'\e[31mATTENTION: Press ENTER to start high-level control!\e[0m'
sleep 0.5s
screen -S bruce -p high_level -X stuff "y^M"
sleep 0.5s
echo '====== High-Level Controller Online ======'
echo ''

# create a window named 'data'
# screen -S bruce -X screen -t data
# screen -S bruce -p data -X stuff 'python3 -m Play.Demo.data_record^M'

# create a window named 'top_level'
# read -p $'\e[31mATTENTION: Press ENTER to start top-level control!\e[0m'
screen -S bruce -X screen -t top_level
if [ $demo = "True" ]
then
  screen -S bruce -p top_level -X stuff 'python3 -m Play.Demo.top_level^M'
else
  screen -S bruce -p top_level -X stuff 'python3 -m Play.Walking.top_level^M'
fi
sleep 0.5s
echo '====== Top-Level Controller Online ======'
echo ''
input='n'
while [ $input != 'y' ]
do
  echo -ne 'Entering Cockpit .\033[0K\r'
  sleep 0.5s
  echo -ne 'Entering Cockpit ..\033[0K\r'
  sleep 0.5s
  echo -ne 'Entering Cockpit ...\033[0K\r'
  sleep 0.5s
  screen -r -p top_level
  sleep 0.2s
  read -p 'Exit? (y/n)' input
done

# terminate the threads
screen -S bruce -p data       -X stuff "^C"
screen -S bruce -p simulation -X stuff "^C"
screen -S bruce -p estimation -X stuff "^C"
screen -S bruce -p low_level  -X stuff "^C"
screen -S bruce -p high_level -X stuff "^C"
screen -S bruce -p top_level  -X stuff "^C"
sleep 2s

# delete the screen
pkill screen