# ADXL343_to_TCP_server
## Table of Contents
- Install PDM driver
- Build firmware
- Install Node JS
- Build server  

## Introduction
The formware can collect data fro adxl 343 via I2C.
Data is transferred via TCP from the P2 to a PC with Node.js server awainting connection and data

##install node js and run server

In the folder you should find the files below - if not, download them from the PDM_Microphone repository on github https://github.com/particle-iot/Microphone_PDM/tree/main/more-examples/tcp-audio-server 
├── package-lock.json
├── package.json
└── server.js

cd more-examples/tcp-audio-server
npm install
node server.js --label <myLabel>