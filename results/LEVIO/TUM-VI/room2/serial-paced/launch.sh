#!/bin/sh
cd /root/app/k230-levio
./levio_replay-tumvi --data /dev/shm/LEVIO_PGM_tumvi_half/room2 --log-dir /dev/shm/levio_runs/room2  --mode paced --seed 0x12345678 > /dev/shm/levio_runs/room2/run.log 2>&1
echo $? > /dev/shm/levio_runs/room2/EXITCODE
