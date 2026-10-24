#!/bin/sh
cd /root/app/k230-levio
./levio_replay-euroc_half --data /dev/shm/LEVIO_PGM_euroc_half/V103 --log-dir /dev/shm/levio_runs/V103  --mode paced --seed 0x12345678 > /dev/shm/levio_runs/V103/run.log 2>&1
echo $? > /dev/shm/levio_runs/V103/EXITCODE
