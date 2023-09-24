#!/bin/bash
cd /home/adam/Desktop/AccumulationSpace
./build/bin/compAccFromMesh /home/adam/Desktop/AccumulationSpace/samples/am_beech2-3.off \
    --autoScaleAcc \
    am_beech2-3.Acc.vol \
    --autoScaleConf \
    am_beech2-3.Conf.vol -r 5