# Run it to generate Accumulation Map and Confidence Map from input mesh
#!/bin/bash
model_name=model-s25 # you can change the modelname to be used (e.g. am_beech2-3(.off))
cd /home/adam/Desktop/AccumulationSpace
./build/bin/compAccFromMesh /home/adam/Desktop/AccumulationSpace/samples/${model_name}.off \
    --autoScaleAcc \
    /home/adam/Desktop/AccumulationSpace/temp/${model_name}.Acc.vol \
    --autoScaleConf \
    /home/adam/Desktop/AccumulationSpace/temp/${model_name}.Conf.vol -r 10 # -r 10 is the radius of the raycast length