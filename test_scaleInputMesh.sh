# Run it to scale the input mesh for correspond Accumulation and Confidence display
# The basicEditMesh program provided by DGDGtalTools-contrib library
#!/bin/bash
model_name=model3-chineseDragon # you can change the modelname to be used (e.g. am_beech2-3(.off))
scale_value=25 # you can change the scale value to be used (e.g. 100)
cd /home/adam/Desktop/AccumulationSpace
basicEditMesh \
    /home/adam/Desktop/AccumulationSpace/samples/${model_name}.off \
    --scale ${scale_value}\
    -o /home/adam/Desktop/AccumulationSpace/samples/${model_name}-s${scale_value}.off

# For model-s25.off -- scaled spot.off 25 times
# For mode2.off -- no scale