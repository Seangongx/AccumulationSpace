# Run it to display the Accumulation or Confidence with input mesh
# The 3dVolViewer program provided by DGDGtalTools library
#!/bin/bash
model_name=model-s25 # you can change the modelname to be used (e.g. am_beech2-3(.off))
cd /home/adam/Desktop/AccumulationSpace
3dVolViewer \
    /home/adam/Desktop/AccumulationSpace/confidence.longvol \
    -m 100 \
    -t 50 \
    --displayMesh /home/adam/Desktop/AccumulationSpace/samples/${model_name}.off \
    --colorMesh 194 194 194 100
