# Run it to display the color segmentation based on Accumulation or Confidence cluster with input mesh
# The meshViewer program provided by DGtal library https://dgtal.org/
#!/bin/bash
modelName=spot-s25          # edit the modelname to be used (e.g. am_beech2-3(.off))
sheetName=spot-s25_r10t25   # edit the table name to be used (e.g. am_beech2-3_r10t25(.dat))
mode=2                      # select confidence (2), accumulation (1), and both by default(0) mode
shader=1                    # select HueShader (1) or GradientShader (0) mode
theta=0.78                     # select threshold value for confidence mode
cd /home/adam/Desktop/ExpeAccSegment/build/expe
./segmentFromAcc \
    /home/adam/Desktop/ExpeAccSegment/Samples/spot/${modelName}.off \
    /home/adam/Desktop/ExpeAccSegment/Samples/spot/${sheetName}.dat \
    ${mode} \
    ${shader} \
    ${theta}

transparency=180            # select transparency of the mesh (0-255)
cd /home/adam/Desktop/ExpeAccSegment/Samples/spot
if [ ${mode} == 2 ]; then
#   /home/adam/DGtalTools/build/visualisation/3dSDPViewer \
#       ${sheetName}_SDP.dat \
#       --addMesh ${sheetName}_ConfColored.off \
#       -s 0.5 \
#       -p sphere \
#       --importColors \
#        --customAlphaMesh 180

    /home/adam/DGtalTools/build/visualisation/meshViewer \
        ${sheetName}_ConfColored.off \
        --displaySDP ${sheetName}_SDP.dat \
        --customAlphaMesh ${transparency}
else
    meshViewer \
        ${sheetName}_AccColored.off \
        --displaySDP ${sheetName}.dat \
        --customAlphaMesh ${transparency}
fi
