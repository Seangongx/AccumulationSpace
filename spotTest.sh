# Run it to display the color segmentation based on Accumulation or Confidence cluster with input mesh
# The meshViewer program provided by DGtal library https://dgtal.org/
#!/bin/bash
segDir=/home/adam/Desktop/ExpeAccSegment/build/expe
viewerDir=/home/adam/DGtalTools/build/visualisation
viewerName=meshViewer
modelDir=/home/adam/Desktop/ExpeAccSegment/Samples
modelName=pi_oak8-30         # edit the modelname to be used (e.g. am_beech2-3(.off))
sheetName=pi_oak8-30_r5t10   # edit the table name to be used (e.g. am_beech2-3_r10t25(.dat))
mode=1                     # select confidence (2), accumulation (1), and both by default(0) mode
shader=1                    # select HueShader (1) or GradientShader (0) mode
theta=0.5                     # select threshold value for confidence mode
cd ${segDir}
./segmentFromAcc \
    ${modelDir}/${modelName}.off \
    ${modelDir}/${sheetName}.dat \
    ${mode} \
    ${shader} \
    ${theta}

transparency=180            # select transparency of the mesh (0-255)
cd ${modelDir}
if [ ${mode} == 2 ]; then
#   /home/adam/DGtalTools/build/visualisation/3dSDPViewer \
#       ${sheetName}_SDP.dat \
#       --addMesh ${sheetName}_ConfColored.off \
#       -s 0.5 \
#       -p sphere \
#       --importColors \
#        --customAlphaMesh 180

    ${viewerDir}/${viewerName} \
        ${sheetName}_ConfColored.off \
        --displaySDP ${sheetName}_SDP.dat \
        --customAlphaMesh ${transparency}
else
#   ${viewerDir}/3dSDPViewer \
#       ${sheetName}_SDP.dat \
#       --addMesh ${sheetName}_AccColored.off \
#       -s 0.5 \
#       -p sphere \
#       --importColors \
#        --customAlphaMesh 180

    ${viewerDir}/${viewerName} \
        ${sheetName}_AccColored.off \
        --displaySDP ${sheetName}_SDP.dat \
        --customAlphaMesh ${transparency}
fi
