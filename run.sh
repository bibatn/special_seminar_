#bsub -n 1 -W 00:10 -o result -e /dev/null OMP_NUM_THREADS=1 ./surface 1.pcd 0.09
# http://hpc.cmc.msu.ru/node/243  
# mpisubmit.pl -p 1 --stdout result build/surface LadaVesta_normalized.pcd 0.0008
# mpisubmit.pl -p 1 --stdout result build/surface LadaVesta_normalized_split.pcd 0.0008
mpisubmit.pl -p 1 --stdout result build/surface LadaVesta_normalized_025.pcd 0.0008
